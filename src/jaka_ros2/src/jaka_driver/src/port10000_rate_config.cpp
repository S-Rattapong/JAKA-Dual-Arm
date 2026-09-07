#include "jaka_driver/port10000_rate_config.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <regex>
#include <thread>
#include <stdexcept>
#include <string>

namespace jaka_driver
{
namespace
{

constexpr int kPort10001 = 10001;
constexpr std::size_t kMaximumResponseBytes = 16 * 1024;

class ScopedSocket
{
public:
  explicit ScopedSocket(int fd) : fd_(fd) {}
  ~ScopedSocket()
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }
  ScopedSocket(const ScopedSocket &) = delete;
  ScopedSocket & operator=(const ScopedSocket &) = delete;
  int get() const {return fd_;}

private:
  int fd_;
};

std::string errno_message(const char * operation)
{
  return std::string(operation) + ": " + std::strerror(errno);
}

bool wait_for_socket(int fd, short events, int timeout_ms, std::string & error)
{
  pollfd descriptor{};
  descriptor.fd = fd;
  descriptor.events = events;
  const int poll_result = ::poll(&descriptor, 1, timeout_ms);
  if (poll_result == 0) {
    error = "timed out";
    return false;
  }
  if (poll_result < 0) {
    error = errno_message("poll failed");
    return false;
  }
  if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    error = "socket reported an error";
    return false;
  }
  return (descriptor.revents & events) != 0;
}

}  // namespace

std::string make_port10000_rate_command(int period_ms)
{
  if (period_ms <= 0) {
    throw std::invalid_argument("port10000 period must be positive");
  }
  return "{\"cmdName\":\"set_port10000_delay_ms\",\"port10000_delay_ms\":" +
         std::to_string(period_ms) + "}";
}

bool port10000_rate_response_ok(const std::string & response)
{
  static const std::regex error_pattern(
    R"("errorCode"\s*:\s*(?:"0"|0)(?:\s*[,}]))");
  static const std::regex command_pattern(
    R"("cmdName"\s*:\s*"(?:set_port10000_delay_ms|get_port10000_delay_ms|setOptionalInfoConfig|getOptionalInfoConfig)")");
  return std::regex_search(response, error_pattern) &&
         std::regex_search(response, command_pattern);
}

int port10000_rate_response_period_ms(const std::string & response)
{
  static const std::regex direct_pattern(
    "\"port10000_delay_ms\"\\s*:\\s*(?:\"([0-9]+)\"|([0-9]+))");
  static const std::regex value_pattern(
    "\"value\"\\s*:\\s*(?:\"([0-9]+)\"|([0-9]+))");
  std::smatch match;
  if (std::regex_search(response, match, direct_pattern) ||
      std::regex_search(response, match, value_pattern))
  {
    const std::string token = match[1].matched ? match[1].str() : match[2].str();
    try {
      return std::stoi(token);
    } catch (const std::exception &) {
      return -1;
    }
  }
  return -1;
}

namespace
{

Port10000RateConfigResult transact_port10001(
  const std::string & robot_ip,
  const std::string & command,
  int timeout_ms,
  std::string & response)
{
  if (timeout_ms <= 0) return {false, "timeout must be positive"};
  const int raw_socket = ::socket(AF_INET, SOCK_STREAM, 0);
  if (raw_socket < 0) return {false, errno_message("socket failed")};
  ScopedSocket socket(raw_socket);

  const int original_flags = ::fcntl(socket.get(), F_GETFL, 0);
  if (original_flags < 0 ||
    ::fcntl(socket.get(), F_SETFL, original_flags | O_NONBLOCK) < 0)
  {
    return {false, errno_message("failed to make socket nonblocking")};
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(kPort10001);
  if (::inet_pton(AF_INET, robot_ip.c_str(), &address.sin_addr) != 1) {
    return {false, "robot ip is not a valid IPv4 address"};
  }

  const int connect_result = ::connect(
    socket.get(), reinterpret_cast<sockaddr *>(&address), sizeof(address));
  if (connect_result < 0 && errno != EINPROGRESS) {
    return {false, errno_message("connect failed")};
  }
  if (connect_result < 0) {
    std::string wait_error;
    if (!wait_for_socket(socket.get(), POLLOUT, timeout_ms, wait_error)) {
      return {false, "connect " + wait_error};
    }
    int socket_error = 0;
    socklen_t error_size = sizeof(socket_error);
    if (::getsockopt(socket.get(), SOL_SOCKET, SO_ERROR, &socket_error, &error_size) < 0) {
      return {false, errno_message("getsockopt failed")};
    }
    if (socket_error != 0) {
      return {false, "connect failed: " + std::string(std::strerror(socket_error))};
    }
  }

  std::size_t sent = 0;
  while (sent < command.size()) {
    std::string wait_error;
    if (!wait_for_socket(socket.get(), POLLOUT, timeout_ms, wait_error)) {
      return {false, "write " + wait_error};
    }
    const ssize_t count = ::send(
      socket.get(), command.data() + sent, command.size() - sent, MSG_NOSIGNAL);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (count <= 0) return {false, errno_message("send failed")};
    sent += static_cast<std::size_t>(count);
  }

  response.clear();
  response.reserve(512);
  while (response.size() < kMaximumResponseBytes) {
    std::string wait_error;
    if (!wait_for_socket(socket.get(), POLLIN, timeout_ms, wait_error)) {
      return {false, "read " + wait_error};
    }
    char buffer[1024];
    const ssize_t count = ::recv(socket.get(), buffer, sizeof(buffer), 0);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    if (count <= 0) {
      return {false, count == 0 ? "connection closed before response" : errno_message("recv failed")};
    }
    response.append(buffer, static_cast<std::size_t>(count));
    if (response.find('}') != std::string::npos) break;
  }
  if (!port10000_rate_response_ok(response)) {
    return {false, "controller rejected or malformed port10001 response"};
  }
  return {true, "controller accepted command"};
}

bool query_period_matches(
  const std::string & robot_ip, int desired_period_ms, int timeout_ms,
  bool optional_query, std::string & detail)
{
  const std::string command = optional_query
    ? "{\"cmdName\":\"getOptionalInfoConfig\",\"section\":\"PORTCONFIG\",\"key\":\"port10000_delay_ms\"}"
    : "{\"cmdName\":\"get_port10000_delay_ms\"}";
  std::string response;
  const auto result = transact_port10001(robot_ip, command, timeout_ms, response);
  if (!result.ok) {
    detail = result.message;
    return false;
  }
  const int observed = port10000_rate_response_period_ms(response);
  detail = "observed port10000 period " + std::to_string(observed) + " ms";
  return observed == desired_period_ms;
}

}  // namespace

Port10000RateConfigResult configure_port10000_feedback_rate(
  const std::string & robot_ip,
  int period_ms,
  int timeout_ms)
{
  if (period_ms <= 0) return {false, "port10000 period must be positive"};
  if (timeout_ms <= 0) return {false, "timeout must be positive"};

  std::string response;
  auto set_result = transact_port10001(
    robot_ip, make_port10000_rate_command(period_ms), timeout_ms, response);
  if (set_result.ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::string detail;
    if (query_period_matches(robot_ip, period_ms, timeout_ms, false, detail)) {
      return {true, "verified port10000 feedback period at " + std::to_string(period_ms) + " ms"};
    }
  }

  const std::string persistent_command =
    "{\"cmdName\":\"setOptionalInfoConfig\",\"section\":\"PORTCONFIG\","
    "\"key\":\"port10000_delay_ms\",\"value\":\"" + std::to_string(period_ms) + "\"}";
  set_result = transact_port10001(robot_ip, persistent_command, timeout_ms, response);
  if (!set_result.ok) {
    return {false, "direct rate set was not verified and persistent fallback failed: " + set_result.message};
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  std::string detail;
  if (!query_period_matches(robot_ip, period_ms, timeout_ms, true, detail)) {
    return {false, "persistent port10000 rate could not be verified: " + detail};
  }
  return {true, "verified persistent port10000 feedback period at " + std::to_string(period_ms) + " ms"};
}

}  // namespace jaka_driver
