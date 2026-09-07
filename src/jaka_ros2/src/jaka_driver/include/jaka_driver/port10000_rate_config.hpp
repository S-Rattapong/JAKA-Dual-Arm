#pragma once

#include <string>

namespace jaka_driver
{

struct Port10000RateConfigResult
{
  bool ok;
  std::string message;
};

std::string make_port10000_rate_command(int period_ms);

bool port10000_rate_response_ok(const std::string & response);

int port10000_rate_response_period_ms(const std::string & response);

Port10000RateConfigResult configure_port10000_feedback_rate(
  const std::string & robot_ip,
  int period_ms,
  int timeout_ms = 500);

}  // namespace jaka_driver
