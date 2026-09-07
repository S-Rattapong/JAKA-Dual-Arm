#include "jaka_driver/port10000_rate_config.hpp"

#include <cassert>
#include <string>

int main()
{
  const std::string command = jaka_driver::make_port10000_rate_command(50);
  assert(command ==
    "{\"cmdName\":\"set_port10000_delay_ms\",\"port10000_delay_ms\":50}");
  assert(jaka_driver::port10000_rate_response_ok(
    "{\"errorCode\":\"0\",\"errorMsg\":\"\","
    "\"cmdName\":\"set_port10000_delay_ms\"}"));
  assert(jaka_driver::port10000_rate_response_ok(
    "{\"cmdName\":\"set_port10000_delay_ms\",\"errorCode\":0}"));
  assert(!jaka_driver::port10000_rate_response_ok(
    "{\"cmdName\":\"set_port10000_delay_ms\",\"errorCode\":-1}"));
  assert(!jaka_driver::port10000_rate_response_ok(
    "{\"cmdName\":\"power_on\",\"errorCode\":0}"));
  assert(jaka_driver::port10000_rate_response_period_ms(
    "{\"errorCode\":0,\"cmdName\":\"get_port10000_delay_ms\",\"port10000_delay_ms\":50}") == 50);
  assert(jaka_driver::port10000_rate_response_period_ms(
    "{\"errorCode\":\"0\",\"cmdName\":\"getOptionalInfoConfig\",\"value\":\"50\"}") == 50);
  return 0;
}
