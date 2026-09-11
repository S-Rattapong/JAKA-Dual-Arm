#include "jaka_driver/operator_state_policy.hpp"
#include <iostream>
using namespace jaka_driver::operator_state;
int main() {
  int failures = 0;
  for (bool valid : {false, true}) for (bool power : {false, true})
  for (bool enabled : {false, true}) for (bool known : {false, true})
  for (bool idle : {false, true}) for (bool active : {false, true})
  for (auto control : {Control::Power, Control::Enable}) for (bool desired : {false, true}) {
    const bool current = control == Control::Power ? power : enabled;
    bool allowed = valid && !active;
    if (control == Control::Power && !desired && enabled) allowed = false;
    if (control == Control::Enable && desired && !power) allowed = false;
    if (current != desired && !desired && (!known || !idle)) allowed = false;
    auto result = evaluate(control, desired, {valid, power, enabled, known, idle}, active);
    if (result.allowed != allowed || result.change != (allowed && current != desired)) ++failures;
  }
  if (failures) std::cerr << failures << " transition matrix failures\n";
  return failures ? 1 : 0;
}
