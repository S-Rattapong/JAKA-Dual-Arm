#pragma once

// Hardware-free policy. Unknown controller state never authorizes a transition.
namespace jaka_driver::operator_state {
enum class Control { Power, Enable };
struct State {
    bool valid;
    bool powered;
    bool enabled;
    bool idle_known;
    bool idle;
};
struct Decision {
    bool allowed;
    bool change;
    const char* message;
};
inline Decision evaluate(Control control, bool desired, const State& state,
                         bool phase5_active) {
    if (phase5_active) return {false, false, "Phase5 owns the SDK control window"};
    if (!state.valid) return {false, false, "Controller power/enable state unavailable"};
    if (control == Control::Power && !desired && state.enabled)
        return {false, false, "Disable robot explicitly before power off"};
    if (control == Control::Enable && desired && !state.powered)
        return {false, false, "Power on before enabling robot"};
    const bool current = control == Control::Power ? state.powered : state.enabled;
    if (current == desired) return {true, false, "Already in requested state"};
    if (!desired && (!state.idle_known || !state.idle))
        return {false, false, "Controller must be idle, in position, and outside drag mode"};
    return {true, true, "Transition permitted"};
}
}  // namespace jaka_driver::operator_state
