#include "jaka_driver/phase5_joint_trajectory.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>

namespace phase5 = jaka_driver::phase5;

namespace
{

phase5::JointSample sample(const double time_s, const double joint_zero)
{
  phase5::JointSample value;
  value.time_from_start_s = time_s;
  value.positions_rad.fill(0.0);
  value.positions_rad[0] = joint_zero;
  return value;
}

void near(const double actual, const double expected, const double tolerance)
{
  assert(std::isfinite(actual));
  assert(std::abs(actual - expected) <= tolerance);
}

std::vector<phase5::JointSample> old_linear_8ms(
  const std::vector<phase5::JointSample> & knots)
{
  std::vector<phase5::JointSample> output;
  const double duration = knots.back().time_from_start_s;
  const std::size_t cycles = static_cast<std::size_t>(
    std::floor(duration / phase5::kInterpolationPeriodS));
  for (std::size_t cycle = 0U; cycle <= cycles; ++cycle) {
    const double time = std::min(
      duration, static_cast<double>(cycle) * phase5::kInterpolationPeriodS);
    auto upper = std::upper_bound(
      knots.begin(), knots.end(), time,
      [](const double value, const phase5::JointSample & knot) {
        return value < knot.time_from_start_s;
      });
    std::size_t lower_index = upper == knots.begin() ? 0U :
      static_cast<std::size_t>(std::distance(knots.begin(), upper) - 1);
    lower_index = std::min(lower_index, knots.size() - 2U);
    const auto & lower = knots[lower_index];
    const auto & upper_knot = knots[lower_index + 1U];
    const double alpha = (time - lower.time_from_start_s) /
      (upper_knot.time_from_start_s - lower.time_from_start_s);
    phase5::JointSample value;
    value.time_from_start_s = time;
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      value.positions_rad[joint] = lower.positions_rad[joint] + alpha *
        (upper_knot.positions_rad[joint] - lower.positions_rad[joint]);
    }
    output.push_back(value);
  }
  if (std::abs(output.back().time_from_start_s - duration) > 1e-12) {
    output.push_back(knots.back());
  } else {
    output.back() = knots.back();
  }
  return output;
}

void test_linear_resampler_preserves_piecewise_linearity()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(0.75, 0.3), sample(1.5, 0.0)};
  const auto generated = phase5::resample_linear(knots, 3U);
  assert(generated.ok);
  assert(generated.message == "linear servo stream generated");
  assert(!generated.used_shape_preserving_fallback);
  near(generated.samples.front().time_from_start_s, 0.0, 0.0);
  near(generated.samples.back().time_from_start_s, 1.5, 0.0);
  near(generated.samples.front().positions_rad[0], 0.0, 0.0);
  near(generated.samples.back().positions_rad[0], 0.0, 0.0);
  // 0.24 s is exactly ten 24-ms servo periods and remains on the first line.
  const auto it = std::find_if(
    generated.samples.begin(), generated.samples.end(),
    [](const phase5::JointSample & value) {
      return std::abs(value.time_from_start_s - 0.24) < 1e-12;
    });
  assert(it != generated.samples.end());
  near(it->positions_rad[0], 0.3 * (0.24 / 0.75), 1e-12);
  assert(!phase5::resample_linear(knots, 0U).ok);
  assert(!phase5::resample_linear(knots, 5U).ok);
}

void test_linear_boundary_rest_transition_guard()
{
  const std::vector<phase5::JointSample> too_aggressive{
    sample(0.0, 0.0), sample(0.96, 2.4)};
  const auto rejected = phase5::resample_linear(too_aggressive, 3U);
  assert(!rejected.ok);
  assert(rejected.message == "linear servo stream diagnostics exceed reasonable bounds");

  const std::vector<phase5::JointSample> moderate{
    sample(0.0, 0.0), sample(0.96, 0.5)};
  const auto accepted = phase5::resample_linear(moderate, 3U);
  assert(accepted.ok);
  assert(accepted.diagnostics.max_abs_acceleration_rad_s2 > 0.0);
  assert(accepted.diagnostics.max_abs_acceleration_rad_s2 <
    phase5::kMaximumAbsAccelerationRadS2);
}

void test_endpoint_and_knot_preservation()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(0.75, 0.3), sample(1.6, 0.55), sample(2.5, 0.2)};
  const auto trajectory = phase5::build_quintic_hermite_trajectory(knots);
  assert(trajectory.ok);
  assert(trajectory.knots.size() == knots.size());
  for (std::size_t index = 0U; index < knots.size(); ++index) {
    const auto evaluated = phase5::evaluate_quintic_hermite(
      trajectory, knots[index].time_from_start_s);
    near(evaluated.time_from_start_s, knots[index].time_from_start_s, 0.0);
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      near(evaluated.positions_rad[joint], knots[index].positions_rad[joint], 1e-12);
    }
  }
  const auto start = phase5::evaluate_quintic_hermite(trajectory, 0.0);
  const auto end = phase5::evaluate_quintic_hermite(trajectory, 2.5);
  for (std::size_t joint = 0U; joint < 6U; ++joint) {
    near(start.velocities_rad_s[joint], 0.0, 1e-12);
    near(start.accelerations_rad_s2[joint], 0.0, 1e-12);
    near(end.velocities_rad_s[joint], 0.0, 1e-12);
    near(end.accelerations_rad_s2[joint], 0.0, 1e-12);
  }
}

void test_c2_continuity_and_symmetric_reversal()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(1.0, 0.5), sample(2.0, 0.0)};
  const auto trajectory = phase5::build_quintic_hermite_trajectory(knots);
  assert(trajectory.ok);
  near(trajectory.knot_velocities_rad_s[1][0], 0.0, 1e-15);
  const auto exact = phase5::evaluate_quintic_hermite(trajectory, 1.0);
  const auto before = phase5::evaluate_quintic_hermite(trajectory, 1.0 - 1e-7);
  const auto after = phase5::evaluate_quintic_hermite(trajectory, 1.0 + 1e-7);
  near(exact.velocities_rad_s[0], 0.0, 1e-12);
  near(before.positions_rad[0], after.positions_rad[0], 1e-8);
  near(before.velocities_rad_s[0], after.velocities_rad_s[0], 1e-6);
  near(before.accelerations_rad_s2[0], after.accelerations_rad_s2[0], 1e-5);
}

void test_finite_and_smoother_than_old_linear()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(1.0, 0.5), sample(2.0, 0.0)};
  const auto generated = phase5::resample_quintic_hermite_8ms(knots);
  assert(generated.ok);
  assert(generated.samples.size() > knots.size());
  for (const auto & value : generated.samples) {
    assert(std::isfinite(value.time_from_start_s));
    for (const double position : value.positions_rad) {
      assert(std::isfinite(position));
      assert(std::abs(position) <= phase5::kMaximumAbsJointRad);
    }
  }
  const auto old_diagnostics = phase5::compute_motion_diagnostics(
    old_linear_8ms(knots));
  assert(generated.diagnostics.max_abs_acceleration_rad_s2 <
    old_diagnostics.max_abs_acceleration_rad_s2);
  assert(generated.diagnostics.rms_acceleration_rad_s2 <
    old_diagnostics.rms_acceleration_rad_s2);
}

void test_shape_preserving_fallback_repairs_local_quintic_overshoot()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(1.0, 0.3), sample(2.0, 0.31), sample(3.0, 0.8)};

  const auto historical = phase5::build_quintic_hermite_trajectory(knots);
  assert(historical.ok);
  double historical_max = -std::numeric_limits<double>::infinity();
  for (std::size_t probe = 0U; probe <= 1000U; ++probe) {
    const double time = 1.0 + static_cast<double>(probe) / 1000.0;
    historical_max = std::max(
      historical_max,
      phase5::evaluate_quintic_hermite(historical, time).positions_rad[0]);
  }
  const double allowance = std::min(
    phase5::kMaximumSegmentOvershootRad, 1e-6 + 0.02 * (0.31 - 0.3));
  assert(historical_max > 0.31 + allowance);

  const auto generated = phase5::resample_quintic_hermite(knots, 3U);
  assert(generated.ok);
  assert(generated.used_shape_preserving_fallback);
  assert(generated.samples.front().positions_rad[0] == 0.0);
  assert(generated.samples.back().positions_rad[0] == 0.8);
  for (std::size_t segment = 0U; segment + 1U < knots.size(); ++segment) {
    const double lower = std::min(
      knots[segment].positions_rad[0], knots[segment + 1U].positions_rad[0]);
    const double upper = std::max(
      knots[segment].positions_rad[0], knots[segment + 1U].positions_rad[0]);
    for (std::size_t probe = 0U; probe <= 256U; ++probe) {
      const double alpha = static_cast<double>(probe) / 256.0;
      const double time = knots[segment].time_from_start_s + alpha *
        (knots[segment + 1U].time_from_start_s - knots[segment].time_from_start_s);
      const auto evaluated = phase5::evaluate_quintic_hermite(generated.trajectory, time);
      assert(evaluated.positions_rad[0] >= lower - 1e-10);
      assert(evaluated.positions_rad[0] <= upper + 1e-10);
    }
  }
  for (const auto & acceleration : generated.trajectory.knot_accelerations_rad_s2) {
    near(acceleration[0], 0.0, 0.0);
  }
}

void test_normal_quintic_path_does_not_use_fallback()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(1.0, 0.2), sample(2.0, 0.4)};
  const auto generated = phase5::resample_quintic_hermite(knots, 3U);
  assert(generated.ok);
  assert(!generated.used_shape_preserving_fallback);
  assert(generated.message == "C2 quintic Hermite servo stream generated");
}

void test_variable_servo_periods_preserve_exact_endpoints()
{
  const std::vector<phase5::JointSample> knots{
    sample(0.0, 0.0), sample(0.055, 0.01)};
  const auto at_8ms = phase5::resample_quintic_hermite(knots, 1U);
  const auto at_16ms = phase5::resample_quintic_hermite(knots, 2U);
  const auto at_24ms = phase5::resample_quintic_hermite(knots, 3U);
  const auto at_32ms = phase5::resample_quintic_hermite(knots, 4U);
  assert(at_8ms.ok);
  assert(at_16ms.ok);
  assert(at_24ms.ok);
  assert(at_32ms.ok);
  assert(at_8ms.samples.size() == 8U);
  assert(at_16ms.samples.size() == 5U);
  assert(at_24ms.samples.size() == 4U);
  assert(at_32ms.samples.size() == 3U);
  near(at_8ms.samples[1].time_from_start_s, 0.008, 1e-15);
  near(at_16ms.samples[1].time_from_start_s, 0.016, 1e-15);
  near(at_24ms.samples[1].time_from_start_s, 0.024, 1e-15);
  near(at_32ms.samples[1].time_from_start_s, 0.032, 1e-15);
  for (const auto *generated : {&at_8ms, &at_16ms, &at_24ms, &at_32ms}) {
    near(generated->samples.front().time_from_start_s, 0.0, 0.0);
    near(generated->samples.back().time_from_start_s, 0.055, 0.0);
    near(generated->samples.front().positions_rad[0], 0.0, 0.0);
    near(generated->samples.back().positions_rad[0], 0.01, 0.0);
  }
  assert(!phase5::resample_quintic_hermite(knots, 0U).ok);
  assert(!phase5::resample_quintic_hermite(knots, 5U).ok);
}

void test_conservative_stream_speed_guard()
{
  std::vector<phase5::JointSample> commands{
    sample(0.0, 0.0), sample(0.008, 0.01), sample(0.016, 0.02)};
  const auto passing = phase5::validate_stream_command_velocity(
    commands, phase5::servo_command_period_s(1U));
  assert(passing.ok);
  assert(passing.observed_max_rad_s < passing.limit_rad_s);

  commands[1].positions_rad[0] = 0.03;
  const auto too_fast = phase5::validate_stream_command_velocity(
    commands, phase5::servo_command_period_s(1U));
  assert(!too_fast.ok);
  assert(too_fast.observed_max_rad_s > too_fast.limit_rad_s);

  commands[1].positions_rad[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!phase5::validate_stream_command_velocity(
    commands, phase5::servo_command_period_s(1U)).ok);

  // The guard must use the scheduled interval, including a shortened final
  // interval, instead of dividing every delta by the nominal command period.
  std::vector<phase5::JointSample> short_interval{
    sample(0.0, 0.0), sample(0.012, 0.05)};
  const auto short_interval_result = phase5::validate_stream_command_velocity(
    short_interval, phase5::servo_command_period_s(3U));
  assert(!short_interval_result.ok);
  assert(short_interval_result.observed_max_rad_s > short_interval_result.limit_rad_s);

  std::vector<phase5::JointSample> missing_cycle{
    sample(0.0, 0.0), sample(0.030, 0.001)};
  const auto missing_cycle_result = phase5::validate_stream_command_velocity(
    missing_cycle, phase5::servo_command_period_s(3U));
  assert(!missing_cycle_result.ok);
  assert(missing_cycle_result.message == "servo stream gap exceeds configured command period");
}

}  // namespace

int main()
{
  test_linear_resampler_preserves_piecewise_linearity();
  test_linear_boundary_rest_transition_guard();
  test_endpoint_and_knot_preservation();
  test_c2_continuity_and_symmetric_reversal();
  test_finite_and_smoother_than_old_linear();
  test_shape_preserving_fallback_repairs_local_quintic_overshoot();
  test_normal_quintic_path_does_not_use_fallback();
  test_variable_servo_periods_preserve_exact_endpoints();
  test_conservative_stream_speed_guard();
  std::cout << "phase5_joint_trajectory_test: PASS\n";
  return 0;
}
