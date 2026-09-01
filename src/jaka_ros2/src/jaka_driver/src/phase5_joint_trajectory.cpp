#include "jaka_driver/phase5_joint_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace jaka_driver::phase5
{

namespace
{

ValidationResult rejected(const std::string & message)
{
  ValidationResult result;
  result.message = message;
  return result;
}

bool finite_array(const std::array<double, 6> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

double square(const double value)
{
  return value * value;
}

}  // namespace

ValidationResult validate_trajectory(
  const std::vector<double> & time_from_start_s,
  const std::vector<double> & flattened_joint_positions_rad,
  const std::int64_t start_time_unix_ns,
  const std::int64_t now_unix_ns)
{
  if (time_from_start_s.size() < 2U) {
    return rejected("timeline must contain at least two samples");
  }
  if (time_from_start_s.size() > kMaximumInputSamples) {
    return rejected("timeline contains too many samples");
  }
  if (flattened_joint_positions_rad.size() != time_from_start_s.size() * 6U) {
    return rejected("joint_positions_rad size must equal sample_count * 6");
  }
  if (start_time_unix_ns <= 0 || now_unix_ns <= 0) {
    return rejected("host wall-clock timestamps must be positive");
  }
  const std::int64_t lead_ns = start_time_unix_ns - now_unix_ns;
  if (lead_ns < kMinimumStartLeadNs) {
    return rejected("start_time_unix_ns is not safely in the future");
  }
  if (lead_ns > kMaximumStartLeadNs) {
    return rejected("start_time_unix_ns is unreasonably far in the future");
  }
  if (!std::isfinite(time_from_start_s.front()) ||
    std::abs(time_from_start_s.front()) > 1e-9)
  {
    return rejected("timeline must start at 0 seconds");
  }

  ValidationResult result;
  result.samples.reserve(time_from_start_s.size());
  double previous_time = -std::numeric_limits<double>::infinity();
  for (std::size_t sample_index = 0; sample_index < time_from_start_s.size(); ++sample_index) {
    const double sample_time = time_from_start_s[sample_index];
    if (!std::isfinite(sample_time)) {
      return rejected("timeline contains a nonfinite value");
    }
    if (sample_time <= previous_time) {
      return rejected("timeline must be strictly increasing");
    }
    if (sample_time < 0.0 || sample_time > kMaximumDurationS) {
      return rejected("timeline duration is outside the supported range");
    }
    JointSample sample;
    sample.time_from_start_s = sample_time;
    for (std::size_t joint = 0; joint < 6U; ++joint) {
      const double value = flattened_joint_positions_rad[sample_index * 6U + joint];
      if (!std::isfinite(value)) {
        return rejected("joint_positions_rad contains a nonfinite value");
      }
      if (std::abs(value) > kMaximumAbsJointRad) {
        return rejected("joint_positions_rad contains an unreasonable absolute value");
      }
      sample.positions_rad[joint] = value;
    }
    result.samples.push_back(sample);
    previous_time = sample_time;
  }
  if (previous_time <= 0.0) {
    return rejected("trajectory duration must be positive");
  }
  result.ok = true;
  result.message = "trajectory validated";
  result.duration_s = previous_time;
  return result;
}

QuinticHermiteTrajectory build_quintic_hermite_trajectory(
  const std::vector<JointSample> & samples)
{
  QuinticHermiteTrajectory trajectory;
  if (samples.size() < 2U || samples.front().time_from_start_s != 0.0 ||
    samples.back().time_from_start_s <= 0.0)
  {
    trajectory.message = "quintic input must contain at least two increasing knots from zero";
    return trajectory;
  }
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (!std::isfinite(samples[index].time_from_start_s) ||
      !finite_array(samples[index].positions_rad) ||
      (index > 0U &&
      samples[index].time_from_start_s <= samples[index - 1U].time_from_start_s))
    {
      trajectory.message = "quintic input contains invalid knots";
      return trajectory;
    }
  }
  trajectory.knots = samples;
  trajectory.knot_velocities_rad_s.resize(samples.size());
  trajectory.knot_accelerations_rad_s2.resize(samples.size());
  // Global endpoint velocity and acceleration remain value-initialized to zero.
  // Interior velocities use the weighted harmonic mean when the two secants
  // agree in direction.  Reversals therefore receive their natural zero
  // velocity.  Interior acceleration is the centered secant derivative.
  for (std::size_t index = 1U; index + 1U < samples.size(); ++index) {
    const double h_previous = samples[index].time_from_start_s -
      samples[index - 1U].time_from_start_s;
    const double h_next = samples[index + 1U].time_from_start_s -
      samples[index].time_from_start_s;
    for (std::size_t joint = 0; joint < 6U; ++joint) {
      const double previous_slope =
        (samples[index].positions_rad[joint] -
        samples[index - 1U].positions_rad[joint]) / h_previous;
      const double next_slope =
        (samples[index + 1U].positions_rad[joint] -
        samples[index].positions_rad[joint]) / h_next;
      double velocity = 0.0;
      if (previous_slope * next_slope > 0.0) {
        const double previous_weight = 2.0 * h_next + h_previous;
        const double next_weight = h_next + 2.0 * h_previous;
        velocity = (previous_weight + next_weight) /
          (previous_weight / previous_slope + next_weight / next_slope);
      }
      trajectory.knot_velocities_rad_s[index][joint] = velocity;
      trajectory.knot_accelerations_rad_s2[index][joint] =
        2.0 * (next_slope - previous_slope) / (h_previous + h_next);
    }
  }
  trajectory.ok = true;
  trajectory.message = "C2 quintic Hermite trajectory built";
  return trajectory;
}

KinematicSample evaluate_quintic_hermite(
  const QuinticHermiteTrajectory & trajectory, const double target_time_s)
{
  KinematicSample result;
  result.time_from_start_s = target_time_s;
  if (!trajectory.ok || trajectory.knots.size() < 2U ||
    !std::isfinite(target_time_s))
  {
    result.positions_rad.fill(std::numeric_limits<double>::quiet_NaN());
    return result;
  }
  const double clamped_time = std::clamp(
    target_time_s, trajectory.knots.front().time_from_start_s,
    trajectory.knots.back().time_from_start_s);
  result.time_from_start_s = clamped_time;
  auto upper = std::upper_bound(
    trajectory.knots.begin(), trajectory.knots.end(), clamped_time,
    [](const double value, const JointSample & sample) {
      return value < sample.time_from_start_s;
    });
  std::size_t lower_index = 0U;
  if (upper == trajectory.knots.end()) {
    lower_index = trajectory.knots.size() - 2U;
  } else if (upper != trajectory.knots.begin()) {
    lower_index = static_cast<std::size_t>(
      std::distance(trajectory.knots.begin(), upper) - 1);
  }
  const std::size_t upper_index = lower_index + 1U;
  const auto & lower = trajectory.knots[lower_index];
  const auto & upper_knot = trajectory.knots[upper_index];
  const double duration = upper_knot.time_from_start_s - lower.time_from_start_s;
  const double u = std::clamp(
    (clamped_time - lower.time_from_start_s) / duration, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  for (std::size_t joint = 0; joint < 6U; ++joint) {
    const double q0 = lower.positions_rad[joint];
    const double q1 = upper_knot.positions_rad[joint];
    const double v0 = trajectory.knot_velocities_rad_s[lower_index][joint];
    const double v1 = trajectory.knot_velocities_rad_s[upper_index][joint];
    const double a0 = trajectory.knot_accelerations_rad_s2[lower_index][joint];
    const double a1 = trajectory.knot_accelerations_rad_s2[upper_index][joint];
    const double c0 = q0;
    const double c1 = v0 * duration;
    const double c2 = 0.5 * a0 * duration * duration;
    const double position_residual = q1 - c0 - c1 - c2;
    const double velocity_residual = v1 * duration - c1 - 2.0 * c2;
    const double acceleration_residual = a1 * duration * duration - 2.0 * c2;
    const double c3 = 10.0 * position_residual - 4.0 * velocity_residual +
      0.5 * acceleration_residual;
    const double c4 = -15.0 * position_residual + 7.0 * velocity_residual -
      acceleration_residual;
    const double c5 = 6.0 * position_residual - 3.0 * velocity_residual +
      0.5 * acceleration_residual;
    result.positions_rad[joint] = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5;
    result.velocities_rad_s[joint] =
      (c1 + 2.0 * c2 * u + 3.0 * c3 * u2 + 4.0 * c4 * u3 + 5.0 * c5 * u4) /
      duration;
    result.accelerations_rad_s2[joint] =
      (2.0 * c2 + 6.0 * c3 * u + 12.0 * c4 * u2 + 20.0 * c5 * u3) /
      square(duration);
    result.jerks_rad_s3[joint] =
      (6.0 * c3 + 24.0 * c4 * u + 60.0 * c5 * u2) /
      (duration * duration * duration);
  }
  return result;
}

MotionDiagnostics compute_motion_diagnostics(
  const std::vector<JointSample> & samples)
{
  MotionDiagnostics diagnostics;
  diagnostics.sample_count = static_cast<std::uint64_t>(samples.size());
  if (samples.size() < 2U) {
    return diagnostics;
  }
  std::vector<std::array<double, 6>> velocities(samples.size());
  std::vector<std::array<double, 6>> accelerations(samples.size());
  std::vector<std::array<double, 6>> jerks(samples.size());
  for (std::size_t index = 1U; index < samples.size(); ++index) {
    const double dt = samples[index].time_from_start_s - samples[index - 1U].time_from_start_s;
    if (!(dt > 0.0) || !std::isfinite(dt)) {
      diagnostics.max_abs_velocity_rad_s = std::numeric_limits<double>::infinity();
      return diagnostics;
    }
  }
  const auto position_derivative = [&](const std::size_t index, const std::size_t joint) {
      const std::size_t lower = index == 0U ? 0U : index - 1U;
      const std::size_t upper = index + 1U < samples.size() ? index + 1U : index;
      return (samples[upper].positions_rad[joint] - samples[lower].positions_rad[joint]) /
        (samples[upper].time_from_start_s - samples[lower].time_from_start_s);
    };
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      velocities[index][joint] = position_derivative(index, joint);
    }
  }
  const auto array_derivative = [&](const std::vector<std::array<double, 6>> &values,
      const std::size_t index, const std::size_t joint) {
      const std::size_t lower = index == 0U ? 0U : index - 1U;
      const std::size_t upper = index + 1U < samples.size() ? index + 1U : index;
      return (values[upper][joint] - values[lower][joint]) /
        (samples[upper].time_from_start_s - samples[lower].time_from_start_s);
    };
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      accelerations[index][joint] = array_derivative(velocities, index, joint);
    }
  }
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      jerks[index][joint] = array_derivative(accelerations, index, joint);
    }
  }
  double velocity_squares = 0.0;
  double acceleration_squares = 0.0;
  double jerk_squares = 0.0;
  std::uint64_t value_count = 0U;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      const double velocity = velocities[index][joint];
      const double acceleration = accelerations[index][joint];
      const double jerk = jerks[index][joint];
      diagnostics.max_abs_velocity_rad_s = std::max(
        diagnostics.max_abs_velocity_rad_s, std::abs(velocity));
      diagnostics.max_abs_acceleration_rad_s2 = std::max(
        diagnostics.max_abs_acceleration_rad_s2, std::abs(acceleration));
      diagnostics.max_abs_jerk_rad_s3 = std::max(
        diagnostics.max_abs_jerk_rad_s3, std::abs(jerk));
      velocity_squares += square(velocity);
      acceleration_squares += square(acceleration);
      jerk_squares += square(jerk);
      ++value_count;
    }
  }
  if (value_count > 0U) {
    diagnostics.rms_velocity_rad_s = std::sqrt(velocity_squares / value_count);
    diagnostics.rms_acceleration_rad_s2 = std::sqrt(acceleration_squares / value_count);
    diagnostics.rms_jerk_rad_s3 = std::sqrt(jerk_squares / value_count);
  }
  return diagnostics;
}

double servo_command_period_s(const std::uint8_t servo_step_num)
{
  if (servo_step_num < 1U || servo_step_num > 4U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(servo_step_num) * kBaseServoPeriodS;
}

static void apply_shape_preserving_fallback(QuinticHermiteTrajectory & trajectory)
{
  for (auto & acceleration : trajectory.knot_accelerations_rad_s2) {
    acceleration.fill(0.0);
  }
  for (std::size_t index = 1U; index + 1U < trajectory.knots.size(); ++index) {
    const double hp = trajectory.knots[index].time_from_start_s - trajectory.knots[index - 1U].time_from_start_s;
    const double hn = trajectory.knots[index + 1U].time_from_start_s - trajectory.knots[index].time_from_start_s;
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      const double previous = (trajectory.knots[index].positions_rad[joint] - trajectory.knots[index - 1U].positions_rad[joint]) / hp;
      const double next = (trajectory.knots[index + 1U].positions_rad[joint] - trajectory.knots[index].positions_rad[joint]) / hn;
      if (previous * next <= 0.0) {
        trajectory.knot_velocities_rad_s[index][joint] = 0.0;
      } else {
        const double limit = 2.0 * std::min(std::abs(previous), std::abs(next));
        const double velocity = trajectory.knot_velocities_rad_s[index][joint];
        trajectory.knot_velocities_rad_s[index][joint] = std::copysign(std::min(std::abs(velocity), limit), velocity);
      }
    }
  }
  trajectory.message = "C2 shape-preserving quintic fallback built";
}

ResampleResult resample_quintic_hermite(
  const std::vector<JointSample> & samples,
  const std::uint8_t servo_step_num)
{
  ResampleResult result;
  const double command_period_s = servo_command_period_s(servo_step_num);
  if (!std::isfinite(command_period_s)) {
    result.message = "servo_step_num must be within [1, 4]";
    return result;
  }
  result.trajectory = build_quintic_hermite_trajectory(samples);
  if (!result.trajectory.ok) {
    result.message = result.trajectory.message;
    return result;
  }
  const double duration_s = samples.back().time_from_start_s;
  const auto rebuild_servo_samples = [&]() {
      result.samples.clear();
      const std::size_t full_cycles = static_cast<std::size_t>(
        std::floor(duration_s / command_period_s));
      result.samples.reserve(full_cycles + 2U);
      for (std::size_t cycle = 0U; cycle <= full_cycles; ++cycle) {
        const double time_s = static_cast<double>(cycle) * command_period_s;
        if (time_s > duration_s + 1e-12) break;
        const auto evaluated = evaluate_quintic_hermite(
          result.trajectory, std::min(time_s, duration_s));
        result.samples.push_back(
          JointSample{evaluated.time_from_start_s, evaluated.positions_rad});
      }
      if (result.samples.empty() ||
        std::abs(result.samples.back().time_from_start_s - duration_s) > 1e-12)
      {
        const auto evaluated = evaluate_quintic_hermite(result.trajectory, duration_s);
        result.samples.push_back(
          JointSample{evaluated.time_from_start_s, evaluated.positions_rad});
      } else {
        result.samples.back() = samples.back();
      }
      result.samples.front() = samples.front();
      result.samples.back() = samples.back();
    };
  rebuild_servo_samples();

validate_dense:
  // Validate both the dispatched stream and a denser view of every segment so
  // nonfinite values, unreasonable dynamics, or local overshoot fail closed.
  for (std::size_t segment = 0U; segment + 1U < samples.size(); ++segment) {
    const double start = samples[segment].time_from_start_s;
    const double end = samples[segment + 1U].time_from_start_s;
    for (std::size_t probe = 0U; probe <= 64U; ++probe) {
      const double time = start + (end - start) * static_cast<double>(probe) / 64.0;
      const auto evaluated = evaluate_quintic_hermite(result.trajectory, time);
      if (!finite_array(evaluated.positions_rad) ||
        !finite_array(evaluated.velocities_rad_s) ||
        !finite_array(evaluated.accelerations_rad_s2) ||
        !finite_array(evaluated.jerks_rad_s3))
      {
        result.message = "quintic trajectory generated nonfinite kinematics at segment " +
          std::to_string(segment) + "->" + std::to_string(segment + 1U);
        result.samples.clear();
        return result;
      }
      for (std::size_t joint = 0U; joint < 6U; ++joint) {
        const double lower = std::min(
          samples[segment].positions_rad[joint], samples[segment + 1U].positions_rad[joint]);
        const double upper = std::max(
          samples[segment].positions_rad[joint], samples[segment + 1U].positions_rad[joint]);
        const double overshoot_allowance = std::min(
          kMaximumSegmentOvershootRad,
          1e-6 + 0.02 * (upper - lower));
        if (std::abs(evaluated.positions_rad[joint]) > kMaximumAbsJointRad ||
          evaluated.positions_rad[joint] < lower - overshoot_allowance ||
          evaluated.positions_rad[joint] > upper + overshoot_allowance)
        {
          if (!result.used_shape_preserving_fallback) {
            apply_shape_preserving_fallback(result.trajectory);
            result.used_shape_preserving_fallback = true;
            rebuild_servo_samples();
            goto validate_dense;
          }
          result.message = "quintic trajectory exhibits unacceptable joint overshoot at segment " +
            std::to_string(segment) + "->" + std::to_string(segment + 1U) +
            ", joint J" + std::to_string(joint + 1U) +
            ", q=" + std::to_string(evaluated.positions_rad[joint]) +
            ", endpoint_range=[" + std::to_string(lower) + "," +
            std::to_string(upper) + "]";
          result.samples.clear();
          return result;
        }
        if (std::abs(evaluated.velocities_rad_s[joint]) > kMaximumAbsVelocityRadS ||
          std::abs(evaluated.accelerations_rad_s2[joint]) > kMaximumAbsAccelerationRadS2 ||
          std::abs(evaluated.jerks_rad_s3[joint]) > kMaximumAbsJerkRadS3)
        {
          result.message = "quintic trajectory exceeds reasonable kinematic bounds at segment " +
            std::to_string(segment) + "->" + std::to_string(segment + 1U) +
            ", joint J" + std::to_string(joint + 1U);
          result.samples.clear();
          return result;
        }
      }
    }
  }
  result.diagnostics = compute_motion_diagnostics(result.samples);
  if (!std::isfinite(result.diagnostics.max_abs_velocity_rad_s) ||
    !std::isfinite(result.diagnostics.max_abs_acceleration_rad_s2) ||
    !std::isfinite(result.diagnostics.max_abs_jerk_rad_s3) ||
    result.diagnostics.max_abs_velocity_rad_s > kMaximumAbsVelocityRadS ||
    result.diagnostics.max_abs_acceleration_rad_s2 > kMaximumAbsAccelerationRadS2 ||
    result.diagnostics.max_abs_jerk_rad_s3 > kMaximumAbsJerkRadS3)
  {
    result.message = "final servo stream diagnostics exceed reasonable bounds";
    result.samples.clear();
    return result;
  }
  result.ok = true;
  result.message = result.used_shape_preserving_fallback
    ? "C2 shape-preserving quintic fallback servo stream generated"
    : "C2 quintic Hermite servo stream generated";
  return result;
}

ResampleResult resample_quintic_hermite_8ms(
  const std::vector<JointSample> & samples)
{
  return resample_quintic_hermite(samples, 1U);
}

StreamGuardResult validate_stream_command_velocity(
  const std::vector<JointSample> & samples,
  const double command_period_s,
  const double limit_rad_s)
{
  StreamGuardResult result;
  result.limit_rad_s = limit_rad_s;
  if (samples.size() < 2U || !std::isfinite(command_period_s) ||
    command_period_s <= 0.0 || !std::isfinite(limit_rad_s) || limit_rad_s <= 0.0)
  {
    result.message = "stream guard inputs are invalid";
    return result;
  }
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    if (!std::isfinite(samples[index].time_from_start_s) ||
      !finite_array(samples[index].positions_rad))
    {
      result.message = "servo stream contains nonfinite command values";
      return result;
    }
    if (index == 0U) {
      continue;
    }
    for (std::size_t joint = 0U; joint < 6U; ++joint) {
      const double command_velocity = std::abs(
        samples[index].positions_rad[joint] -
        samples[index - 1U].positions_rad[joint]) / command_period_s;
      if (!std::isfinite(command_velocity)) {
        result.message = "servo stream guard generated a nonfinite velocity";
        return result;
      }
      result.observed_max_rad_s = std::max(
        result.observed_max_rad_s, command_velocity);
      if (command_velocity > limit_rad_s + 1e-12) {
        result.message = "servo stream exceeds conservative command velocity guard";
        return result;
      }
    }
  }
  result.ok = true;
  result.message = "servo stream command velocity guard passed";
  return result;
}

}  // namespace jaka_driver::phase5
