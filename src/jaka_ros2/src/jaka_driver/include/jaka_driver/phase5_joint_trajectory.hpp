#ifndef JAKA_DRIVER__PHASE5_JOINT_TRAJECTORY_HPP_
#define JAKA_DRIVER__PHASE5_JOINT_TRAJECTORY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace jaka_driver::phase5
{

constexpr double kBaseServoPeriodS = 0.008;
constexpr double kInterpolationPeriodS = kBaseServoPeriodS;
constexpr double kMaximumCommandVelocityDegS = 150.0;
constexpr double kMaximumCommandVelocityRadS =
  kMaximumCommandVelocityDegS * 3.14159265358979323846 / 180.0;
constexpr std::int64_t kMinimumStartLeadNs = 1000000000LL;
constexpr std::int64_t kMaximumStartLeadNs = 30000000000LL;
constexpr double kMaximumDurationS = 600.0;
constexpr std::size_t kMaximumInputSamples = 100000;
constexpr double kMaximumAbsJointRad = 8.0;
constexpr double kMaximumAbsVelocityRadS = 12.0;
constexpr double kMaximumAbsAccelerationRadS2 = 100.0;
constexpr double kMaximumAbsJerkRadS3 = 10000.0;
constexpr double kMaximumSegmentOvershootRad = 0.02;

struct JointSample
{
  double time_from_start_s{0.0};
  std::array<double, 6> positions_rad{};
};

struct ValidationResult
{
  bool ok{false};
  std::string message;
  std::vector<JointSample> samples;
  double duration_s{0.0};
};

struct KinematicSample
{
  double time_from_start_s{0.0};
  std::array<double, 6> positions_rad{};
  std::array<double, 6> velocities_rad_s{};
  std::array<double, 6> accelerations_rad_s2{};
  std::array<double, 6> jerks_rad_s3{};
};

struct QuinticHermiteTrajectory
{
  bool ok{false};
  std::string message;
  std::vector<JointSample> knots;
  std::vector<std::array<double, 6>> knot_velocities_rad_s;
  std::vector<std::array<double, 6>> knot_accelerations_rad_s2;
};

struct MotionDiagnostics
{
  std::uint64_t sample_count{0};
  double max_abs_velocity_rad_s{0.0};
  double rms_velocity_rad_s{0.0};
  double max_abs_acceleration_rad_s2{0.0};
  double rms_acceleration_rad_s2{0.0};
  double max_abs_jerk_rad_s3{0.0};
  double rms_jerk_rad_s3{0.0};
};

struct ResampleResult
{
  bool ok{false};
  bool used_shape_preserving_fallback{false};
  std::string message;
  QuinticHermiteTrajectory trajectory;
  std::vector<JointSample> samples;
  MotionDiagnostics diagnostics;
};

struct StreamGuardResult
{
  bool ok{false};
  std::string message;
  double limit_rad_s{kMaximumCommandVelocityRadS};
  double observed_max_rad_s{0.0};
};

ValidationResult validate_trajectory(
  const std::vector<double> & time_from_start_s,
  const std::vector<double> & flattened_joint_positions_rad,
  std::int64_t start_time_unix_ns,
  std::int64_t now_unix_ns);

QuinticHermiteTrajectory build_quintic_hermite_trajectory(
  const std::vector<JointSample> & samples);

KinematicSample evaluate_quintic_hermite(
  const QuinticHermiteTrajectory & trajectory,
  double target_time_s);

MotionDiagnostics compute_motion_diagnostics(
  const std::vector<JointSample> & samples);

double servo_command_period_s(std::uint8_t servo_step_num);

ResampleResult resample_linear(
  const std::vector<JointSample> & samples,
  std::uint8_t servo_step_num);

ResampleResult resample_quintic_hermite(
  const std::vector<JointSample> & samples,
  std::uint8_t servo_step_num);

ResampleResult resample_quintic_hermite_8ms(
  const std::vector<JointSample> & samples);

StreamGuardResult validate_stream_command_velocity(
  const std::vector<JointSample> & samples,
  double command_period_s,
  double limit_rad_s = kMaximumCommandVelocityRadS);

}  // namespace jaka_driver::phase5

#endif  // JAKA_DRIVER__PHASE5_JOINT_TRAJECTORY_HPP_
