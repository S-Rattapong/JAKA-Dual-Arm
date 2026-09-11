#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"

#include "jaka_msgs/msg/robot_msg.hpp"
#include "jaka_msgs/srv/move.hpp"
#include "jaka_msgs/srv/servo_move_enable.hpp"
#include "jaka_msgs/srv/servo_move.hpp"
#include "jaka_msgs/srv/set_user_frame.hpp"
#include "jaka_msgs/srv/set_tcp_frame.hpp"
#include "jaka_msgs/srv/set_payload.hpp"
#include "jaka_msgs/srv/set_collision.hpp"
#include "jaka_msgs/srv/set_io.hpp"
#include "jaka_msgs/srv/get_io.hpp"
#include "jaka_msgs/srv/get_fk.hpp"
#include "jaka_msgs/srv/get_ik.hpp"
#include "jaka_msgs/srv/get_frame_state.hpp"
#include "jaka_msgs/srv/execute_joint_trajectory.hpp"
#include "jaka_msgs/srv/get_execution_status.hpp"
#include "jaka_msgs/srv/clear_error.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include "jaka_driver/phase5_joint_trajectory.hpp"
#include "jaka_driver/operator_state_policy.hpp"

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <algorithm>
#include <map>
#include <chrono>
#include <thread>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <mutex>
#include <numeric>
#include <utility>
#include <vector>
using namespace std;

const double PI = 3.1415926;
//Define variable: the direction that was sent down the last time the jog was called
int jog_index_last = -1; 
//Define variable: number of calls to jog
int jog_count = 0;
//Define variable: save the number of jog calls
int jog_count_temp = 0;
JAKAZuRobot robot;

// Phase-5 trajectory reservation/cancellation is isolated from legacy motion
// services. The worker reuses the single global SDK session above.
mutex phase5_trajectory_mutex;
atomic<bool> phase5_trajectory_cancel_requested{false};
atomic<bool> phase5_sdk_control_window_active{false};
atomic<uint64_t> phase5_telemetry_suppressed_poll_count{0U};
mutex phase5_telemetry_gate_mutex;
bool phase5_trajectory_active = false;
string phase5_active_trajectory_id;
constexpr int32_t kPhase5LegacyForesightMaxBuf = 15;
constexpr double kPhase5LegacyForesightKp = 0.03;
constexpr const char * kPhase5ExecutionContractMarker =
    "PHASE5_DRIVER_CONTRACT=LINEAR_JOINT_SPACE_V2";
struct Phase5ExecutionStatus
{
    bool valid = true;
    int64_t ret = 1;
    string message = string("read-only in-memory Phase-5 execution status; ") +
        kPhase5ExecutionContractMarker;
    string trajectory_id;
    string state = "IDLE";
    bool active = false;
    int64_t start_time_unix_ns = 0;
    int64_t first_dispatch_unix_ns = 0;
    int64_t first_servo_return_unix_ns = 0;
    int64_t last_dispatch_unix_ns = 0;
    int64_t last_servo_return_unix_ns = 0;
    int64_t terminal_time_unix_ns = 0;
    double duration_s = 0.0;
    double elapsed_s = 0.0;
    double progress_0_to_1 = 0.0;
    int64_t sample_index = -1;
    uint64_t sample_count = 0;
    bool commanded_sample_valid = false;
    double commanded_time_from_start_s = 0.0;
    array<double, 6> commanded_joints_rad{};
    string terminal_reason;
    double max_abs_velocity_rad_s = 0.0;
    double rms_velocity_rad_s = 0.0;
    double max_abs_acceleration_rad_s2 = 0.0;
    double rms_acceleration_rad_s2 = 0.0;
    double max_abs_jerk_rad_s3 = 0.0;
    double rms_jerk_rad_s3 = 0.0;
    uint64_t dispatch_sample_count = 0;
    double mean_abs_lateness_ms = 0.0;
    double p95_abs_lateness_ms = 0.0;
    double p99_abs_lateness_ms = 0.0;
    double max_abs_lateness_ms = 0.0;
    double mean_abs_jitter_ms = 0.0;
    double p95_abs_jitter_ms = 0.0;
    double p99_abs_jitter_ms = 0.0;
    double max_abs_jitter_ms = 0.0;
    uint64_t missed_cycle_count = 0;
    uint64_t servo_j_call_sample_count = 0;
    uint64_t servo_j_overrun_count = 0;
    double mean_servo_j_call_duration_ms = 0.0;
    double p95_servo_j_call_duration_ms = 0.0;
    double p99_servo_j_call_duration_ms = 0.0;
    double max_servo_j_call_duration_ms = 0.0;
    uint8_t servo_step_num = 1U;
    double command_period_ms = 8.0;
    string telemetry_mode = "NORMAL";
    uint64_t telemetry_suppressed_poll_count = 0U;
    double stream_guard_limit_rad_s =
        jaka_driver::phase5::kMaximumCommandVelocityRadS;
    double stream_guard_observed_max_rad_s = 0.0;
    string servo_filter_mode = "LEGACY_FORESIGHT";
    int32_t servo_filter_legacy_max_buf = kPhase5LegacyForesightMaxBuf;
    double servo_filter_legacy_kp = kPhase5LegacyForesightKp;
    double servo_filter_lpf_cutoff_hz = 0.0;
    double servo_filter_nlf_max_velocity_deg_s = 0.0;
    double servo_filter_nlf_max_acceleration_deg_s2 = 0.0;
    double servo_filter_nlf_max_jerk_deg_s3 = 0.0;
};
Phase5ExecutionStatus phase5_execution_status;
//SDK interface return status
map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};

// Declare publishers
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tool_position_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_position_pub;
rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr robot_state_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_current_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_monitor_velocity_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_monitor_raw_pub;

bool linear_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    CartesianPose end_pose;
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    // Rpy rpy;
    OptionalCond *option_cond = nullptr;
    end_pose.tran.x = request->pose[0];
    end_pose.tran.y = request->pose[1];
    end_pose.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3], request->pose[4], request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(end_pose.rpy));
    
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // end_pose.rpy.rx = rpy.x();
    // end_pose.rpy.ry = rpy.y();
    // end_pose.rpy.rz = rpy.z();
    
    int ret = robot.linear_move(&end_pose, MoveMode::ABS, TRUE, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "linear_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool joint_move_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request->pose[0];
    joint_pose.jVal[1] = request->pose[1];
    joint_pose.jVal[2] = request->pose[2];
    joint_pose.jVal[3] = request->pose[3];
    joint_pose.jVal[4] = request->pose[4]; 
    joint_pose.jVal[5] = request->pose[5];
    double speed = static_cast<double>(request->mvvelo);
    double accel = static_cast<double>(request->mvacc);
    double tol = 0.5;
    OptionalCond *option_cond = nullptr;

    int ret = robot.joint_move(&joint_pose, MoveMode::ABS, false, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "joint_move has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool jog_callback(const shared_ptr<jaka_msgs::srv::Move::Request> request,
    shared_ptr<jaka_msgs::srv::Move::Response> response)
{
    // 1. Initialization parameters
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;
    
    // 2. Select index   mapping the index and velocity
    //request.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    //request.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_temp = static_cast<float>(request->index) / 2 + 0.1;
    int index = static_cast<int>(index_temp);
    
    // 3. Select coordinates
    switch (request->coord_mode)
    {
        case 0:
            //coordinate system of joints
            coord_type = COORD_JOINT; 
            //Joint Movement Velocity (rad/s)
            move_velocity = request->mvacc;       
            break;
        case 1:
            //Base coordinate system (Cartesian space)
            coord_type = COORD_BASE;
            //movement speed (mm/s)
            move_velocity = request->mvacc;  
            break;
        case 2:
            //Tool coordinate system (Cartesian space)
		    coord_type = COORD_TOOL;
            //movement speed (mm/s)
            move_velocity = request->mvacc;
            break; 
        default:
            RCLCPP_INFO(rclcpp::get_logger("jog_callback"), "Coordinate system input error, please re-enter");
            return true;
    }
    // 4. Determine the direction of velocity 
    //Determine whether robot motion (articulated or Cartesian) is in a positive or negative direction
    if(request->index & 1)
    {
        move_velocity = -move_velocity;
    }
    //5. Conducting jogging
    if (jog_index_last != request->index)
    {   
        int ret = robot.motion_abort();
        if (ret == 0)
        {
            int jog_state = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch(jog_state)
            {
                case 0:
                    response->ret = 1;
                    response->message = "Position is reached";
                    break;
                default:
                    response->ret = jog_state;
                    response->message = "error occurred:" + mapErr[jog_state];
                    break;
            }
        }
        else
        {
            response->ret = ret;
            response->message = "error occurred:" + mapErr[ret];
        }
        jog_index_last = request->index;
    }
    else
    {
        response->ret = 1;
        response->message = "Robot is jogging";
        RCLCPP_INFO(rclcpp::get_logger("jog_callback"), "Robot is jogging");
    }
    jog_count = jog_count + 1;
    return true;
}

bool servo_move_enable_callback(const shared_ptr<jaka_msgs::srv::ServoMoveEnable::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMoveEnable::Response> response)
{
    BOOL enable = request->enable;
    int ret = robot.servo_move_enable(enable);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "servo_move_enable has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_p_callback(const shared_ptr<jaka_msgs::srv::ServoMove::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMove::Response> response)
{
    //speed * 0.008
    CartesianPose cartesian_pose;
    cartesian_pose.tran.x = request->pose[0];
    cartesian_pose.tran.y = request->pose[1];
    cartesian_pose.tran.z = request->pose[2];
    cartesian_pose.rpy.rx = request->pose[3];
    cartesian_pose.rpy.ry = request->pose[4];
    cartesian_pose.rpy.rz = request->pose[5];
    int ret = robot.servo_p(&cartesian_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Servo_p has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_j_callback(const shared_ptr<jaka_msgs::srv::ServoMove::Request> request,
    shared_ptr<jaka_msgs::srv::ServoMove::Response> response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request->pose[0];
    joint_pose.jVal[1] = request->pose[1];
    joint_pose.jVal[2] = request->pose[2];
    joint_pose.jVal[3] = request->pose[3];
    joint_pose.jVal[4] = request->pose[4];
    joint_pose.jVal[5] = request->pose[5];
    int ret = robot.servo_j(&joint_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Servo_j has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

static int64_t host_wall_clock_now_ns()
{
    return chrono::duration_cast<chrono::nanoseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
}

bool get_execution_status_callback(
    [[maybe_unused]] const shared_ptr<jaka_msgs::srv::GetExecutionStatus::Request> request,
    shared_ptr<jaka_msgs::srv::GetExecutionStatus::Response> response)
{
    // P5.11 status is a strict memory snapshot. Do not add robot/JAKA SDK calls.
    lock_guard<mutex> lock(phase5_trajectory_mutex);
    response->valid = phase5_execution_status.valid;
    response->ret = phase5_execution_status.ret;
    response->message = phase5_execution_status.message;
    response->trajectory_id = phase5_execution_status.trajectory_id;
    response->state = phase5_execution_status.state;
    response->active = phase5_execution_status.active;
    response->start_time_unix_ns = phase5_execution_status.start_time_unix_ns;
    response->first_dispatch_unix_ns = phase5_execution_status.first_dispatch_unix_ns;
    response->first_servo_return_unix_ns = phase5_execution_status.first_servo_return_unix_ns;
    response->last_dispatch_unix_ns = phase5_execution_status.last_dispatch_unix_ns;
    response->last_servo_return_unix_ns = phase5_execution_status.last_servo_return_unix_ns;
    response->terminal_time_unix_ns = phase5_execution_status.terminal_time_unix_ns;
    response->duration_s = phase5_execution_status.duration_s;
    response->elapsed_s = phase5_execution_status.elapsed_s;
    response->progress_0_to_1 = phase5_execution_status.progress_0_to_1;
    response->sample_index = phase5_execution_status.sample_index;
    response->sample_count = phase5_execution_status.sample_count;
    response->commanded_sample_valid = phase5_execution_status.commanded_sample_valid;
    response->commanded_time_from_start_s =
        phase5_execution_status.commanded_time_from_start_s;
    response->commanded_joints_rad.assign(
        phase5_execution_status.commanded_joints_rad.begin(),
        phase5_execution_status.commanded_joints_rad.end());
    response->terminal_reason = phase5_execution_status.terminal_reason;
    response->max_abs_velocity_rad_s = phase5_execution_status.max_abs_velocity_rad_s;
    response->rms_velocity_rad_s = phase5_execution_status.rms_velocity_rad_s;
    response->max_abs_acceleration_rad_s2 = phase5_execution_status.max_abs_acceleration_rad_s2;
    response->rms_acceleration_rad_s2 = phase5_execution_status.rms_acceleration_rad_s2;
    response->max_abs_jerk_rad_s3 = phase5_execution_status.max_abs_jerk_rad_s3;
    response->rms_jerk_rad_s3 = phase5_execution_status.rms_jerk_rad_s3;
    response->dispatch_sample_count = phase5_execution_status.dispatch_sample_count;
    response->mean_abs_lateness_ms = phase5_execution_status.mean_abs_lateness_ms;
    response->p95_abs_lateness_ms = phase5_execution_status.p95_abs_lateness_ms;
    response->p99_abs_lateness_ms = phase5_execution_status.p99_abs_lateness_ms;
    response->max_abs_lateness_ms = phase5_execution_status.max_abs_lateness_ms;
    response->mean_abs_jitter_ms = phase5_execution_status.mean_abs_jitter_ms;
    response->p95_abs_jitter_ms = phase5_execution_status.p95_abs_jitter_ms;
    response->p99_abs_jitter_ms = phase5_execution_status.p99_abs_jitter_ms;
    response->max_abs_jitter_ms = phase5_execution_status.max_abs_jitter_ms;
    response->missed_cycle_count = phase5_execution_status.missed_cycle_count;
    response->servo_j_call_sample_count = phase5_execution_status.servo_j_call_sample_count;
    response->servo_j_overrun_count = phase5_execution_status.servo_j_overrun_count;
    response->mean_servo_j_call_duration_ms = phase5_execution_status.mean_servo_j_call_duration_ms;
    response->p95_servo_j_call_duration_ms = phase5_execution_status.p95_servo_j_call_duration_ms;
    response->p99_servo_j_call_duration_ms = phase5_execution_status.p99_servo_j_call_duration_ms;
    response->max_servo_j_call_duration_ms = phase5_execution_status.max_servo_j_call_duration_ms;
    response->servo_step_num = phase5_execution_status.servo_step_num;
    response->command_period_ms = phase5_execution_status.command_period_ms;
    response->telemetry_mode = phase5_execution_status.telemetry_mode;
    response->telemetry_suppressed_poll_count =
        phase5_execution_status.active
        ? phase5_telemetry_suppressed_poll_count.load()
        : phase5_execution_status.telemetry_suppressed_poll_count;
    response->stream_guard_limit_rad_s = phase5_execution_status.stream_guard_limit_rad_s;
    response->stream_guard_observed_max_rad_s = phase5_execution_status.stream_guard_observed_max_rad_s;
    response->servo_filter_mode = phase5_execution_status.servo_filter_mode;
    response->servo_filter_legacy_max_buf = phase5_execution_status.servo_filter_legacy_max_buf;
    response->servo_filter_legacy_kp = phase5_execution_status.servo_filter_legacy_kp;
    response->servo_filter_lpf_cutoff_hz = phase5_execution_status.servo_filter_lpf_cutoff_hz;
    response->servo_filter_nlf_max_velocity_deg_s = phase5_execution_status.servo_filter_nlf_max_velocity_deg_s;
    response->servo_filter_nlf_max_acceleration_deg_s2 = phase5_execution_status.servo_filter_nlf_max_acceleration_deg_s2;
    response->servo_filter_nlf_max_jerk_deg_s3 = phase5_execution_status.servo_filter_nlf_max_jerk_deg_s3;
    return true;
}

struct Phase5ServoFilterConfig
{
    uint8_t mode = 3U;
    int32_t legacy_max_buf = kPhase5LegacyForesightMaxBuf;
    double legacy_kp = kPhase5LegacyForesightKp;
    double lpf_cutoff_hz = 0.0;
    double nlf_max_velocity_deg_s = 0.0;
    double nlf_max_acceleration_deg_s2 = 0.0;
    double nlf_max_jerk_deg_s3 = 0.0;
};

static string phase5_filter_label(const uint8_t mode)
{
    if (mode == 1U) return "LPF";
    if (mode == 2U) return "NLF";
    if (mode == 3U) return "LEGACY_FORESIGHT";
    return "NONE";
}

static bool finite_in_range(const double value, const double minimum, const double maximum)
{
    return isfinite(value) && value >= minimum && value <= maximum;
}

static bool validate_phase5_filter(
    const Phase5ServoFilterConfig &config, string &message)
{
    const bool lpf_zero = config.lpf_cutoff_hz == 0.0;
    const bool legacy_zero = config.legacy_max_buf == 0 && config.legacy_kp == 0.0;
    const bool nlf_zero = config.nlf_max_velocity_deg_s == 0.0 &&
        config.nlf_max_acceleration_deg_s2 == 0.0 &&
        config.nlf_max_jerk_deg_s3 == 0.0;
    if (config.mode == 0U && legacy_zero && lpf_zero && nlf_zero)
    {
        return true;
    }
    if (config.mode == 1U && legacy_zero &&
        finite_in_range(config.lpf_cutoff_hz, 0.1, 100.0) && nlf_zero)
    {
        return true;
    }
    if (config.mode == 2U && legacy_zero && lpf_zero &&
        finite_in_range(config.nlf_max_velocity_deg_s, 0.1, 2000.0) &&
        finite_in_range(config.nlf_max_acceleration_deg_s2, 0.1, 20000.0) &&
        finite_in_range(config.nlf_max_jerk_deg_s3, 0.1, 200000.0))
    {
        return true;
    }
    if (config.mode == 3U && config.legacy_max_buf == kPhase5LegacyForesightMaxBuf &&
        config.legacy_kp == kPhase5LegacyForesightKp && lpf_zero && nlf_zero)
    {
        return true;
    }
    message = "servo filter mode/tuning is invalid or contains nonzero unused fields";
    return false;
}

static int apply_phase5_filter(const Phase5ServoFilterConfig &config)
{
    if (config.mode == 1U)
    {
        return robot.servo_move_use_joint_LPF(config.lpf_cutoff_hz);
    }
    if (config.mode == 2U)
    {
        return robot.servo_move_use_joint_NLF(
            config.nlf_max_velocity_deg_s,
            config.nlf_max_acceleration_deg_s2,
            config.nlf_max_jerk_deg_s3);
    }
    if (config.mode == 3U)
    {
        return robot.servo_speed_foresight(config.legacy_max_buf, config.legacy_kp);
    }
    return robot.servo_move_use_none_filter();
}

static int restore_phase5_legacy_foresight_baseline()
{
    return robot.servo_speed_foresight(
        kPhase5LegacyForesightMaxBuf, kPhase5LegacyForesightKp);
}

static double percentile_ms(vector<double> values, const double percentile)
{
    if (values.empty()) return 0.0;
    sort(values.begin(), values.end());
    const double index = percentile * static_cast<double>(values.size() - 1U);
    const size_t lower = static_cast<size_t>(floor(index));
    const size_t upper = min(lower + 1U, values.size() - 1U);
    const double alpha = index - static_cast<double>(lower);
    return values[lower] + alpha * (values[upper] - values[lower]);
}

static void store_phase5_dispatch_timing(
    const string &trajectory_id,
    const vector<double> &abs_lateness_ms,
    const vector<double> &abs_jitter_ms,
    const vector<double> &servo_j_call_duration_ms,
    const uint64_t missed_cycle_count,
    const uint64_t servo_j_overrun_count)
{
    lock_guard<mutex> lock(phase5_trajectory_mutex);
    if (phase5_active_trajectory_id != trajectory_id) return;
    const auto mean = [](const vector<double> &values) {
        return values.empty() ? 0.0 :
            accumulate(values.begin(), values.end(), 0.0) /
            static_cast<double>(values.size());
    };
    phase5_execution_status.dispatch_sample_count =
        static_cast<uint64_t>(abs_lateness_ms.size());
    phase5_execution_status.mean_abs_lateness_ms = mean(abs_lateness_ms);
    phase5_execution_status.p95_abs_lateness_ms = percentile_ms(abs_lateness_ms, 0.95);
    phase5_execution_status.p99_abs_lateness_ms = percentile_ms(abs_lateness_ms, 0.99);
    phase5_execution_status.max_abs_lateness_ms = abs_lateness_ms.empty() ? 0.0 :
        *max_element(abs_lateness_ms.begin(), abs_lateness_ms.end());
    phase5_execution_status.mean_abs_jitter_ms = mean(abs_jitter_ms);
    phase5_execution_status.p95_abs_jitter_ms = percentile_ms(abs_jitter_ms, 0.95);
    phase5_execution_status.p99_abs_jitter_ms = percentile_ms(abs_jitter_ms, 0.99);
    phase5_execution_status.max_abs_jitter_ms = abs_jitter_ms.empty() ? 0.0 :
        *max_element(abs_jitter_ms.begin(), abs_jitter_ms.end());
    phase5_execution_status.missed_cycle_count = missed_cycle_count;
    phase5_execution_status.servo_j_call_sample_count =
        static_cast<uint64_t>(servo_j_call_duration_ms.size());
    phase5_execution_status.servo_j_overrun_count = servo_j_overrun_count;
    phase5_execution_status.mean_servo_j_call_duration_ms =
        mean(servo_j_call_duration_ms);
    phase5_execution_status.p95_servo_j_call_duration_ms =
        percentile_ms(servo_j_call_duration_ms, 0.95);
    phase5_execution_status.p99_servo_j_call_duration_ms =
        percentile_ms(servo_j_call_duration_ms, 0.99);
    phase5_execution_status.max_servo_j_call_duration_ms =
        servo_j_call_duration_ms.empty() ? 0.0 :
        *max_element(servo_j_call_duration_ms.begin(), servo_j_call_duration_ms.end());
}

struct Phase5SdkCleanupResult
{
    int disable_ret = 0;
    int baseline_restore_ret = 0;
    bool baseline_restore_skipped = false;
};

static Phase5SdkCleanupResult cleanup_phase5_sdk_control_window(
    const bool servo_mode_enabled)
{
    Phase5SdkCleanupResult result;
    if (servo_mode_enabled)
    {
        result.disable_ret = robot.servo_move_enable(FALSE);
    }
    // JAKA filter selection is configured only outside servo mode. If disable
    // failed, do not risk calling a filter-reset API while mode may remain on.
    if (!servo_mode_enabled || result.disable_ret == 0)
    {
        result.baseline_restore_ret = restore_phase5_legacy_foresight_baseline();
    }
    else
    {
        result.baseline_restore_skipped = true;
    }
    {
        lock_guard<mutex> telemetry_gate(phase5_telemetry_gate_mutex);
        phase5_sdk_control_window_active.store(false);
    }
    return result;
}

static void finish_phase5_trajectory(
    const string &trajectory_id, const bool servo_mode_enabled,
    const char *terminal_status)
{
    const Phase5SdkCleanupResult cleanup =
        cleanup_phase5_sdk_control_window(servo_mode_enabled);
    if (cleanup.disable_ret != 0)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("phase5_joint_trajectory"),
            "trajectory_id=%s failed to disable servo mode, error_code=%d",
            trajectory_id.c_str(), cleanup.disable_ret);
    }
    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == trajectory_id)
        {
            string reason = terminal_status;
            if (cleanup.baseline_restore_skipped)
            {
                reason = "SERVO_BASELINE_RESTORE_SKIPPED_AFTER_" + reason;
            }
            else if (cleanup.baseline_restore_ret != 0)
            {
                reason = "SERVO_BASELINE_RESTORE_FAILED_AFTER_" + reason;
            }
            if (cleanup.disable_ret != 0)
            {
                reason = "SERVO_DISABLE_FAILED_AFTER_" + reason;
            }
            if (reason.rfind("CANCELLED", 0) == 0)
            {
                phase5_execution_status.state = "ABORTED";
            }
            else if (reason == "COMPLETED")
            {
                phase5_execution_status.state = "COMPLETED";
            }
            else
            {
                phase5_execution_status.state = "FAILED";
            }
            phase5_execution_status.terminal_reason = reason;
            const int64_t terminal_time_unix_ns = host_wall_clock_now_ns();
            phase5_execution_status.terminal_time_unix_ns = terminal_time_unix_ns;
            if (phase5_execution_status.state == "COMPLETED")
            {
                phase5_execution_status.elapsed_s = phase5_execution_status.duration_s;
                phase5_execution_status.progress_0_to_1 = 1.0;
                if (phase5_execution_status.sample_count > 0U)
                {
                    phase5_execution_status.sample_index =
                        static_cast<int64_t>(phase5_execution_status.sample_count - 1U);
                }
            }
            else if (phase5_execution_status.start_time_unix_ns > 0)
            {
                const double terminal_elapsed_s = max(
                    0.0,
                    static_cast<double>(terminal_time_unix_ns -
                        phase5_execution_status.start_time_unix_ns) / 1e9);
                phase5_execution_status.elapsed_s = min(
                    max(phase5_execution_status.elapsed_s, terminal_elapsed_s),
                    phase5_execution_status.duration_s);
                phase5_execution_status.progress_0_to_1 =
                    phase5_execution_status.duration_s > 0.0
                    ? min(max(
                        phase5_execution_status.elapsed_s /
                        phase5_execution_status.duration_s, 0.0), 1.0)
                    : 0.0;
            }
            // Record terminal authority before clearing the active reservation.
            phase5_execution_status.telemetry_suppressed_poll_count =
                phase5_telemetry_suppressed_poll_count.load();
            phase5_execution_status.active = false;
            phase5_trajectory_active = false;
            phase5_active_trajectory_id.clear();
        }
    }
    RCLCPP_INFO(
        rclcpp::get_logger("phase5_joint_trajectory"),
        "trajectory_id=%s terminal_status=%s",
        trajectory_id.c_str(), terminal_status);
}

static void execute_phase5_joint_trajectory_worker(
    vector<jaka_driver::phase5::JointSample> resampled,
    const int64_t start_time_unix_ns,
    const string trajectory_id,
    const uint8_t servo_step_num)
{
    // accepted=true is returned only after this driver has successfully entered
    // servo mode. This worker owns disabling servo mode on every terminal path.
    const bool servo_mode_enabled = true;
    if (phase5_trajectory_cancel_requested.load())
    {
        finish_phase5_trajectory(trajectory_id, servo_mode_enabled, "CANCELLED_BEFORE_START");
        return;
    }

    const auto absolute_start = chrono::system_clock::time_point(
        chrono::nanoseconds(start_time_unix_ns));
    while (chrono::system_clock::now() < absolute_start)
    {
        if (phase5_trajectory_cancel_requested.load())
        {
            finish_phase5_trajectory(trajectory_id, servo_mode_enabled, "CANCELLED_BEFORE_START");
            return;
        }
        this_thread::sleep_for(chrono::milliseconds(2));
    }

    // The common absolute wall clock is a host-timed release authority only.
    // Once released, steady_clock prevents wall-clock adjustments from changing
    // the configured interpolation cadence. This is not controller hard real-time sync.
    const auto steady_start = chrono::steady_clock::now();
    const double command_period_ms =
        jaka_driver::phase5::servo_command_period_s(servo_step_num) * 1000.0;
    vector<double> abs_lateness_ms;
    vector<double> abs_jitter_ms;
    vector<double> servo_j_call_duration_ms;
    abs_lateness_ms.reserve(resampled.size());
    if (resampled.size() > 1U) abs_jitter_ms.reserve(resampled.size() - 1U);
    servo_j_call_duration_ms.reserve(resampled.size());
    chrono::steady_clock::time_point previous_dispatch;
    double previous_sample_time_s = 0.0;
    bool have_previous_dispatch = false;
    uint64_t missed_cycle_count = 0U;
    uint64_t servo_j_overrun_count = 0U;
    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == trajectory_id)
        {
            phase5_execution_status.state = "RUNNING";
            phase5_execution_status.active = true;
        }
    }
    for (size_t sample_index = 0; sample_index < resampled.size(); ++sample_index)
    {
        const auto &sample = resampled[sample_index];
        const auto sample_offset = chrono::nanoseconds(
            static_cast<int64_t>(llround(sample.time_from_start_s * 1e9)));
        this_thread::sleep_until(steady_start + sample_offset);
        if (phase5_trajectory_cancel_requested.load())
        {
            store_phase5_dispatch_timing(
                trajectory_id, abs_lateness_ms, abs_jitter_ms,
                servo_j_call_duration_ms, missed_cycle_count, servo_j_overrun_count);
            finish_phase5_trajectory(trajectory_id, servo_mode_enabled, "CANCELLED");
            return;
        }
        const auto dispatch_time = chrono::steady_clock::now();
        const double lateness_ms = abs(chrono::duration<double, milli>(
            dispatch_time - (steady_start + sample_offset)).count());
        abs_lateness_ms.push_back(lateness_ms);
        if (lateness_ms >= command_period_ms)
        {
            ++missed_cycle_count;
        }
        if (have_previous_dispatch)
        {
            const double actual_interval_ms = chrono::duration<double, milli>(
                dispatch_time - previous_dispatch).count();
            const double scheduled_interval_ms =
                (sample.time_from_start_s - previous_sample_time_s) * 1000.0;
            abs_jitter_ms.push_back(abs(actual_interval_ms - scheduled_interval_ms));
        }
        previous_dispatch = dispatch_time;
        previous_sample_time_s = sample.time_from_start_s;
        have_previous_dispatch = true;
        JointValue joint_pose;
        for (size_t joint = 0; joint < 6U; ++joint)
        {
            joint_pose.jVal[joint] = sample.positions_rad[joint];
        }
        const int64_t dispatch_wall_unix_ns = host_wall_clock_now_ns();
        {
            lock_guard<mutex> lock(phase5_trajectory_mutex);
            if (phase5_active_trajectory_id == trajectory_id)
            {
                if (sample_index == 0U)
                {
                    phase5_execution_status.first_dispatch_unix_ns =
                        dispatch_wall_unix_ns;
                }
                phase5_execution_status.last_dispatch_unix_ns =
                    dispatch_wall_unix_ns;
            }
        }
        const auto servo_call_start = chrono::steady_clock::now();
        const int servo_ret = robot.servo_j(
            &joint_pose, MoveMode::ABS, static_cast<int>(servo_step_num));
        const auto servo_call_end = chrono::steady_clock::now();
        const int64_t servo_return_wall_unix_ns = host_wall_clock_now_ns();
        {
            lock_guard<mutex> lock(phase5_trajectory_mutex);
            if (phase5_active_trajectory_id == trajectory_id)
            {
                if (sample_index == 0U)
                {
                    phase5_execution_status.first_servo_return_unix_ns =
                        servo_return_wall_unix_ns;
                }
                phase5_execution_status.last_servo_return_unix_ns =
                    servo_return_wall_unix_ns;
            }
        }
        const double servo_call_duration_ms = chrono::duration<double, milli>(
            servo_call_end - servo_call_start).count();
        servo_j_call_duration_ms.push_back(servo_call_duration_ms);
        if (servo_call_duration_ms >= command_period_ms)
        {
            ++servo_j_overrun_count;
        }
        if (servo_ret != 0)
        {
            robot.motion_abort();
            store_phase5_dispatch_timing(
                trajectory_id, abs_lateness_ms, abs_jitter_ms,
                servo_j_call_duration_ms, missed_cycle_count, servo_j_overrun_count);
            finish_phase5_trajectory(trajectory_id, servo_mode_enabled, "SERVO_STREAM_FAILED");
            return;
        }
        {
            lock_guard<mutex> lock(phase5_trajectory_mutex);
            if (phase5_active_trajectory_id == trajectory_id)
            {
                phase5_execution_status.sample_index = static_cast<int64_t>(sample_index);
                phase5_execution_status.commanded_sample_valid = true;
                phase5_execution_status.commanded_time_from_start_s =
                    sample.time_from_start_s;
                phase5_execution_status.commanded_joints_rad = sample.positions_rad;
                phase5_execution_status.elapsed_s = min(
                    sample.time_from_start_s, phase5_execution_status.duration_s);
                phase5_execution_status.progress_0_to_1 =
                    phase5_execution_status.duration_s > 0.0
                    ? min(max(
                        phase5_execution_status.elapsed_s /
                        phase5_execution_status.duration_s, 0.0), 1.0)
                    : 0.0;
            }
        }
    }
    store_phase5_dispatch_timing(
        trajectory_id, abs_lateness_ms, abs_jitter_ms,
        servo_j_call_duration_ms, missed_cycle_count, servo_j_overrun_count);
    this_thread::sleep_for(chrono::duration<double, milli>(command_period_ms));
    finish_phase5_trajectory(trajectory_id, servo_mode_enabled, "COMPLETED");
}

bool execute_joint_trajectory_callback(
    const shared_ptr<jaka_msgs::srv::ExecuteJointTrajectory::Request> request,
    shared_ptr<jaka_msgs::srv::ExecuteJointTrajectory::Response> response)
{
    response->accepted = false;
    response->ret = 0;
    response->trajectory_id = request->trajectory_id;
    if (request->trajectory_id.empty() || request->trajectory_id.size() > 128U ||
        all_of(
            request->trajectory_id.begin(), request->trajectory_id.end(),
            [](const unsigned char character) {
                return isprint(character) != 0 && isspace(character) == 0;
            }) == false)
    {
        response->message =
            "trajectory_id must contain 1..128 printable non-whitespace characters";
        return true;
    }

    auto validation = jaka_driver::phase5::validate_trajectory(
        request->time_from_start_s,
        request->joint_positions_rad_flat,
        request->start_time_unix_ns,
        host_wall_clock_now_ns());
    if (!validation.ok)
    {
        response->message = "rejected: " + validation.message;
        return true;
    }
    if (request->servo_step_num < 1U || request->servo_step_num > 4U)
    {
        response->message = "rejected: servo_step_num must be within [1, 4]";
        return true;
    }
    const double command_period_s =
        jaka_driver::phase5::servo_command_period_s(request->servo_step_num);
    // Preserve the planner's rigid-grasp knots without independently bending
    // each arm through a C2 quintic path between them. The stream remains
    // continuous at the configured servo cadence; controller-side foresight
    // filtering remains configured separately.
    auto generated = jaka_driver::phase5::resample_linear(
        validation.samples, request->servo_step_num);
    if (!generated.ok || generated.samples.size() < 2U)
    {
        response->message = "rejected: " + generated.message;
        return true;
    }
    Phase5ServoFilterConfig filter_config;
    filter_config.mode = request->servo_filter_mode;
    filter_config.legacy_max_buf = request->servo_filter_legacy_max_buf;
    filter_config.legacy_kp = request->servo_filter_legacy_kp;
    filter_config.lpf_cutoff_hz = request->servo_filter_lpf_cutoff_hz;
    filter_config.nlf_max_velocity_deg_s = request->servo_filter_nlf_max_velocity_deg_s;
    filter_config.nlf_max_acceleration_deg_s2 =
        request->servo_filter_nlf_max_acceleration_deg_s2;
    filter_config.nlf_max_jerk_deg_s3 = request->servo_filter_nlf_max_jerk_deg_s3;
    string filter_validation_message;
    if (!validate_phase5_filter(filter_config, filter_validation_message))
    {
        response->message = "rejected: " + filter_validation_message;
        return true;
    }
    const auto stream_guard = jaka_driver::phase5::validate_stream_command_velocity(
        generated.samples, command_period_s);
    if (!stream_guard.ok)
    {
        response->message = "rejected: " + stream_guard.message;
        return true;
    }
    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_trajectory_active)
        {
            response->message = "rejected: another Phase-5 trajectory is active";
            return true;
        }
        // Recheck the common start at reservation time to close validation races.
        if (request->start_time_unix_ns - host_wall_clock_now_ns() <
            jaka_driver::phase5::kMinimumStartLeadNs)
        {
            response->message = "rejected: common start lead expired during validation";
            return true;
        }
        phase5_trajectory_cancel_requested.store(false);
        phase5_trajectory_active = true;
        phase5_active_trajectory_id = request->trajectory_id;
        // Gate legacy request/response telemetry before the first Phase-5 SDK
        // preparation call and keep it gated through terminal cleanup.
        lock_guard<mutex> telemetry_gate(phase5_telemetry_gate_mutex);
        phase5_telemetry_suppressed_poll_count.store(0U);
        phase5_sdk_control_window_active.store(true);
    }

    // JAKA filter lifecycle requires filter selection outside servo mode.
    const int filter_ret = apply_phase5_filter(filter_config);
    if (filter_ret != 0)
    {
        const Phase5SdkCleanupResult cleanup =
            cleanup_phase5_sdk_control_window(false);
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == request->trajectory_id)
        {
            phase5_trajectory_active = false;
            phase5_active_trajectory_id.clear();
        }
        phase5_trajectory_cancel_requested.store(true);
        response->message = cleanup.baseline_restore_ret == 0
            ? "rejected: Phase-5 servo filter configuration failed"
            : "rejected: Phase-5 filter failed and baseline restore also failed";
        return true;
    }

    // STOP/cancel may arrive while filter configuration is in flight. Recheck
    // before servo-mode entry so an operator STOP can never be followed by a
    // fresh servo enable from this preparation attempt.
    bool cancelled_before_servo_enable = false;
    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        cancelled_before_servo_enable =
            phase5_trajectory_cancel_requested.load() ||
            phase5_active_trajectory_id != request->trajectory_id;
    }
    if (cancelled_before_servo_enable)
    {
        const Phase5SdkCleanupResult cleanup =
            cleanup_phase5_sdk_control_window(false);
        {
            lock_guard<mutex> lock(phase5_trajectory_mutex);
            if (phase5_active_trajectory_id == request->trajectory_id)
            {
                phase5_trajectory_active = false;
                phase5_active_trajectory_id.clear();
            }
        }
        response->message = cleanup.baseline_restore_ret == 0
            ? "rejected: STOP/cancel arrived before servo-mode entry"
            : "rejected: STOP/cancel arrived and baseline restore failed";
        return true;
    }

    // Do not claim acceptance until filter configuration and servo-mode entry
    // have both succeeded. Entering servo mode alone sends no joint motion.
    const int enable_ret = robot.servo_move_enable(TRUE);
    if (enable_ret != 0)
    {
        const Phase5SdkCleanupResult cleanup =
            cleanup_phase5_sdk_control_window(false);
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == request->trajectory_id)
        {
            phase5_trajectory_active = false;
            phase5_active_trajectory_id.clear();
        }
        phase5_trajectory_cancel_requested.store(true);
        response->message = cleanup.baseline_restore_ret == 0
            ? "rejected: servo mode enable failed before acceptance"
            : "rejected: servo enable failed and baseline restore also failed";
        return true;
    }
    bool cancelled_during_preparation = false;
    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        cancelled_during_preparation =
            phase5_trajectory_cancel_requested.load() ||
            phase5_active_trajectory_id != request->trajectory_id;
    }
    if (cancelled_during_preparation)
    {
        const Phase5SdkCleanupResult cleanup =
            cleanup_phase5_sdk_control_window(true);
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == request->trajectory_id)
        {
            phase5_trajectory_active = false;
            phase5_active_trajectory_id.clear();
        }
        response->message = cleanup.disable_ret == 0 && cleanup.baseline_restore_ret == 0
            ? "rejected: STOP/cancel arrived during servo preparation"
            : "rejected: STOP/cancel arrived and servo cleanup failed";
        return true;
    }

    {
        lock_guard<mutex> lock(phase5_trajectory_mutex);
        if (phase5_active_trajectory_id == request->trajectory_id)
        {
            phase5_execution_status = Phase5ExecutionStatus{};
            phase5_execution_status.trajectory_id = request->trajectory_id;
            phase5_execution_status.state = "ARMED";
            phase5_execution_status.active = true;
            phase5_execution_status.start_time_unix_ns = request->start_time_unix_ns;
            phase5_execution_status.duration_s = validation.duration_s;
            phase5_execution_status.sample_index = -1;
            phase5_execution_status.sample_count =
                static_cast<uint64_t>(generated.samples.size());
            phase5_execution_status.servo_step_num = request->servo_step_num;
            phase5_execution_status.command_period_ms = command_period_s * 1000.0;
            phase5_execution_status.telemetry_mode =
                "PHASE5_SDK_EXCLUSIVE_TELEMETRY_FROZEN";
            phase5_execution_status.stream_guard_limit_rad_s =
                stream_guard.limit_rad_s;
            phase5_execution_status.stream_guard_observed_max_rad_s =
                stream_guard.observed_max_rad_s;
            phase5_execution_status.max_abs_velocity_rad_s =
                generated.diagnostics.max_abs_velocity_rad_s;
            phase5_execution_status.rms_velocity_rad_s =
                generated.diagnostics.rms_velocity_rad_s;
            phase5_execution_status.max_abs_acceleration_rad_s2 =
                generated.diagnostics.max_abs_acceleration_rad_s2;
            phase5_execution_status.rms_acceleration_rad_s2 =
                generated.diagnostics.rms_acceleration_rad_s2;
            phase5_execution_status.max_abs_jerk_rad_s3 =
                generated.diagnostics.max_abs_jerk_rad_s3;
            phase5_execution_status.rms_jerk_rad_s3 =
                generated.diagnostics.rms_jerk_rad_s3;
            phase5_execution_status.servo_filter_mode =
                phase5_filter_label(filter_config.mode);
            phase5_execution_status.servo_filter_legacy_max_buf =
                filter_config.legacy_max_buf;
            phase5_execution_status.servo_filter_legacy_kp =
                filter_config.legacy_kp;
            phase5_execution_status.servo_filter_lpf_cutoff_hz =
                filter_config.lpf_cutoff_hz;
            phase5_execution_status.servo_filter_nlf_max_velocity_deg_s =
                filter_config.nlf_max_velocity_deg_s;
            phase5_execution_status.servo_filter_nlf_max_acceleration_deg_s2 =
                filter_config.nlf_max_acceleration_deg_s2;
            phase5_execution_status.servo_filter_nlf_max_jerk_deg_s3 =
                filter_config.nlf_max_jerk_deg_s3;
        }
    }

    try
    {
        thread(
            execute_phase5_joint_trajectory_worker,
            std::move(generated.samples),
            request->start_time_unix_ns,
            request->trajectory_id,
            request->servo_step_num).detach();
    }
    catch (const exception &error)
    {
        phase5_trajectory_cancel_requested.store(true);
        finish_phase5_trajectory(
            request->trajectory_id, true, "WORKER_LAUNCH_FAILED");
        response->message = string("rejected: worker launch failed: ") + error.what();
        return true;
    }
    response->accepted = true;
    response->ret = 1;
    response->message = string(
        "accepted: host-timed common absolute start; no hard real-time guarantee; ") +
        kPhase5ExecutionContractMarker;
    return true;
}

bool stop_move_callback([[maybe_unused]] const shared_ptr<std_srvs::srv::Empty::Request> request,
    [[maybe_unused]] shared_ptr<std_srvs::srv::Empty::Response> response)
{
    //Initialize jog related parameters
    jog_count = 0;
    jog_count_temp = 0;
    jog_index_last = -1;
    // Cancel any armed/running Phase-5 stream before preserving the legacy
    // motion_abort path below.
    phase5_trajectory_cancel_requested.store(true);
    int ret = robot.motion_abort();
    switch(ret)
    {
        case 0:
            RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "stop_move has been executed");
            break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("stop_move_callback"), "error occurred: %s", mapErr[ret].c_str());
            return false;;
    }
    return true;
}


bool set_toolFrame_callback(const shared_ptr<jaka_msgs::srv::SetTcpFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetTcpFrame::Response> response)
{
    CartesianPose tool_frame;
    int tool_frame_id = request->tool_num;
    tool_frame.tran.x = request->pose[0];
    tool_frame.tran.y = request->pose[1];
    tool_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(tool_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // tool_frame.rpy.rx = rpy.x();
    // tool_frame.rpy.ry = rpy.y();
    // tool_frame.rpy.rz = rpy.z();

    int ret = robot.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_toolFrame has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}


bool set_userFrame_callback(const shared_ptr<jaka_msgs::srv::SetUserFrame::Request> request,
    shared_ptr<jaka_msgs::srv::SetUserFrame::Response> response)
{
    CartesianPose user_frame;
    int user_frame_id = request->user_num; 
    user_frame.tran.x = request->pose[0];
    user_frame.tran.y = request->pose[1];
    user_frame.tran.z = request->pose[2];
    Eigen::Vector3d Angaxis = {request->pose[3],request->pose[4],request->pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(user_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // user_frame.rpy.rx = rpy.x();
    // user_frame.rpy.ry = rpy.y();
    // user_frame.rpy.rz = rpy.z();
    int ret = robot.set_user_frame_data(user_frame_id, &user_frame, "BaseCoord");
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_userFrame has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool set_payload_callback(const shared_ptr<jaka_msgs::srv::SetPayload::Request> request,
    shared_ptr<jaka_msgs::srv::SetPayload::Response> response)
{
    PayLoad payload;
    int tool_id = request->tool_num;

    payload.centroid.x = request->xc;
    payload.centroid.y = request->yc;
    payload.centroid.z = request->zc;
    payload.mass = request->mass;

    robot.set_tool_id(tool_id);
    int ret = robot.set_payload(&payload);

    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set_payload has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;
}

bool drag_mode_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
    shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int ret = robot.drag_mode_enable(request->data);
    switch(ret)
    {
        case 0:
            response->success = 1;
            response->message = "drag_mode has been executed";
            break;
        default:
            response->success = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool set_collisionLevel_callback(const shared_ptr<jaka_msgs::srv::SetCollision::Request> request,
    shared_ptr<jaka_msgs::srv::SetCollision::Response> response)
{
    int collision_level;
    if(request->is_enable == 0)
    {
        collision_level = 0;
    }
    else
    {
        if(request->value <= 25)
        {
            collision_level = 1;
        }
        else if(request->value <= 50)
        {
            collision_level = 2;
        }
        else if(request->value <= 75)
        {
            collision_level = 3;
        }
        else if(request->value <=100)
        {
            collision_level = 4;
        }
        else
        {
            collision_level = 5;
        }
    }
    int ret = robot.set_collision_level(collision_level);
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "Collision level" + to_string(collision_level) + " has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;

    }
    return true;
}


bool set_io_callback(const shared_ptr<jaka_msgs::srv::SetIO::Request> request,
    shared_ptr<jaka_msgs::srv::SetIO::Response> response)
{   
    IOType type;
    int ret;
    switch(request->type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    float value = request->value;
    string signal = request->signal;
    int index = request->index;
    if(signal == "digital")
    {      
        BOOL digital_value;
        if(value)
        {
            digital_value = TRUE;
        }
        else
        {
            digital_value = FALSE;
        }
        ret = robot.set_digital_output(type, index, digital_value);
    }
    else if(signal == "analog")
    {
        ret = robot.set_analog_output(type, index, value);
    }
    switch(ret)
    {
        case 0:
            response->ret = 1;
            response->message = "set IO has been executed";
            break;
        default:
            response->ret = 0;
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}



bool get_io_callback(const shared_ptr<jaka_msgs::srv::GetIO::Request> request,
    shared_ptr<jaka_msgs::srv::GetIO::Response> response)
{   
    IOType type;
    int ret;
    BOOL digital_result;
    float analog_result;
    switch(request->type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
        default:
            response->value = -999999;
            response->message = "Invalid IO type";
            return false;  // Add return if an invalid type is requested
    }
    string signal = request->signal;
    int index = request->index;
    int path = request->path;
    if(signal == "digital")
    {       
        if(path == 0)
        {
            ret = robot.get_digital_input(type, index, &digital_result);
        }
        else if(path == 1)
        {
            ret = robot.get_digital_output(type, index, &digital_result);
        }
        else
        {
            response->value = -999999;
            response->message = "Invalid path value";
            return false;  // Ensure return for invalid path
        }
        switch(ret)
        {
            case 0:
                response->value = float(digital_result);
                response->message = "get IO has been executed";
                break;
            default:
                response->value = -999999;
                response->message = "error occurred:" + mapErr[ret];
        }
        return true;
    }
    else if(signal == "analog")
    {
        if(path == 0)
        {
            ret = robot.get_analog_input(type, index, &analog_result);
        }
        else if(path == 1)
        {
            ret = robot.get_analog_output(type, index, &analog_result);
        }
        else
        {
            response->value = -999999;
            response->message = "Invalid path value";
            return false;  // Ensure return for invalid path
        }
        switch(ret)
        {
            case 0:
                response->value = analog_result;
                response->message = "get IO has been executed";
                break;
            default:
                response->value = -999999;
                response->message = "error occurred:" + mapErr[ret];

        }
    return true;
    }
    else 
    {
        // Handle case where signal is neither "digital" nor "analog"
        response->value = -999999;
        response->message = "Invalid signal type";
        return false;  // Return false if invalid signal
    }
    // This part is redundant but is here to prevent reaching the end without a return
    return true;
    
}

bool get_frame_state_callback(
    [[maybe_unused]] const shared_ptr<jaka_msgs::srv::GetFrameState::Request> request,
    shared_ptr<jaka_msgs::srv::GetFrameState::Response> response)
{
    int tool_id = -1;
    int user_frame_id = -1;
    CartesianPose tool_pose{};
    CartesianPose user_pose{};
    Quaternion installation_quaternion{};
    Rpy installation_rpy{};

    const int tool_id_ret = robot.get_tool_id(&tool_id);
    const int user_id_ret = robot.get_user_frame_id(&user_frame_id);
    const int installation_ret = robot.get_installation_angle(
        &installation_quaternion, &installation_rpy);
    if (tool_id_ret != 0 || user_id_ret != 0 || installation_ret != 0) {
        response->ret = 0;
        response->message = "read-only frame-state getter failed";
        return true;
    }

    const int tool_ret = robot.get_tool_data(tool_id, &tool_pose);
    const int user_ret = robot.get_user_frame_data(user_frame_id, &user_pose);
    if (tool_ret != 0 || user_ret != 0) {
        response->ret = 0;
        response->message = "read-only active frame-data getter failed";
        return true;
    }

    response->ret = 1;
    response->message = "READ ONLY frame state";
    response->tool_id = static_cast<int16_t>(tool_id);
    response->tool_pose = {
        tool_pose.tran.x, tool_pose.tran.y, tool_pose.tran.z,
        tool_pose.rpy.rx, tool_pose.rpy.ry, tool_pose.rpy.rz};
    response->user_frame_id = static_cast<int16_t>(user_frame_id);
    response->user_frame_pose = {
        user_pose.tran.x, user_pose.tran.y, user_pose.tran.z,
        user_pose.rpy.rx, user_pose.rpy.ry, user_pose.rpy.rz};
    response->installation_rpy = {
        installation_rpy.rx, installation_rpy.ry, installation_rpy.rz};
    response->installation_quaternion = {
        installation_quaternion.s, installation_quaternion.x,
        installation_quaternion.y, installation_quaternion.z};
    return true;
}

bool get_fk_callback(const shared_ptr<jaka_msgs::srv::GetFK::Request> request,
    shared_ptr<jaka_msgs::srv::GetFK::Response> response)
{
    JointValue joint_pose;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        joint_pose.jVal[i] = request->joint[i];
    }
    int ret = robot.kine_forward(&joint_pose, &cartesian_pose);
    switch(ret)
    {
        case 0:
            response->cartesian_pose.push_back(cartesian_pose.tran.x);
            response->cartesian_pose.push_back(cartesian_pose.tran.y);
            response->cartesian_pose.push_back(cartesian_pose.tran.z);
            response->cartesian_pose.push_back(cartesian_pose.rpy.rx);
            response->cartesian_pose.push_back(cartesian_pose.rpy.ry);
            response->cartesian_pose.push_back(cartesian_pose.rpy.rz);
            response->message = "get FK has been executed";
            break;
        default:
            float pose_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response->cartesian_pose.push_back(pose_init[i]);
            }
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}

bool get_ik_callback(const shared_ptr<jaka_msgs::srv::GetIK::Request> request,
    shared_ptr<jaka_msgs::srv::GetIK::Response> response)
{
    JointValue joint_pose;
    JointValue ref_joint;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        ref_joint.jVal[i] = request->ref_joint[i];

    }
    cartesian_pose.tran.x = request->cartesian_pose[0];
    cartesian_pose.tran.y = request->cartesian_pose[1];
    cartesian_pose.tran.z = request->cartesian_pose[2];
    cartesian_pose.rpy.rx = request->cartesian_pose[3];
    cartesian_pose.rpy.ry = request->cartesian_pose[4];
    cartesian_pose.rpy.rz = request->cartesian_pose[5];
    int ret = robot.kine_inverse(&ref_joint, &cartesian_pose, &joint_pose);
    switch(ret)
    {
        case 0:
            for(int i = 0; i < 6; i++)
            {
                response->joint.push_back(joint_pose.jVal[i]);
            }
            response->message = "get IK has been executed";
            break;
        default:
            float joint_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response->joint.push_back(joint_init[i]);
            }
            response->message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}


/*
bool clear_error_callback(const shared_ptr<jaka_msgs::srv::ClearError::Request> request,
    shared_ptr<jaka_msgs::srv::ClearError::Response> response)
{

    return true;
}
*/

void tool_position_callback(const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& tool_position_pub)
{
    // Check if publisher is valid
    if (!tool_position_pub)
    {
        RCLCPP_ERROR(rclcpp::get_logger("jaka_driver"), "Publisher is not initialized!");
        return;
    }

    geometry_msgs::msg::TwistStamped  tool_position;
    // RobotStatus robotstatus;
    CartesianPose tcp_position;
    RotMatrix rot;
    Rpy rpy;
    // robot.get_robot_status(&robotstatus);
    robot.get_tcp_position(&tcp_position);

    // tool_position.twist.linear.x = robotstatus.cartesiantran_position[0];
    // tool_position.twist.linear.y = robotstatus.cartesiantran_position[1];
    // tool_position.twist.linear.z = robotstatus.cartesiantran_position[2];
    // rpy.rx = robotstatus.cartesiantran_position[3];
    // rpy.ry = robotstatus.cartesiantran_position[4];
    // rpy.rz = robotstatus.cartesiantran_position[5];

    tool_position.twist.linear.x = tcp_position.tran.x;
    tool_position.twist.linear.y = tcp_position.tran.y;
    tool_position.twist.linear.z = tcp_position.tran.z;
    rpy.rx = tcp_position.rpy.rx;
    rpy.ry = tcp_position.rpy.ry;
    rpy.rz = tcp_position.rpy.rz;

    robot.rpy_to_rot_matrix(&rpy, &rot);
    // Eigen::Vector3d angaxis = Rot2Angaxis(rot);
    // tool_position.twist.angular.x = angaxis[0];
    // tool_position.twist.angular.y = angaxis[1];
    // tool_position.twist.angular.z = angaxis[2];
    tool_position.twist.angular.x = (rpy.rx )/PI*180;
    tool_position.twist.angular.y = (rpy.ry )/PI*180;
    tool_position.twist.angular.z = (rpy.rz )/PI*180;

    // Eigen::Vector3d eulerAngle(rpy.rx,rpy.ry,rpy.rz);
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    // Eigen::AngleAxisd rotation_vector;
    // rotation_vector=yawAngle*pitchAngle*rollAngle;
    // cout << "angle is: " << rotation_vector.angle() << endl;
    // cout << "axis is: " << rotation_vector.axis() << endl;
    // double angle = rotation_vector.angle();
    // Eigen::Vector3d axis = rotation_vector.axis();
    // Eigen::Vector3d v = angle * axis;
    // tool_position.twist.angular.x = v[0];
    // tool_position.twist.angular.y = v[1];
    // tool_position.twist.angular.z = v[2];
    
    tool_position.header.stamp = rclcpp::Clock().now();
    tool_position_pub->publish(tool_position);
}

void joint_position_callback(const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& joint_position_pub)
{
    sensor_msgs::msg::JointState joint_position;
    // RobotStatus robotstatus;
    JointValue joint_pos;
    // robot.get_robot_status(&robotstatus);
    robot.get_joint_position(&joint_pos);
    
    for (int i = 0; i < 6; i++)
    {
        // joint_position.position.push_back(robotstatus.joint_position[i]);
        // int j = i + 1;
        // joint_position.name.push_back("joint_" + to_string(j));

        joint_position.position.push_back(joint_pos.jVal[i]); 
        joint_position.name.push_back("joint_" + to_string(i + 1));
    }
    joint_position.header.stamp = rclcpp::Clock().now();
    joint_position_pub->publish(joint_position);
}

// Uses only the existing SDK session; the gate also serializes both controls.
void operator_state_control(jaka_driver::operator_state::Control control, bool desired,
                            std_srvs::srv::SetBool::Response& response)
{
    using namespace jaka_driver::operator_state;
    response.success = false;
    // Close the reservation race as well as the active SDK-window race.
    // Phase-5 reservation uses the same lock order: trajectory -> telemetry.
    if (phase5_sdk_control_window_active.load()) {
        response.message = "Phase5 owns the SDK control window";
        return;
    }
    unique_lock<mutex> trajectory_gate(phase5_trajectory_mutex);
    if (phase5_trajectory_active || phase5_sdk_control_window_active.load()) {
        response.message = "Phase5 trajectory is reserved or owns the SDK control window";
        return;
    }
    lock_guard<mutex> telemetry_gate(phase5_telemetry_gate_mutex);
    if (phase5_trajectory_active || phase5_sdk_control_window_active.load()) {
        response.message = "Phase5 trajectory is reserved or owns the SDK control window";
        return;
    }
    RobotStatus_simple status{};
    const int status_ret = robot.get_robot_status_simple(&status);
    State state{status_ret == 0 && (status.powered_on == 0 || status.powered_on == 1)
                && (status.enabled == 0 || status.enabled == 1),
                status.powered_on == 1, status.enabled == 1, false, false};
    auto decision = evaluate(control, desired, state, false);
    // Only off/disable transitions require extra idle reads. Never abort/stop.
    if (!desired && state.valid &&
        ((control == Control::Power && state.powered && !state.enabled) ||
         (control == Control::Enable && state.enabled))) {
        BOOL in_pos = false;
        BOOL drag = true;
        ProgramState program{};
        const int pos_ret = robot.is_in_pos(&in_pos);
        const int program_ret = robot.get_program_state(&program);
        const int drag_ret = robot.is_in_drag_mode(&drag);
        state.idle_known = pos_ret == 0 && program_ret == 0 && drag_ret == 0;
        state.idle = in_pos && program == PROGRAM_IDLE && !drag && status.errcode == 0;
        decision = evaluate(control, desired, state, false);
    }
    response.message = decision.message;
    if (!decision.allowed) return;
    if (!decision.change) {
        response.success = true;
        return;
    }
    const int ret = control == Control::Power
        ? (desired ? robot.power_on() : robot.power_off())
        : (desired ? robot.enable_robot() : robot.disable_robot());
    response.success = ret == 0;
    response.message = ret == 0
        ? "SDK accepted transition; verify fresh RobotMsg for actual state"
        : "SDK rejected transition, error code " + to_string(ret);
}

void robot_power_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
                          shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    operator_state_control(jaka_driver::operator_state::Control::Power, request->data, *response);
}

void robot_enable_callback(const shared_ptr<std_srvs::srv::SetBool::Request> request,
                           shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    operator_state_control(jaka_driver::operator_state::Control::Enable, request->data, *response);
}

void robot_states_callback(const rclcpp::Publisher<jaka_msgs::msg::RobotMsg>::SharedPtr& robot_states_pub)
{
    jaka_msgs::msg::RobotMsg robot_states;
    // RobotStatus robotstatus;
    RobotStatus_simple robotstatus_simple;
    ProgramState programstate;
    BOOL in_pos = true;
    BOOL in_col = false;
    BOOL drag_mode = false;
    BOOL emergency_stop = false;
    robot.is_in_pos(&in_pos);
    robot.is_in_collision(&in_col);
    robot.is_in_drag_mode(&drag_mode);
    robot.is_in_estop(&emergency_stop);
    // robot.get_robot_status(&robotstatus);
    robot.get_robot_status_simple(&robotstatus_simple);
    robot.get_program_state(&programstate);

    // if(robotstatus.emergency_stop)
    if(emergency_stop)
    {
        robot_states.motion_state = 2;
    }
    // else if(robotstatus.errcode)
    else if(robotstatus_simple.errcode)
    {
        robot_states.motion_state = 4;
    }
    // else if(in_pos && programstate == PROGRAM_IDLE && (!robotstatus.drag_status))
    else if(in_pos && programstate == PROGRAM_IDLE && (!drag_mode))
    {
        robot_states.motion_state = 0;
    }
    else if(programstate == PROGRAM_PAUSED)
    {
        robot_states.motion_state = 1;
    }
    // else if((!in_pos) || programstate == PROGRAM_RUNNING || robotstatus.drag_status)
    else if((!in_pos) || programstate == PROGRAM_RUNNING || drag_mode)
    {
        robot_states.motion_state = 3;
    }

    // if(robotstatus.powered_on)
    if(robotstatus_simple.powered_on)
    {
        robot_states.power_state = 1;
 
    }
    else
    {
        robot_states.power_state = 0;
    }

    // if(robotstatus.enabled)
    if(robotstatus_simple.enabled)
    {
        robot_states.servo_state = 1;
    }
    else
    {
        robot_states.servo_state = 0;
    }

    if(in_col)
    {
        robot_states.collision_state = 1;
    }
    else
    {
        robot_states.collision_state = 0;
    }
    robot_states_pub->publish(robot_states);
}

void stop_jog_callback()
{
     if (jog_count >= 1 && jog_count_temp == jog_count )
    {
        robot.jog_stop(-1);
        jog_count = 0;
        jog_count_temp = 0;
        jog_index_last = -1;
        RCLCPP_INFO(rclcpp::get_logger("stop_jog_callback"), "jog stop");
        
    }
    jog_count_temp = jog_count;
}

void get_conn_scoket_state(){
    JointValue temp_joints;
    RobotStatus robot_status{};
    int joint_monitor_counter = 0;

    while (rclcpp::ok())
    {
        // Holding this gate across the legacy chain closes the transition race:
        // once reservation sets the atomic window flag, no later normal SDK
        // request/response telemetry call can begin until cleanup clears it.
        unique_lock<mutex> telemetry_gate(phase5_telemetry_gate_mutex);
        if (phase5_sdk_control_window_active.load())
        {
            // The Phase-5 servo stream exclusively owns the global JAKA SDK
            // session. Keep the last genuinely observed ROS telemetry cached
            // and stale rather than issuing or fabricating a fresh sample.
            // STOP remains independent of this gate and calls motion_abort
            // directly from its ROS service callback.
            phase5_telemetry_suppressed_poll_count.fetch_add(1U);
            telemetry_gate.unlock();
            rclcpp::sleep_for(chrono::milliseconds(50));
            continue;
        }
        // int ret = robot.get_robot_status(&robot_status);
        int ret = robot.get_joint_position(&temp_joints);

		// if (ret)
        // {
        //     RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), "get_robot_status error!!!");
        // }
        // else if(!robot_status.is_socket_connect)
		// {
        //     RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), "connect error!!!");
        // }

        if (ret)
        {
            RCLCPP_ERROR(rclcpp::get_logger("get_conn_socket_state"), 
                         "Connection error or get_joint_position failed, error_code: %d, error: %s", ret, mapErr[ret].c_str());
        }

        if(ret==0)
        {
            // RCLCPP_INFO(rclcpp::get_logger("get_conn_scoket_state"), "ret=0");


            tool_position_callback(tool_position_pub);
            joint_position_callback(joint_position_pub);
            robot_states_callback(robot_state_pub);

            // Read-only diagnostic: print joint monitor data about once per second.
            joint_monitor_counter++;
            if (joint_monitor_counter >= 15)
            {
                joint_monitor_counter = 0;

                int status_ret = robot.get_robot_status(&robot_status);

                if (status_ret != 0)
                {
                    RCLCPP_WARN(
                        rclcpp::get_logger("joint_monitor"),
                        "get_robot_status failed, error_code=%d",
                        status_ret
                    );
                }
                else
                {
                    const auto &jm =
                        robot_status.robot_monitor_data.jointMonitorData;

                    RCLCPP_INFO(
                        rclcpp::get_logger("joint_monitor"),
                        "CURRENT_RAW [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                        jm[0].instCurrent, jm[1].instCurrent,
                        jm[2].instCurrent, jm[3].instCurrent,
                        jm[4].instCurrent, jm[5].instCurrent
                    );

                    RCLCPP_INFO(
                        rclcpp::get_logger("joint_monitor"),
                        "TORQUE_RAW  [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                        jm[0].instTorq, jm[1].instTorq,
                        jm[2].instTorq, jm[3].instTorq,
                        jm[4].instTorq, jm[5].instTorq
                    );

                    RCLCPP_INFO(
                        rclcpp::get_logger("joint_monitor"),
                        "VELOCITY_RAW [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                        jm[0].instVel, jm[1].instVel,
                        jm[2].instVel, jm[3].instVel,
                        jm[4].instVel, jm[5].instVel
                    );

                    std_msgs::msg::Float64MultiArray current_msg;
                    std_msgs::msg::Float64MultiArray torque_msg;
                    std_msgs::msg::Float64MultiArray velocity_msg;

                    current_msg.data.resize(6);
                    torque_msg.data.resize(6);
                    velocity_msg.data.resize(6);

                    for (size_t i = 0; i < 6; ++i)
                    {
                        current_msg.data[i] = jm[i].instCurrent;
                        torque_msg.data[i] = jm[i].instTorq;
                        velocity_msg.data[i] = jm[i].instVel;
                    }

                    std_msgs::msg::Float64MultiArray monitor_msg;
                    monitor_msg.data.resize(18);

                    for (size_t i = 0; i < 6; ++i)
                    {
                        // Index 0-5: current
                        monitor_msg.data[i] = jm[i].instCurrent;

                        // Index 6-11: torque
                        monitor_msg.data[6 + i] = jm[i].instTorq;

                        // Index 12-17: velocity
                        monitor_msg.data[12 + i] = jm[i].instVel;
                    }

                    if (joint_current_pub &&
                        joint_torque_pub &&
                        joint_monitor_velocity_pub &&
                        joint_monitor_raw_pub)
                    {
                        joint_current_pub->publish(current_msg);
                        joint_torque_pub->publish(torque_msg);
                        joint_monitor_velocity_pub->publish(velocity_msg);
                        joint_monitor_raw_pub->publish(monitor_msg);
                    }
                }
            }

        }
        telemetry_gate.unlock();
        rclcpp::sleep_for(chrono::milliseconds(50)); 
    }    
}

int main(int argc, char *argv[])
{

    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("jaka_driver");
    rclcpp::Rate rate(125); 
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    string robot_ip = node->declare_parameter("ip", default_ip);

    // Safety-first parameters for real robot commissioning.
    // read_only=true prevents automatic power_on/enable_robot at startup.
    bool read_only = node->declare_parameter("read_only", true);
    bool auto_enable = node->declare_parameter("auto_enable", false);
    // Prefix for running multiple JAKA drivers in the same ROS graph.
    // Example: /left_jaka_driver or /right_jaka_driver
    string service_prefix = node->declare_parameter("service_prefix", string("/jaka_driver"));
    if (!service_prefix.empty() && service_prefix[0] != '/') {
        service_prefix = "/" + service_prefix;
    }
    while (service_prefix.size() > 1 && service_prefix.back() == '/') {
        service_prefix.pop_back();
    }

    RCLCPP_WARN(
        rclcpp::get_logger("jaka_driver"),
        "Starting JAKA driver with ip=%s, read_only=%s, auto_enable=%s",
        robot_ip.c_str(),
        read_only ? "true" : "false",
        auto_enable ? "true" : "false"
    );

    robot.login_in(robot_ip.c_str(), false);
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);

    if (!read_only && auto_enable) {
        RCLCPP_WARN(rclcpp::get_logger("jaka_driver"), "AUTO ENABLE IS ON: robot.power_on() will be called.");
        robot.power_on();
        sleep(8);

        RCLCPP_WARN(rclcpp::get_logger("jaka_driver"), "AUTO ENABLE IS ON: robot.enable_robot() will be called.");
        robot.enable_robot();
        sleep(4);

        //Joint-space first-order low-pass filtering in robot servo mode
        //robot.servo_move_use_joint_LPF(2);
        robot.servo_speed_foresight(15,0.03);
    } else {
        RCLCPP_WARN(
            rclcpp::get_logger("jaka_driver"),
            "SAFE READ-ONLY STARTUP: skipped robot.power_on(), robot.enable_robot(), and servo_speed_foresight()."
        );
    }

    auto robot_power_service = node->create_service<std_srvs::srv::SetBool>(service_prefix + "/robot_power", &robot_power_callback);
    auto robot_enable_service = node->create_service<std_srvs::srv::SetBool>(service_prefix + "/robot_enable", &robot_enable_callback);

    //1.1 Linear motion (in customized user coordinate system)
    auto linear_move_service = node->create_service<jaka_msgs::srv::Move>((service_prefix + "/linear_move"), &linear_move_callback);
    //1.2 Joint motion
    auto joint_move_service = node->create_service<jaka_msgs::srv::Move>((service_prefix + "/joint_move"), &joint_move_callback);
    //1.3 Jog motion
    auto jog_service = node->create_service<jaka_msgs::srv::Move>((service_prefix + "/jog"), &jog_callback);
    //1.4 Servo Position Control Mode Enable
    auto servo_move_enable_service = node->create_service<jaka_msgs::srv::ServoMoveEnable>((service_prefix + "/servo_move_enable"), &servo_move_enable_callback);
    //1.5 Servo-mode motion in Cartesian space
    auto servo_p_service = node->create_service<jaka_msgs::srv::ServoMove>((service_prefix + "/servo_p"), &servo_p_callback);
    //1.6 Joint space servo mode motion
    auto servo_j_service = node->create_service<jaka_msgs::srv::ServoMove>((service_prefix + "/servo_j"), &servo_j_callback);
    //1.6P5 Whole absolute joint trajectory with a host-timed common start.
    auto execute_joint_trajectory_service = node->create_service<jaka_msgs::srv::ExecuteJointTrajectory>((service_prefix + "/execute_joint_trajectory"), &execute_joint_trajectory_callback);
    //1.6P5R Read-only in-memory trajectory status; no SDK access in callback.
    auto get_execution_status_service = node->create_service<jaka_msgs::srv::GetExecutionStatus>((service_prefix + "/get_execution_status"), &get_execution_status_callback);
    //1.7 stop motion
    auto stop_move_service = node->create_service<std_srvs::srv::Empty>((service_prefix + "/stop_move"), &stop_move_callback);
    //2.1 Setting tcp parameters
    auto set_toolframe_service = node->create_service<jaka_msgs::srv::SetTcpFrame>((service_prefix + "/set_toolframe"), &set_toolFrame_callback);
    //2.2 Setting user coordinate system parameters
    auto set_userframe_service = node->create_service<jaka_msgs::srv::SetUserFrame>((service_prefix + "/set_userframe"), &set_userFrame_callback);
    //2.3 Set the center of gravity parameters of the robot arm load
    auto set_payload_service = node->create_service<jaka_msgs::srv::SetPayload>((service_prefix + "/set_payload"), &set_payload_callback);
    //2.4 Set free drive mode
    auto drag_move_service = node->create_service<std_srvs::srv::SetBool>((service_prefix + "/drag_move"), &drag_mode_callback);
    //2.5 Set collision sensitivity
    auto set_collisionlevel_service = node->create_service<jaka_msgs::srv::SetCollision>((service_prefix + "/set_collisionlevel"), &set_collisionLevel_callback);
    //2.6 Set IO
    auto set_io_service = node->create_service<jaka_msgs::srv::SetIO>((service_prefix + "/set_io"),&set_io_callback);
    //2.7 Get IO
    auto get_io_service = node->create_service<jaka_msgs::srv::GetIO>((service_prefix + "/get_io"),&get_io_callback);
    //2.8 Find the positive solution
    auto get_fk_service = node->create_service<jaka_msgs::srv::GetFK>((service_prefix + "/get_fk"), &get_fk_callback);
    //2.9 Find the inverse solution
    auto get_ik_service = node->create_service<jaka_msgs::srv::GetIK>((service_prefix + "/get_ik"), &get_ik_callback);
    //2.10 Read active Tool/User/Mounting state only; no robot state is modified.
    auto get_frame_state_service = node->create_service<jaka_msgs::srv::GetFrameState>((service_prefix + "/get_frame_state"), &get_frame_state_callback);

    // //3.1 End position pose status information reporting
    tool_position_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>((service_prefix + "/tool_position"), 10);
    // //3.2 Joint status information reporting
    joint_position_pub = node->create_publisher<sensor_msgs::msg::JointState>((service_prefix + "/joint_position"), 10);
    // //3.3 Report robot event status information
    robot_state_pub = node->create_publisher<jaka_msgs::msg::RobotMsg>((service_prefix + "/robot_states"), 10);

    // Read-only joint motor monitoring from JAKA SDK RobotStatus.
    joint_current_pub =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(
            (service_prefix + "/joint_current_raw"), 10);

    joint_torque_pub =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(
            (service_prefix + "/joint_torque_raw"), 10);

    joint_monitor_velocity_pub =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(
            (service_prefix + "/joint_monitor_velocity_raw"), 10);

    // Combined synchronized monitor:
    // [current J1-J6, torque J1-J6, velocity J1-J6]
    joint_monitor_raw_pub =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(
            (service_prefix + "/joint_monitor_raw"), 10);
    
    // Automatically stop robot jog and motion
    auto stop_jog = node->create_wall_timer(chrono::seconds(3), stop_jog_callback);

    // Monitor network connection status
    thread conn_state_thread(get_conn_scoket_state);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start");

    rclcpp::spin(node);
     // Ensure thread is joined before shutting down the node
    if (conn_state_thread.joinable()) {
        conn_state_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
