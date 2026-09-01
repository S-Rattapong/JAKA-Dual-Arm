#include "rclcpp/rclcpp.hpp"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jktypes.h"

#include <iostream>
#include <string>

namespace
{

void print_pose(const CartesianPose &pose)
{
  std::cout << "  translation [mm]: x=" << pose.tran.x
            << " y=" << pose.tran.y
            << " z=" << pose.tran.z << '\n';
  std::cout << "  RPY [rad]: rx=" << pose.rpy.rx
            << " ry=" << pose.rpy.ry
            << " rz=" << pose.rpy.rz << '\n';
}

}  // namespace

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("frame_config_readonly");
  const std::string robot_ip = node->declare_parameter<std::string>("ip", "");

  std::cout << "JAKA CONTROLLER FRAME CONFIGURATION — READ ONLY\n";
  std::cout << "ROBOT IP: " << (robot_ip.empty() ? "MISSING" : robot_ip) << '\n';

  if (robot_ip.empty()) {
    std::cerr << "FAILED: required ROS parameter ip is empty\n";
    rclcpp::shutdown();
    return 2;
  }

  JAKAZuRobot robot;
  const errno_t login_ret = robot.login_in(robot_ip.c_str(), false);
  std::cout << "login_in ret=" << login_ret << '\n';
  if (login_ret != 0) {
    std::cerr << "FAILED ret=" << login_ret << " login_in\n";
    rclcpp::shutdown();
    return 1;
  }

  bool getter_failed = false;

  std::cout << "\nCURRENT USER FRAME\n";
  int user_frame_id = -1;
  const errno_t user_id_ret = robot.get_user_frame_id(&user_frame_id);
  std::cout << "get_user_frame_id ret=" << user_id_ret << '\n';
  if (user_id_ret == 0) {
    std::cout << "  active user frame ID: " << user_frame_id << '\n';
    CartesianPose user_frame{};
    const errno_t user_data_ret =
      robot.get_user_frame_data(user_frame_id, &user_frame);
    std::cout << "get_user_frame_data ret=" << user_data_ret << '\n';
    if (user_data_ret == 0) {
      print_pose(user_frame);
    } else {
      getter_failed = true;
      std::cout << "FAILED ret=" << user_data_ret
                << " get_user_frame_data\n";
    }
  } else {
    getter_failed = true;
    std::cout << "FAILED ret=" << user_id_ret << " get_user_frame_id\n";
    std::cout << "get_user_frame_data SKIPPED: active ID unavailable\n";
  }

  std::cout << "\nCURRENT TOOL FRAME\n";
  int tool_id = -1;
  const errno_t tool_id_ret = robot.get_tool_id(&tool_id);
  std::cout << "get_tool_id ret=" << tool_id_ret << '\n';
  if (tool_id_ret == 0) {
    std::cout << "  active tool ID: " << tool_id << '\n';
    CartesianPose tool_frame{};
    const errno_t tool_data_ret = robot.get_tool_data(tool_id, &tool_frame);
    std::cout << "get_tool_data ret=" << tool_data_ret << '\n';
    if (tool_data_ret == 0) {
      print_pose(tool_frame);
    } else {
      getter_failed = true;
      std::cout << "FAILED ret=" << tool_data_ret << " get_tool_data\n";
    }
  } else {
    getter_failed = true;
    std::cout << "FAILED ret=" << tool_id_ret << " get_tool_id\n";
    std::cout << "get_tool_data SKIPPED: active ID unavailable\n";
  }

  std::cout << "\nINSTALLATION / MOUNTING\n";
  Quaternion installation_quaternion{};
  Rpy installation_rpy{};
  const errno_t installation_ret = robot.get_installation_angle(
    &installation_quaternion, &installation_rpy);
  std::cout << "get_installation_angle ret=" << installation_ret << '\n';
  if (installation_ret == 0) {
    std::cout << "  quaternion [unitless]: s=" << installation_quaternion.s
              << " x=" << installation_quaternion.x
              << " y=" << installation_quaternion.y
              << " z=" << installation_quaternion.z << '\n';
    std::cout << "  application RPY [rad]: rx=" << installation_rpy.rx
              << " ry=" << installation_rpy.ry
              << " rz=" << installation_rpy.rz << '\n';
  } else {
    getter_failed = true;
    std::cout << "FAILED ret=" << installation_ret
              << " get_installation_angle\n";
  }

  const errno_t logout_ret = robot.login_out();
  std::cout << "\nlogin_out ret=" << logout_ret << '\n';
  if (logout_ret != 0) {
    std::cout << "FAILED ret=" << logout_ret << " login_out\n";
  }

  rclcpp::shutdown();
  return !getter_failed && logout_ret == 0 ? 0 : 1;
}
