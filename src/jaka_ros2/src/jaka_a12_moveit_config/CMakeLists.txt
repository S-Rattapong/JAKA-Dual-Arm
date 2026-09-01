cmake_minimum_required(VERSION 3.22)
project(jaka_a12_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../dual_arm_app/config/world_frame_calibration.json
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../../dual_arm_app/backend/world_frame_calibration.py
  DESTINATION share/${PROJECT_NAME}/launch
)
