cmake_minimum_required(VERSION 3.8)
project(tennis_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallObservation.msg"
  "msg/CollectorCmd.msg"
  "msg/RobotCommand.msg"
  "msg/IrReadings.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
