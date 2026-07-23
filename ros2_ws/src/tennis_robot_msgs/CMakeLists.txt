cmake_minimum_required(VERSION 3.8)
project(tennis_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallDetection.msg"
  "msg/BallDetectionArray.msg"
  "msg/CollectorCmd.msg"
  "msg/RobotCommand.msg"
  "msg/IrReadings.msg"
  "msg/CollectionExecutionProfile.msg"
  "msg/CollectionControllerTuning.msg"
  "msg/CollectionPlannedCrossing.msg"
  "msg/CollectionExecutionSegment.msg"
  "msg/CollectionExecutionContext.msg"
  "msg/CollectionProfileComplianceVerdict.msg"
  "msg/CollectionControllerState.msg"
  "srv/LoadCollectionExecutionContext.srv"
  "srv/ResetCollectionExecutionContext.srv"
  "srv/SetCollectionSafetyHold.srv"
  "srv/FinalizeCollectionExecutionContext.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_package()
