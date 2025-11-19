cmake_minimum_required(VERSION 3.8)
project(sentry_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/HurtArmor.msg
  msg/Hp.msg
  msg/MatchStage.msg
  msg/RemainTime.msg
  msg/Remain17mm.msg
  msg/IsBlue.msg
  msg/RobotId.msg
  msg/Vw.msg
  msg/ScanMode.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
