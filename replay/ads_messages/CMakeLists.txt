cmake_minimum_required(VERSION 3.8)
project(ads_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Order does not matter here, but the FIELD order inside each .msg does: CDR is
# positional, so a reordered field decodes to the wrong value with no error.
# The files were generated from the bag's own schema text, not transcribed.
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CANImu.msg
  msg/CANMsg.msg
  msg/CANSignals.msg
  msg/ExtraCameraInfo.msg
  msg/VehicleConfig.msg
  DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
