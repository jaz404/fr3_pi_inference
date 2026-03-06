cmake_minimum_required(VERSION 3.8)
project(franky_msgs)

# 1. Find the necessary generators and base types
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Actions ALWAYS need this package to provide the goal/result/feedback structure
find_package(action_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Organize your files into variables for cleanliness
set(msg_files
  "msg/CartesianMove.msg"
  "msg/CartesianVelocity.msg"
  "msg/CorrectionInfo.msg"
  "msg/JointMove.msg"
  "msg/JointVelocity.msg"
  "msg/GripperState.msg"
  "msg/GripperMove.msg"
  "msg/GripperGrasp.msg"
)

set(srv_files
  "srv/SetDynamicsFactor.srv"
  "srv/BlockingCartesianMove.srv"
  "srv/BlockingJointMove.srv"
  "srv/BlockingGripperMove.srv"
  "srv/BlockingGripperGrasp.srv"
)

# set(action_files
#   "action/MoveToPose.action"
# )

# 3. Generate everything in one go
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
#   ${action_files}
  DEPENDENCIES 
    action_msgs 
    geometry_msgs 
    std_msgs
)

ament_package()