cmake_minimum_required(VERSION 3.8)
project(interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(MSG_SRC 
  "msg/Cart.msg"
  "msg/RobotState.msg"
  "msg/AudioDataPackage.msg"
  "msg/ArmState.msg"
  "msg/BatteryInfo.msg"
  "msg/CameraPTZ.msg"
  "msg/CameraPTZMove.msg"
  "msg/QuadState.msg"
  "msg/RunState.msg"
  "msg/WalkMode.msg"
  "msg/Pose.msg"
  "msg/ArmInfo.msg"
  "msg/IoInState.msg"
  "msg/SmartFollow.msg"
  "msg/ClosestPoint.msg"
  "msg/ClosestPoints.msg"
  "msg/GamepadCmd.msg"
)

set(SRV_SRC
  "srv/DeviceInfo.srv"
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/DogMove.srv"
  "srv/Dance.srv"
  "srv/EnableSpeechRecog.srv"
  "srv/ArmCmd.srv"
  "srv/GetToolName.srv"
  "srv/SetToolName.srv"
  "srv/GetToolValue.srv"
  "srv/SetToolValue.srv"
  "srv/GetToolNameList.srv"
  "srv/SetToHome.srv"
  "srv/SetToTransport.srv"
  "srv/SetIoOutput.srv"
  "srv/Task.srv"
  "srv/SetGripper.srv"
  "srv/GoToPose.srv"
  "srv/GoToPoseFinish.srv"
  "srv/GoToPoseCancel.srv"
  "srv/ExecTree.srv"
  "srv/MyInfo.srv"
)

set(ACTION_SRC

)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cart.msg"
  "msg/RobotState.msg"
  "msg/AudioDataPackage.msg"
  "msg/ArmState.msg"
  "msg/BatteryInfo.msg"
  "msg/CameraPTZ.msg"
  "msg/CameraPTZMove.msg"
  "msg/QuadState.msg"
  "msg/RunState.msg"
  "msg/WalkMode.msg"
  "msg/Pose.msg"
  "msg/ArmInfo.msg"
  "msg/IoInState.msg"
  "msg/SmartFollow.msg"
  "msg/ClosestPoint.msg"
  "msg/ClosestPoints.msg"
  "msg/GamepadCmd.msg"
  "srv/DeviceInfo.srv"
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/DogMove.srv"
  "srv/Dance.srv"
  "srv/EnableSpeechRecog.srv"
  "srv/ArmCmd.srv"
  "srv/GetToolName.srv"
  "srv/SetToolName.srv"
  "srv/GetToolValue.srv"
  "srv/SetToolValue.srv"
  "srv/GetToolNameList.srv"
  "srv/SetToHome.srv"
  "srv/SetToTransport.srv"
  "srv/SetIoOutput.srv"
  "srv/Task.srv"
  "srv/SetGripper.srv"
  "srv/GoToPose.srv"
  "srv/GoToPoseFinish.srv"
  "srv/GoToPoseCancel.srv"
  "srv/ExecTree.srv"
  "srv/MyInfo.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)
 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
