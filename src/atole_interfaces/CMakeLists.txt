cmake_minimum_required(VERSION 3.8)
project(atole_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/EstimatePods.action"
  "action/GripperCommand.action"
  "action/Harvest.action"
  "action/MoveJoints.action"
  "action/MovePose.action"
  "action/RunCalibSequence.action"
  "action/SetCameraMode.action"
  "msg/ArmState.msg"
  "msg/CalibrationResult.msg"
  "msg/CameraSlot.msg"
  "msg/CameraStatus.msg"
  "msg/ConfigSnapshot.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/GripperState.msg"
  "msg/HealthItem.msg"
  "msg/MissionStatus.msg"
  "msg/Pod.msg"
  "msg/PodArray.msg"
  "msg/SystemHealth.msg"
  "srv/AcquireControl.srv"
  "srv/CaptureCalibPose.srv"
  "srv/ComputeCalibration.srv"
  "srv/ComputeFk.srv"
  "srv/ComputeIk.srv"
  "srv/ConfigGet.srv"
  "srv/ConfigSet.srv"
  "srv/DetectBoard.srv"
  "srv/DetectOnce.srv"
  "srv/FuseViews.srv"
  "srv/JogJoint.srv"
  "srv/ListDatasets.srv"
  "srv/LoadDataset.srv"
  "srv/SaveCalibration.srv"
  "srv/SavePose.srv"
  "srv/ScanPort.srv"
  "srv/SelectEth.srv"
  "srv/SelectPod.srv"
  "srv/SetDisplay.srv"
  "srv/SetFreedrive.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
