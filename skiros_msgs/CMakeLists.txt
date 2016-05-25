cmake_minimum_required(VERSION 2.8.3)

project(skiros_msgs)

find_package(catkin REQUIRED COMPONENTS 
genmsg 
message_generation 
std_msgs 
actionlib_msgs 
trajectory_msgs 
sensor_msgs 
control_msgs
)

add_message_files(
  FILES
  Finger.msg
  FingerFB.msg
  DeviceData.msg
  ParamMapSerialized.msg
  WmElement.msg
  WmMonitor.msg
  WmTypeLikelihood.msg
  WmObjLikelihood.msg
  WmRelation.msg
  WoNode.msg
  WoStatement.msg
  RmMonitor.msg
  PrimitiveEvent.msg
  PrimSchedulerStartStop.msg
  PrimSchedulerSet.msg
  ModuleStatus.msg
  ModuleDescription.msg
  TmMonitor.msg
  TmTaskExe.msg
  )

add_service_files(
  FILES
  MotionPlanningService.srv
  ModulesListQuery.srv
  SkillExe.srv
  WmSetRelation.srv
  WmQuery.srv
  WoQuery.srv
  WoModify.srv
  WmElementGet.srv
  WmElementModify.srv
  WmIdentify.srv
  WmClassify.srv
  RmDeviceHandle.srv
  ModuleCommand.srv
  TmModifyTask.srv
  TmGoal.srv
  TmQueryTask.srv
  WmSceneLoadAndSave.srv
  SetBool.srv
  )

add_action_files(
  FILES
  Gripper.action
  )

generate_messages(
  DEPENDENCIES trajectory_msgs std_msgs actionlib_msgs sensor_msgs control_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime genmsg std_msgs trajectory_msgs actionlib_msgs sensor_msgs control_msgs
)
