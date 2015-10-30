#include <ros/ros.h>
#include "vrep_ros_common/vrep.h"
#include "v_repConst.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vrep_node");

  ros::NodeHandle nh("~");

  VRep vrep;
  // std::vector<std::string> objectNames;
  // vrep.getObjectNames(objectNames);
  // for (auto& objName : objectNames) {
  //   ROS_INFO("Obj: %s", objName.c_str());
  // }
  vrep.startSimulation();

  std::vector<std::string> objectNames;
  nh.getParam("objects", objectNames);

  for (auto& objName : objectNames) {
    uint32_t handle = vrep.getObjectHandle(objName);
    ROS_INFO("%d: '%s'", handle, objName.c_str());
    vrep.enablePublisher("topic", 10, simros_strmcmd_get_transform, handle, -1, "");
  }
}
