#pragma once
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

class VRep
{
public:
  VRep();

  void startSimulation();

  uint32_t getObjectHandle(
    const std::string& name);

  bool getObjectPose(
    uint32_t handle,
    geometry_msgs::PoseStamped& result);

  std::string enablePublisher(
    const std::string& topicName,
    uint32_t queueSize,
    uint32_t streamCmd,
    uint32_t auxInt1,
    uint32_t auxInt2,
    const std::string& auxString);

  uint32_t enableSubscriber(
    const std::string& topicName,
    uint32_t queueSize,
    uint32_t streamCmd,
    uint32_t auxInt1,
    uint32_t auxInt2,
    const std::string& auxString);

  void getObjectNames(
    std::vector<std::string>& result);

private:
  ros::NodeHandle handle_;

};
