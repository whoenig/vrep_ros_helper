#pragma once
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

class VRepImpl;

class VRep
{
public:
  VRep();

  void startSimulation();

  void stopSimulation();

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

  uint32_t setJointTargetVelocity(
    uint32_t handle,
    double targetVelocity);

  void synchronous(
    bool enable);

  void synchronousTrigger();

  double getSimulationTime();

private:
  ros::NodeHandle handle_;
  VRepImpl* impl_;

};
