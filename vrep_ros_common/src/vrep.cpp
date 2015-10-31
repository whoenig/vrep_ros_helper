#include "vrep.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosGetObjectGroupData.h"
#include "vrep_common/simRosSetJointTargetVelocity.h"
#include "vrep_common/simRosSynchronous.h"
#include "vrep_common/simRosSynchronousTrigger.h"
#include "vrep_common/simRosGetInfo.h"

#include "v_repConst.h"

class VRepImpl
{
public:
  ros::ServiceClient startSimulation;
  ros::ServiceClient stopSimulation;
  ros::ServiceClient getObjectHandle;
  ros::ServiceClient getObjectPose;
  ros::ServiceClient enablePublisher;
  ros::ServiceClient enableSubscriber;
  ros::ServiceClient getObjectGroupData;
  ros::ServiceClient setJointTargetVelocity;
  ros::ServiceClient synchronous;
  ros::ServiceClient synchronousTrigger;
  ros::ServiceClient getInfo;
};

VRep::VRep()
  : handle_()
  , impl_(0)
{
  impl_ = new VRepImpl;
  impl_->startSimulation        = handle_.serviceClient<vrep_common::simRosStartSimulation       >("/vrep/simRosStartSimulation");
  impl_->stopSimulation         = handle_.serviceClient<vrep_common::simRosStopSimulation        >("/vrep/simRosStopSimulation");
  impl_->getObjectHandle        = handle_.serviceClient<vrep_common::simRosGetObjectHandle       >("/vrep/simRosGetObjectHandle");
  impl_->getObjectPose          = handle_.serviceClient<vrep_common::simRosGetObjectPose         >("/vrep/simRosGetObjectPose");
  impl_->enablePublisher        = handle_.serviceClient<vrep_common::simRosEnablePublisher       >("/vrep/simRosEnablePublisher");
  impl_->enableSubscriber       = handle_.serviceClient<vrep_common::simRosEnableSubscriber      >("/vrep/simRosEnableSubscriber");
  impl_->getObjectGroupData     = handle_.serviceClient<vrep_common::simRosGetObjectGroupData    >("/vrep/simRosGetObjectGroupData");
  impl_->setJointTargetVelocity = handle_.serviceClient<vrep_common::simRosSetJointTargetVelocity>("/vrep/simRosSetJointTargetVelocity");
  impl_->synchronous            = handle_.serviceClient<vrep_common::simRosSynchronous           >("/vrep/simRosSynchronous");
  impl_->synchronousTrigger     = handle_.serviceClient<vrep_common::simRosSynchronousTrigger    >("/vrep/simRosSynchronousTrigger");
  impl_->getInfo                = handle_.serviceClient<vrep_common::simRosGetInfo               >("/vrep/simRosGetInfo");
}

void VRep::startSimulation()
{
  vrep_common::simRosStartSimulation srv;
  impl_->startSimulation.call(srv);
}

void VRep::stopSimulation()
{
  vrep_common::simRosStopSimulation srv;
  impl_->stopSimulation.call(srv);
}

uint32_t VRep::getObjectHandle(
  const std::string& name)
{
  vrep_common::simRosGetObjectHandle srv;
  srv.request.objectName = name;
  impl_->getObjectHandle.call(srv);
  return srv.response.handle;
}

bool VRep::getObjectPose(
  uint32_t handle,
  geometry_msgs::PoseStamped& result)
{
  vrep_common::simRosGetObjectPose srv;
  srv.request.handle = handle;
  srv.request.relativeToObjectHandle = -1;//absolute pose
  impl_->getObjectPose.call(srv);
  result = srv.response.pose;
  return srv.response.result != -1;
}

std::string VRep::enablePublisher(
  const std::string& topicName,
  uint32_t queueSize,
  uint32_t streamCmd,
  uint32_t auxInt1,
  uint32_t auxInt2,
  const std::string& auxString)
{
  vrep_common::simRosEnablePublisher srv;
  srv.request.topicName = topicName;
  srv.request.queueSize = queueSize;
  srv.request.streamCmd = streamCmd;
  srv.request.auxInt1 = auxInt1;
  srv.request.auxInt2 = auxInt2;
  srv.request.auxString = auxString;
  impl_->enablePublisher.call(srv);
  return srv.response.effectiveTopicName;
}

uint32_t VRep::enableSubscriber(
  const std::string& topicName,
  uint32_t queueSize,
  uint32_t streamCmd,
  uint32_t auxInt1,
  uint32_t auxInt2,
  const std::string& auxString)
{
  vrep_common::simRosEnableSubscriber srv;
  srv.request.topicName = topicName;
  srv.request.queueSize = queueSize;
  srv.request.streamCmd = streamCmd;
  srv.request.auxInt1 = auxInt1;
  srv.request.auxInt2 = auxInt2;
  srv.request.auxString = auxString;
  impl_->enableSubscriber.call(srv);
  return srv.response.subscriberID;
}

void VRep::getObjectNames(
  std::vector<std::string>& result)
{
  vrep_common::simRosGetObjectGroupData srv;
  srv.request.objectType = sim_appobj_object_type;
  srv.request.dataType = 0; // retrieves the object names
  impl_->getObjectGroupData.call(srv);
  result.clear();
  result.insert(result.begin(), srv.response.strings.begin(), srv.response.strings.end());
}

uint32_t VRep::setJointTargetVelocity(
  uint32_t handle,
  double targetVelocity)
{
  vrep_common::simRosSetJointTargetVelocity srv;
  srv.request.handle = handle;
  srv.request.targetVelocity = targetVelocity;
  impl_->setJointTargetVelocity.call(srv);
  return srv.response.result;
}

void VRep::synchronous(
  bool enable)
{
  vrep_common::simRosSynchronous srv;
  srv.request.enable = enable;
  impl_->synchronous.call(srv);
  if (srv.response.result == -1) {
    throw std::runtime_error("VRep::synchronous failed");
  }
}

void VRep::synchronousTrigger()
{
  vrep_common::simRosSynchronousTrigger srv;
  impl_->synchronousTrigger.call(srv);
  if (srv.response.result == -1) {
    throw std::runtime_error("VRep::synchronousTrigger failed");
  }
}

double VRep::getSimulationTime()
{
  vrep_common::simRosGetInfo srv;
  impl_->getInfo.call(srv);
  return srv.response.simulationTime;
}
