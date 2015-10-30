#include "vrep.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosGetObjectGroupData.h"

#include "v_repConst.h"

VRep::VRep()
{
}

void VRep::startSimulation()
{
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
  vrep_common::simRosStartSimulation srv;
  client.call(srv);
}

uint32_t VRep::getObjectHandle(
  const std::string& name)
{
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
  vrep_common::simRosGetObjectHandle srv;
  srv.request.objectName = name;
  client.call(srv);
  return srv.response.handle;
}

bool VRep::getObjectPose(
  uint32_t handle,
  geometry_msgs::PoseStamped& result)
{
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
  vrep_common::simRosGetObjectPose srv;
  srv.request.handle = handle;
  srv.request.relativeToObjectHandle = -1;//absolute pose
  client.call(srv);
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
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosEnablePublisher>("/vrep/simRosEnablePublisher");
  vrep_common::simRosEnablePublisher srv;
  srv.request.topicName = topicName;
  srv.request.queueSize = queueSize;
  srv.request.streamCmd = streamCmd;
  srv.request.auxInt1 = auxInt1;
  srv.request.auxInt2 = auxInt2;
  srv.request.auxString = auxString;
  client.call(srv);
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
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
  vrep_common::simRosEnableSubscriber srv;
  srv.request.topicName = topicName;
  srv.request.queueSize = queueSize;
  srv.request.streamCmd = streamCmd;
  srv.request.auxInt1 = auxInt1;
  srv.request.auxInt2 = auxInt2;
  srv.request.auxString = auxString;
  client.call(srv);
  return srv.response.subscriberID;
}

void VRep::getObjectNames(
  std::vector<std::string>& result)
{
  ros::ServiceClient client = handle_.serviceClient<vrep_common::simRosGetObjectGroupData>("/vrep/simRosGetObjectGroupData");
  vrep_common::simRosGetObjectGroupData srv;
  srv.request.objectType = sim_appobj_object_type;
  srv.request.dataType = 0; // retrieves the object names
  client.call(srv);
  result.clear();
  result.insert(result.begin(), srv.response.strings.begin(), srv.response.strings.end());
}
