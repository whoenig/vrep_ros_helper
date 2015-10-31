#include <ros/ros.h>
#include "vrep_ros_common/vrep.h"
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Empty.h>
#include <signal.h>

VRep* g_pVrep;

void mySigintHandler(int sig)
{
  g_pVrep->stopSimulation();
  g_pVrep->synchronous(false);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vrep_ros_clock_server_node");

  ros::NodeHandle nh;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("clock", 10);

  // avoid TF_OLD_DATA error (see http://wiki.ros.org/tf/Errors%20explained#Error:_TF_OLD_DATA)
  {
    ros::Publisher pub = nh.advertise<std_msgs::Empty>("/reset_time", 1, true);
    std_msgs::Empty msg;
    for (size_t i = 0; i < 100; ++i) {
      pub.publish(msg);
      ros::spinOnce();
    }
  }

  rosgraph_msgs::Clock msg;

  VRep vrep;
  g_pVrep = &vrep;

  vrep.synchronous(true);
  vrep.startSimulation();

  while (ros::ok()) {
    vrep.synchronousTrigger();

    float time = vrep.getSimulationTime();
    msg.clock.sec = (int)time;
    msg.clock.nsec = (int)((time - msg.clock.sec) *  1e9);
    pub.publish(msg);

    ros::spinOnce();
  }

  // std::vector<std::string> objectNames;
  // nh.getParam("objects", objectNames);

  // for (auto& objName : objectNames) {
  //   uint32_t handle = vrep.getObjectHandle(objName);
  //   ROS_INFO("%d: '%s'", handle, objName.c_str());
  //   vrep.enablePublisher("topic", 10, simros_strmcmd_get_transform, handle, -1, "");
  // }
}


    // #include <ros/ros.h>
    // #include <vrep_common/VrepInfo.h>
    // #include <rosgraph_msgs/Clock.h>

    // ros::Publisher clock_publisher;
    // ros::Subscriber vrep_clock_subscriber;

    // float sim_time = -1;

    // void updateJointStates(const vrep_common::VrepInfo::ConstPtr& new_time){
    //    if(sim_time != new_time->simulationTime.data){
    //       sim_time = new_time->simulationTime.data;
    //       rosgraph_msgs::Clock c;
    //       c.clock.sec = (int)sim_time;
    //       c.clock.nsec = (int)((sim_time - (int)sim_time)*1000000000);
    //       clock_publisher.publish(c);
    //    }
    //    else
    //       sim_time = new_time->simulationTime.data;
    // }

    // int main(int argc, char** argv){
    //    ros::init(argc,argv,"vrep_clock_remapper");
    //    ros::NodeHandle node_handle;
    //    vrep_clock_subscriber = node_handle.subscribe("/vrep/info",1,updateJointStates);
    //    clock_publisher = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);
    //    ros::spin();
    // }
