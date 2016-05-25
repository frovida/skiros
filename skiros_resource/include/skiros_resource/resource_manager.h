#ifndef DEVICEMGR_HPP
#define DEVICEMGR_HPP

#include "ros/ros.h"
#include <map>
#include <vector>
#include <pluginlib/class_loader.h>
#include "skiros_common/node_spawner.h"
#include <skiros_resource/base_arm_device.h>
#include <skiros_resource/base_gripper_device.h>
#include "skiros_resource/definitions.h"
#include <skiros_resource/gripper_action_server.h>
#include "skiros_msgs/RmDeviceHandle.h"
#include <skiros_msgs/GripperAction.h>
#include "skiros_common/logger_sys.h"
#include <skiros_resource/exceptions.h>

namespace skiros_resource
{

struct DeviceType
{
    std::string device_type;
    DriverType driver;
    //----- TODO: generalize the following ----
    //If proxy //If gripper
    boost::shared_ptr<BaseGripperDevice> proxy;
    skiros_resource::GripperActionServer * gas;
};

struct ConfigParam
{
    std::string robot_urdf;
    std::string robot_srdf;
    std::string robot_joint_limits;
    std::string robot_kinematics;

    int device_num;
};


typedef std::pair<std::string, DeviceType> RegDevicePair;
typedef std::map<std::string, DeviceType> RegDeviceMap;

class ResourceManager
{
public:
    ResourceManager(ros::NodeHandle & nh, std::string manager_name);
    ~ResourceManager();


    bool deviceHandler(skiros_msgs::RmDeviceHandle::Request &req,
                       skiros_msgs::RmDeviceHandle::Response &res);

    void driverMonitor();

    bool launchDriver(DriverType &driver);

    std::string listDrivers();

    bool shutdownDriver(std::string address);
    bool shutdownDriver(RegDeviceMap::iterator it);
    bool shutdownAllDrivers();


private:
    boost::thread monitor_thread_;
    ros::Publisher monitor_pub_;
    skiros_common::RosNodeSpawner node_spawner_;

    boost::mutex devices_mutex_;
    RegDeviceMap registered_devices_;

    std::string ros_namespace_;
    ros::NodeHandle & nh_;
    pluginlib::ClassLoader<skiros_resource::BaseArmDevice> * arm_loader;
    pluginlib::ClassLoader<skiros_resource::BaseGripperDevice> * gripper_loader;
    //std::vector<Device*> device_library_;
    //ros::ServiceServer device_handling_srv_;


};


}

#endif //DEVICEMGR_HPP
