#include "ros/ros.h"
#include <fstream>
#include "skiros_resource/definitions.h"
#include "skiros_resource/resource_manager.h"
#include "pluginlib/class_loader.h"
#include <skiros_msgs/RmMonitor.h>
#include <boost/foreach.hpp>


using namespace std;
using namespace skiros_common;

namespace skiros_resource
{
//TODO: all 'device' term must be traduced to 'resource'
bool ResourceManager::deviceHandler(skiros_msgs::RmDeviceHandle::Request &req,
                   skiros_msgs::RmDeviceHandle::Response &res)
{
    boost::unique_lock<boost::mutex> lock(devices_mutex_);
    lock.unlock();
    skiros_msgs::RmMonitor msg;
    if(req.action==skiros_msgs::RmDeviceHandle::Request::START)
    {
        if(registered_devices_.find(req.id_name) != registered_devices_.end())
        {
            FERROR("Device with address " << req.id_name << " is already running.");
            res.success = true;
            return true;
        }
        DeviceType d;
        if(req.package=="skiros_proxy") //Load a proxy
        {
            if(req.type == "Gripper") //Load a gripper
            {
                try
                {
                    std::string driver_name = "skiros_lib_proxy::" + req.executable;
                    d.proxy = gripper_loader->createInstance(driver_name.c_str());
                }
                catch(pluginlib::PluginlibException& ex)
                {
                    //handle the class failing to load
                    FERROR("[deviceHandler]The plugin failed to load. Error: " << ex.what());
                    res.success = false;
                    return true;
                }
                //Load eventually the driver
                d.driver = d.proxy->getRelatedDriver();
                if(d.driver.package!="")
                {
                    d.driver.id_name = req.id_name; //Name gets overwritten
                    d.driver.parameters += req.parameters; //Parameters are added
                    if(!launchDriver(d.driver))
                    {
                        res.success = false;
                        return true;
                    }
                }
                else
                {
                    d.driver.pid = -1;
                }
                //Load the action server
                d.gas = new skiros_resource::GripperActionServer(ros_namespace_+req.id_name, d.proxy, nh_);
                FINFO("Gripper action server activated at address " << req.id_name << " using proxy " << req.executable << ".");
            }
        }
        else //Load a ROS node
        {
            d.driver.id_name = req.id_name;
            d.driver.package = req.package;
            d.driver.executable = req.executable;
            d.driver.parameters = req.parameters;
            if(!launchDriver(d.driver))
            {
                res.success = false;
                return true;
            }
        }
        d.device_type = req.type;
        d.driver.state = ""; //TODO: connect this to spawner data
        //Add to list
        lock.lock();
        registered_devices_.insert(RegDevicePair(d.driver.id_name, d));
        lock.unlock();
        msg.event_type = skiros_msgs::RmMonitor::STARTED;
        msg.resource_address = req.id_name;
        monitor_pub_.publish(msg);
        //TODO: publish in the monitor
        /*skiros_msgs::NodeEvent msg;
        msg.header.stamp.setNow(ros::Time::now());
        msg.event_type = skiros_msgs::NodeEvent::NODE_STARTED;
        msg.node_name = progname;
        __pub_node_events.publish(msg);*/
        sleep(1);
        res.success = true;
        return true;
    }
    //----------------------------------------------------------------------------------
    else if(req.action==skiros_msgs::RmDeviceHandle::Request::STOP)
    {
        res.success = shutdownDriver(req.id_name);
        return true;
    }
    //----------------------------------------------------------------------------------
    else if(req.action==skiros_msgs::RmDeviceHandle::Request::STOP_ALL)
    {
        res.success = shutdownAllDrivers();
        return true;
    }
    //----------------------------------------------------------------------------------
    else if(req.action==skiros_msgs::RmDeviceHandle::Request::PING)
    {
        res.success = true;
        return true;
    }
    return false;
}


void ResourceManager::driverMonitor()
{
    skiros_msgs::RmMonitor msg;

    while(ros::ok())
    {
        if(!node_spawner_.hasEvent())sleep(1);
        else
        {
            SpawnerProcessData pd = node_spawner_.getEvent();
            if(pd.status == "Terminated")
            {
                for(RegDeviceMap::iterator it=registered_devices_.begin();it!=registered_devices_.end();it++)
                {
                    DeviceType d = it->second;
                    if(d.driver.pid==pd.id)
                    {
                        msg.event_type = skiros_msgs::RmMonitor::TERMINATED;
                        msg.resource_address = d.driver.id_name;
                        monitor_pub_.publish(msg);
                        shutdownDriver(it);
                        //FWARN("[ResourceManager::driverMonitor] Driver for device '" << d.device_type << "' (address: " << d.address << " ) terminated.");
                    }
                }
            }
        }
    }

}

bool ResourceManager::shutdownDriver(std::string address)
{
    RegDeviceMap::iterator it = registered_devices_.find(address);
    if(it == registered_devices_.end())
    {
        FERROR("[deviceHandler::STOP] No device with name " << address << ".");
        return false;
    }
    FINFO("Closing node: " << address);
    return shutdownDriver(it);
}

bool ResourceManager::shutdownDriver(RegDeviceMap::iterator it)
{
    if(it->second.driver.package =="skiros_proxy") //Unload a proxy
    {
        if(it->second.device_type == "Gripper") //Unload a gripper
        {
            //delete it->second.gas; //Destroy the action server
            //it->second.proxy.reset(); //Destroy the proxy
        }
    }
    else //Unload a ROS node
    {
        node_spawner_.shutdown(it->second.driver.pid);
    }
    //De-index
    boost::mutex::scoped_lock lock(devices_mutex_);
    registered_devices_.erase(it);
    return true;
}

bool ResourceManager::shutdownAllDrivers()
{
    while(!registered_devices_.empty()) shutdownDriver(registered_devices_.begin());
    return true;
}

bool ResourceManager::launchDriver(DriverType & driver)
{
    /* Launch a device with "device_name". Device_name is in skiros: "type"_"name"; e.g. gripper_RQ3
     *
     * Returns 0 on success and one of the following error
     *codes on failure:
     *
     *-1: Device wasn't recognized
     *-2: No device driver specified (in device description file)
     *
     */
    std::string global_namespace = ros_namespace_;
    if(driver.parameters.find("__ns")!=std::string::npos)
    {
        std::stringstream ss(driver.parameters);
        std::string temp;
        driver.parameters = "";
        while(!ss.eof())
        {
            ss >> temp;
            if(temp.find("__ns")!=std::string::npos)
            {
                global_namespace += "/"+ temp.substr(6);
            }
            else driver.parameters += temp + " ";
        }
    }
    std::string argv = driver.parameters + " __name:=" + driver.id_name + " __ns:=" + global_namespace;
    //If its a py file I use a special policy - NOTE: this is a temporary workaround
    if(driver.executable.find(".py")!=std::string::npos)
    {
        driver.executable = "aau.launch";
        argv = " __name:=" + driver.id_name + " __ns:=" + global_namespace;
    }
    driver.pid = node_spawner_.execute(driver.package, driver.executable, argv);
    if(driver.pid<=0)
    {
        FERROR("[ResourceManager::launchDriver]Failed to launch the ROS node.");
        return false;
    }
    return true;
}

std::string ResourceManager::listDrivers()
{
    std::stringstream ss;
    std::vector<skiros_common::SpawnerProcessData> list = node_spawner_.listLoaded();
    BOOST_FOREACH(skiros_common::SpawnerProcessData data, list)
    {
        ss << data.id << " " << data.exe_name << " " << data.status << std::endl;
    }
    return ss.str();
}

//------------------------ Utility ----------------------


ResourceManager::ResourceManager(ros::NodeHandle & nh, std::string ros_namespace) : nh_(nh), ros_namespace_(ros_namespace)
{
    //instanciate the plugin loader (proxy loader)
    arm_loader = new pluginlib::ClassLoader<skiros_resource::BaseArmDevice>("skiros_resource", "skiros_resource::BaseArmDevice");
    gripper_loader = new pluginlib::ClassLoader<skiros_resource::BaseGripperDevice>("skiros_resource", "skiros_resource::BaseGripperDevice");
    monitor_pub_ = nh_.advertise<skiros_msgs::RmMonitor>(ros::this_node::getName()+"/monitor", 10);
    monitor_thread_ = boost::thread(boost::bind(&ResourceManager::driverMonitor, this));
}

ResourceManager::~ResourceManager()
{

}

}
