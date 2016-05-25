//#include <ncurses.h>  Commented by CS to allow build
#include <pluginlib/class_loader.h>
//#include <actionlib/server/simple_action_server.h>

#include <skiros_msgs/GripperAction.h>
#include <skiros_resource/base_arm_device.h>
#include <skiros_resource/base_gripper_device.h>
#include <skiros_resource/exceptions.h>
#include <skiros_resource/gripper_action_server.h>
#include <skiros_msgs/Finger.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iterator>

using namespace skiros_resource;

int main (int argc, char **argv){


    if(argc<4)
    {
        std::cout << "Usage: device_node <device_type> <device_proxy>" << std::endl
                  << std::endl
                  << "<device_type>: ArmDevice OR GripperDevice" << std::endl;
        return -1;
    }
    std::string device_type(argv[1]);
    std::string proxy_name(argv[2]);
    //std::string as_name(argv[3]);
    std::string node_name = "generic_device";

	// Initialise the ros node
    ros::init(argc, argv, node_name);
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */
	ros::NodeHandle node_h;

    pluginlib::ClassLoader<skiros_resource::BaseArmDevice> arm_loader("skiros_resource", "skiros_resource::BaseArmDevice");
    pluginlib::ClassLoader<skiros_resource::BaseGripperDevice> gripper_loader("skiros_resource", "skiros_resource::BaseGripperDevice");

    boost::shared_ptr<skiros_resource::BaseArmDevice> arm_proxy;
    boost::shared_ptr<skiros_resource::BaseGripperDevice> gripper_proxy;
    std::vector<std::string> v;
    if(device_type == "ArmDevice")v = arm_loader.getDeclaredClasses();

    if(device_type == "GripperDevice")v = gripper_loader.getDeclaredClasses();

    //Check that the selected proxy exists
    bool found=false;
    for (int i = 0; i < v.size(); i++)
    {
        if(proxy_name==v[i])
        {
            found = true;
            break;
        }
    }
    if(!found)
    {
        std::cout << "Proxy not found.";
        return -1;
    }
    //Create the proxy
    try
    {
        if(device_type == "ArmDevice")
    	{
            arm_proxy = arm_loader.createInstance(proxy_name.c_str());
    	}
        else if(device_type == "GripperDevice")
    	{
            gripper_proxy = gripper_loader.createInstance(proxy_name.c_str());
    	}
    }
    catch(pluginlib::PluginlibException& ex)
    {
		//handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

    ros::AsyncSpinner spinner(1); // Use 1 thread to handle callbacks
    spinner.start();
    //Starts the action server
    if(device_type == "GripperDevice")
    {
        GripperActionServer gas(ros::this_node::getName(), gripper_proxy, node_h);
        while(!gas.start()) sleep(1);
        ros::spin();
    }
    spinner.stop();
	return 0;
} // main
