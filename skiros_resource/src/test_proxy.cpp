//#include <ncurses.h>  Commented by CS to allow build
#include <pluginlib/class_loader.h>
//#include <actionlib/server/simple_action_server.h>

#include <skiros_msgs/GripperAction.h>
#include <skiros_resource/base_arm_device.h>
#include <skiros_resource/base_gripper_device.h>
#include <skiros_resource/exceptions.h>
#include <skiros_msgs/Finger.h>

#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iterator>

void handle_CTRL_C_signal(int signum)
{
	   //ROS_INFO("Caught signal %d\n",signum);
	   // Cleanup and close up stuff here
	   ros::shutdown();
	   // Terminate program
	   exit(signum);
}

int main (int argc, char **argv){

	// Initialise the ros node
    ros::init(argc, argv, "resource_manager", ros::init_options::NoSigintHandler);
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */
	ros::NodeHandle node_h;

	signal(SIGINT, handle_CTRL_C_signal);// set the CTRL+C handler

    pluginlib::ClassLoader<skiros_resource::BaseArmDevice> arm_loader("skiros_resource", "skiros_resource::BaseArmDevice");
    pluginlib::ClassLoader<skiros_resource::BaseGripperDevice> gripper_loader("skiros_resource", "skiros_resource::BaseGripperDevice");

    boost::shared_ptr<skiros_resource::BaseArmDevice> arm_proxy;
    boost::shared_ptr<skiros_resource::BaseGripperDevice> gripper_proxy;
    std::vector<std::string> v;

	char input; bool go_forward = false; std::string prx_type;
	while(!go_forward)
	{
		ROS_INFO("Which type of proxy would you like to test? (a = arm; g = gripper)");
		std::cin >> input;
		switch(input)
		{
			case 'a':
                prx_type = "skiros_resource::BaseArmDevice";
				v = arm_loader.getDeclaredClasses();
				go_forward = true;
				break;
			case 'g':
                prx_type = "skiros_resource::BaseGripperDevice";
				v = gripper_loader.getDeclaredClasses();
				go_forward = true;
				break;
			default:
				ROS_INFO("%c is not a valid character.", input);
				break;
		}
	}

    std::vector<std::string>::iterator it = v.begin();
	int i = 1;int num_input = 1;go_forward = false;std::string prx_class;
	while(!go_forward)
	{
		i = 1;
		ROS_INFO("Choose the proxy (number) to load from the list ");
		for (it = v.begin() ; it != v.end(); ++it)
		{
			ROS_INFO("%i %s", i++, it->c_str());
		}
		std::cin >> num_input;
		if(num_input>0 && num_input<=v.size())
		{
			prx_class = v.at(num_input-1).c_str();
			ROS_INFO("%s", prx_class.c_str());
			go_forward = true;
		}
		else
		{
			ROS_INFO("Number %i not valid", num_input);
		}
	}

    try
    {
        if(prx_type == "skiros_resource::BaseArmDevice")
    	{
    		arm_proxy = arm_loader.createInstance(prx_class.c_str());
    	}
        else if(prx_type == "skiros_resource::BaseGripperDevice")
    	{
    		gripper_proxy = gripper_loader.createInstance(prx_class.c_str());
    	}
    }
    catch(pluginlib::PluginlibException& ex)
    {
		//handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

	ros::AsyncSpinner spinner(1); // Use 1 thread to handle callbacks

    if(prx_type == "skiros_resource::BaseArmDevice")
	{
        arm_proxy->setModality("bo");
	}
    else if(prx_type == "skiros_resource::BaseGripperDevice")
	{
        gripper_proxy->init(node_h);
		spinner.start();
		gripper_proxy->reset();
		gripper_proxy->activate();

		while(ros::ok)
		{
			num_input = -1;
			//For now this test is very rq3 specific...
			while(num_input < 0 || num_input > 259)
			{
				ROS_INFO("Which command do you want to execute? (0-255 goToPosition, 256 basic, 257 pinch, 258 wide, 259 scissor )");
				std::cin >> num_input;
			}
			if(num_input <= 255)
			{
				skiros_msgs::Finger f;
				f.pos[0] = num_input;
				f.spd[0] = 255;
				f.frc[0] = 150;
				std::vector<skiros_msgs::Finger> v;
				v.push_back(f);
				try
				{
					gripper_proxy->fingersToPos(0, v);
				}
				catch(skiros_device::EndEffectorException& e)
				{
					ROS_ERROR("problems");
				}
			}
			else
			{
				try
				{
					gripper_proxy->changeModality(num_input-256);
				}
				catch(skiros_device::EndEffectorException& e)
				{}
			}
		}
	}
	return 0;
} // main
