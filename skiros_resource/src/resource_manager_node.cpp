/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Francesco Rovida
 *	Robotics, Vision and Machine Intelligence Laboratory
 *  Aalborg University, Denmark
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Aalborg Universitet nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <actionlib/server/simple_action_server.h>

// basic file operations
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "skiros_msgs/RmDeviceHandle.h"
#include "skiros_resource/resource_manager.h"
#include "skiros_common/logger_sys.h"

using namespace skiros_resource;

bool KEEP_RUNNING;

/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

// Get keyboard input from user
void input(ResourceManager * rm)
{
    while(ros::ok())
    {
        char c = 'n';
        std::cout << "Type: 'l' list drivers, 'c' close." << std::endl;
        std::cin >> c;
        if(c=='c')KEEP_RUNNING = false;
        else if (c=='l')
        {
            std::cout << rm->listDrivers() << std::endl;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
    // Initialise the ros node
    if(argc<2)
    {
        std::cout << "Must specify the robot description to be loaded. Closing.";
        return -1;
    }
    std::string robot_name(argv[1]);
    std::string mgr_name = robot_name + "_rm"; //TODO: if argv[2] exist should be the number of the rm
    ros::init(argc, argv, mgr_name.c_str());
	ros::NodeHandle node_h;
    KEEP_RUNNING = true;
	
    //TODO: Get a list of devices to handle. -> access to the ontology? why not...

	//Get other configuration from ros parameters
    /*bool have_arm; // haveCamera, haveGripper;
	std::string arm_address, arm_type;
	if (ros::param::get("/arm/type", arm_type))
    {
		    if (ros::param::get("/arm/address", arm_address))
			{
				have_arm = true;
				ROS_INFO("%s", arm_type.c_str());
				ROS_INFO("%s", arm_address.c_str());
			}
			else
			{
				ROS_ERROR("The address of the arm was not specified");
			}
    }*/

    // Use multiple threads to handle callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ResourceManager rm(node_h, robot_name);
    //Create services:
    std::string service_name = ros::this_node::getName()+"/device_handler";
    //FINFO(service_name);
    ros::ServiceServer device_handling_srv_= node_h.advertiseService(service_name, &ResourceManager::deviceHandler, &rm);

    //Start the keyboard input
    boost::thread get_input(input, &rm);

    //Loops ROS queue spin
    while(ros::ok() && KEEP_RUNNING)ros::spinOnce();

    FINFO("Closing");
    //----------------------- Close --------------------------------
    spinner.stop();
    ros::shutdown();
	return 0;
} // main
/*
//Show the available proxies
 std::vector<std::string> v = arm_loader.getDeclaredClasses();
std::vector<std::string>::iterator it = v.begin();
while (it != v.end()) {
    ROS_INFO("%s", it->c_str());
}*/
