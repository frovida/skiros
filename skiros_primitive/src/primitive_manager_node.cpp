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

#include <pluginlib/class_loader.h>
#include <actionlib/server/simple_action_server.h>
#include <skiros_primitive/gripper_action_client.h>
#include <skiros_primitive/primitive_base.h>



/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv){

	// Initialise the ros node
	ros::init(argc, argv, "primitive_manager"); //ros::this_node::getName()
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */
	ros::NodeHandle node_h;

    //Initialize the gripper action client
	skiros_primitive::GripperClientClass gripper_ac("gripper_srv");

    // Use multiple threads to handle callbacks
	ros::AsyncSpinner spinner(3);
	spinner.start();

	// Load shared libraries:
  //  pluginlib::ClassLoader<skiros_lib_primitive::BaseGripperPrimitive> gripper_prim_loader("skiros_device", "skiros_primitive::BaseGripperPrimitive");

    /*
    //Show the available plugins
    std::vector<std::string> v = gripper_prim_loader.getDeclaredClasses();
    std::vector<std::string>::iterator it = v.begin();
	while (it != v.end()) {
		ROS_INFO("%s", it->c_str());
    }*/

//    boost::shared_ptr<skiros_primitive::BaseGripperPrimitive> gripper_primitive;

    //Load proxies
	//TODO: generalize
    try
    {
    	//gripper_primitive = gripper_prim_loader.createInstance("skiros_lib_primitive::GripperForceCtrl");
    }
    catch(pluginlib::PluginlibException& ex)
    {
		//handle the class failing to load
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

	int num_input = -1;


	while(num_input < 0 || num_input > 259)
	{
		ROS_INFO("Which command do you want to execute? (0-255 goToPosition)");
		std::cin >> num_input;
	}
	std:vector<int> v;
	v.push_back(255);
	//gripper_ac.test(1, 1, v);
	if(num_input <= 255)
	{
		//gripper_ac.grasp(2,5);
		gripper_ac.waitFinish();
	}
	gripper_ac.release();

	spinner.stop();

	return 0;
} // main

