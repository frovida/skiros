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

#include <typeinfo>
#include <signal.h>
#include <ros/ros.h>
#include "skiros_common/param.h"
#include "skiros_common/logger_sys.h"
#include "skiros_common/param_handler.h"
#include "skiros_config/param_types.h"
#include "skiros_config/node_names.h"
#include "skiros_skill/skill_manager.h"

using namespace skiros_skill;
using namespace skiros_config;

//Assure unregister the robot before shutting down
bool KEEP_RUNNING = true;

/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////


void sigIntHandler(int sig)
{
  KEEP_RUNNING = false;
}

// Get keyboard input from user
void input(SkillManager * manager)
{
    while(ros::ok())
    {
        char c = 'n';
        std::cout << "Type: 'r' reload drivers, 'k' kill a skill execution." << std::endl;
        std::cin >> c;
        if(c=='c')KEEP_RUNNING = false;
        else if (c=='r')
        {
            manager->reload();
        }
        else if (c=='k')
        {
            manager->preemptAll();
        }
    }
}


/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv)
{
    // Initialise the ros node
    ros::init(argc, argv, skill_mgr_node_name, ros::init_options::NoSigintHandler);
    // Override default exit handlers for roscpp
    signal(SIGINT, sigIntHandler);
    boost::shared_ptr<ros::NodeHandle> nh_ptr(new ros::NodeHandle());
    ros::NodeHandle skiros_nh(skiros_namespace);
    std::string robot_name_prefix;

    if(argc>1)
    {
        double delay_start_time = atof(argv[1]);
        ros::Duration(delay_start_time).sleep();
    }
    if(argc>2)
    {
        robot_name_prefix = argv[2];
    }

    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

	// Use multiple threads to handle callbacks
    ros::AsyncSpinner spinner(4);
	spinner.start();

    ///Instanciate a skill manager class
    boost::shared_ptr<skiros_wm::WorldModelInterfaceS> wm_ptr(new skiros_wm::WorldModelInterfaceS(*nh_ptr));
    wm_ptr->startTfListener();
    std::string robot_name = ros::this_node::getName();
    robot_name.erase(0,1);//Remove slash
    SkillManager manager(nh_ptr, wm_ptr, robot_name_prefix, robot_name);

    ///Create services:
    //-skill_list_query
    ros::ServiceServer skill_list_query = nh_ptr->advertiseService(ros::this_node::getName()+skill_list_query_srv_name, &skiros_skill::SkillManager::skillListQuery, &manager);
    //-module_list_query
    ros::ServiceServer module_list_query = nh_ptr->advertiseService(ros::this_node::getName()+module_list_query_srv_name, &skiros_skill::SkillManager::moduleListQuery, &manager);
    //-Command: skill command service
    ros::ServiceServer skill_command_srv = nh_ptr->advertiseService(ros::this_node::getName()+skill_cmd_srv_name,&skiros_skill::SkillManager::skillCommandService, &manager);
    //-Command: module command service
    ros::ServiceServer module_command_srv = nh_ptr->advertiseService(ros::this_node::getName()+module_cmd_srv_name,&skiros_skill::SkillManager::moduleCommandService, &manager);

    ///Wait the world model to be up and register the robot presence
    if(!wm_ptr->isConnected()) FWARN("World model seems down, waiting...")
    while(!wm_ptr->isConnected() && KEEP_RUNNING) {ros::spinOnce(); sleep(0.5);}
    FINFO("Connected to world model");
    while(!manager.registerRobot() && KEEP_RUNNING)
	{
        FWARN("World model seems down, waiting...")
        while(!wm_ptr->isConnected() && KEEP_RUNNING) sleep(1);
    }

    ///Start the keyboard input
    boost::thread get_input(input, &manager);

    ///Configure the logger (based on ROS param value)
    bool save_log;
    skiros_nh.param(skiros_config::save_log, save_log, false);
    skiros_common::InitLogger("skiros_skill", save_log, save_log, save_log);

    ///Infinite ROS queue spin
    while(ros::ok() && KEEP_RUNNING)
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    //----------------------- Close --------------------------------
    //std::cout << "Closing" << std::endl;
    manager.shutdown();
	spinner.stop();
	ros::shutdown();
	return 0;
} // main
