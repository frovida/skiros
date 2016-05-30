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

#include <ros/ros.h>
#include <typeinfo>
#include <dirent.h>
#include "skiros_common/param.h"
#include "skiros_common/logger_sys.h"
#include "skiros_common/param_handler.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_msgs/WmMonitor.h"
#include "skiros_task/task_manager.h"
#include "skiros_config/param_types.h"
#include "skiros_config/node_names.h"
#include "boost/thread.hpp"
#include "boost/foreach.hpp"

//To load interfaces
#include <pluginlib/class_loader.h>
#include "skiros_task/external_interface.h"

using namespace skiros_task;
using namespace skiros_config;

/////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

// Get keyboard input from user
void input(TaskManager * tm)
{
    //Loops
    while(ros::ok())
    {
        // Allow user to create a skill-sequence and save to a xml file
        // Allow the load of a task from xml file and subsequently execution
        char c = 'n';
        std::string name;
        bool loop = true;
        std::cout << "Actual skill sequence: " << std::endl;
        if(tm->printSkillSequence()!="") std::cout << tm->printSkillSequence();
        else std::cout << "Empty" << std::endl;
        std::cout << std::endl << "Task menu: 'p'-visualize available skills, 'a'-add skill, 'r'-remove skill, 'e'-execute the skill list, 'i' iterate execution, 'm'-execute a module, 's'-save list, 'l'-load list" << std::endl;
        std::cin >> c;
        switch(c)
        {
          case 'm'://exe a module
            tm->exeModule();
            break;
          case 'a'://add a skill
            tm->insertSkillInSequence();
            break;
          case 'r'://remove a skill
            tm->removeSkillInSequence("task_manager");
            break;
          case 'e'://execute skill list
            tm->exeSkillSequence();
            break;
          case 's'://save
            std::cout << "Please, enter the file name (standard name: 'my_world_state.xml'): " << std::endl;
            std::getline (std::cin,name);
            std::cout << "Saving file " << name << "..." << std::endl;
            if(tm->saveSkillSequence(name))std::cout << "File saved successfully." << std::endl;
            break;
          case 'l'://load a skill sequence from file
            struct dirent *dirpent;
            DIR * dirp;
            dirp = opendir(tm->getSavePath().c_str());
            std::cout << tm->getSavePath() << ": " << std::endl;

            if(dirp)
            {
                std::vector<std::string> v;int i = 0;
                //List the .xml files found in the directory
                while((dirpent=readdir(dirp)) != NULL)
                {
                  std::string temp(dirpent->d_name);
                  if(temp.find(".xml") != std::string::npos)
                  {
                  std::cout << i++ << ": " << dirpent->d_name << std::endl;
                  v.push_back(temp);
                  }
                }
                closedir(dirp);
                //Wait for a selection
                while(true)
                {
                  if(v.size()>0)
                  {
                std::cout << "Select the file to load. (number)" << std::endl;
                std::cin >> i;
                if(i>=0 && i < v.size())
                {
                    if(tm->loadSkillSequence(v[i]))std::cout << "File loaded successfully." << std::endl;
                    break;
                }
                  }
                  else
                  {
                  std::cout << "No files to load." << std::endl;
                  break;
                  }
                }
            }
            else  std::cout << "Problem occured while opening the directory." << std::endl;
            break;
         case 'c'://close
            loop = false;
            break;
         case 'p'://Print available robots\skills\modules
            tm->printAvailableMgrs();
            break;
         case 'i'://Iterate a skill sequence execution
            while(tm->exeSkillSequence());
         default:
            break;
         }
    }
}

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////


int main (int argc, char **argv){

    /*
      -find the agents in the world (skill_mgrs)
      -query the agents for the skill list
      -query the agents for the skill parameter list -> yes
      -make the agents execute a skill
    */

	// Initialise the ros node
    ros::init(argc, argv, task_mgr_node_name);
	ros::NodeHandle nh;
    ros::NodeHandle skiros_nh(skiros_namespace);
    ros::NodeHandle private_nh(ros::this_node::getName());

	// Use multiple threads to handle callbacks
    ros::AsyncSpinner spinner(4);
    spinner.start();

    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

    //Wait the world model to be up
    skiros_wm::WorldModelInterface wm(nh, true);
    if(!wm.isConnected())
        FWARN("World model seems down, waiting...");

    while(!wm.isConnected() && ros::ok())
        ros::Duration(0.5).sleep();

    FINFO("Connected to world model");

    boost::shared_ptr<skiros_wm::WorldModelInterface> wm_ptr(&wm);
    boost::shared_ptr<ros::NodeHandle> nh_ptr(&nh);
    //Create the task manager
    TaskManager tm(nh_ptr, wm_ptr);

    //Load interfaces modules
    pluginlib::ClassLoader<ExternalInterface> * interfaces_loader = new pluginlib::ClassLoader<ExternalInterface>("skiros_task", "skiros_task::ExternalInterface");

    std::vector<std::string> module_list = interfaces_loader->getDeclaredClasses();
    typedef std::map<std::string, boost::shared_ptr<ExternalInterface> > ModulesMapType;
    ModulesMapType loaded_modules;

    BOOST_FOREACH(std::string module, module_list)
    {
        FINFO("Loading module: " << module);
        try{
        loaded_modules[module] = interfaces_loader->createInstance(module);
        loaded_modules[module]->init(&tm, &private_nh);
        loaded_modules[module]->execute();
        }
        catch(pluginlib::PluginlibException& ex)
        {
            FERROR("[TaskManager]The module " << module << " failed to load. Error: " << ex.what());
        }
    }
    //Start the keyboard input
    boost::thread get_input(input, &tm);

    //Configure the logger (based on ROS param value)
    bool save_log;
    skiros_nh.param(skiros_config::save_log, save_log, false);
    skiros_common::InitLogger("skiros_task", save_log, save_log, save_log);
    //Create services:
    ros::ServiceServer task_query = nh.advertiseService(ros::this_node::getName()+task_query_srv_name, &TaskManager::taskQuery, &tm);
    ros::ServiceServer task_modify = nh.advertiseService(ros::this_node::getName()+task_modify_srv_name, &TaskManager::taskModify, &tm);
    //Infinite ROS queue spin
    ros::spin();

    //------------------- CLOSE ------------------------
    tm.shutdown();
    spinner.stop();
    ros::shutdown();
    return 0;

} // main
