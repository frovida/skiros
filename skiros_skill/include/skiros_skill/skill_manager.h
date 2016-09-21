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

#ifndef SKILL_MANAGER_H
#define SKILL_MANAGER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "skiros_common/serializable_any.h"
#include "skiros_common/param_handler.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_skill/skill_base.h"
#include "skiros_skill/module_base.h"
#include "skiros_skill/modules_interface.h"
#include "skiros_common/resource_manager_interface.h"
//Service msgs
#include "skiros_msgs/ModulesListQuery.h"
#include "skiros_msgs/SkillExe.h"
#include "skiros_msgs/RmMonitor.h"
#include "skiros_msgs/ModuleCommand.h"
#include <skiros_skill/module_monitor.h>


namespace skiros_skill
{

    /*! \brief The skill manager collects and loads the skills and offers interfaces to execute them over the ROS network
	 *
	 *  Detailed description: TODO
	 *
	 */
	class SkillManager
	{
	public:
        SkillManager(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model, std::string prefix, std::string robot_uri);
        ~SkillManager();
        //---------- Initialization functions ------------------
        //! \brief Register the robot on the world model and triggers a reload()
        bool registerRobot();
        //! \brief Call init and upload available skills, modules and devices on the world model
        void reload(bool reload_plugins=true);
        //! \brief Unload the plugins and de-register from the world model
        void shutdown();
    private:
        //! \brief Use pluginlib to instanciate and initialize all modules\skills
        void init();
        //! \brief Upload available skills on the world model
        void loadSkills();
        //! \brief Prototype TOFINISH
        void configureModule(std::string name);
        //! \brief Upload available modules on the world model (clear and load)
        void loadModules();
        //! \brief Upload available devices on the world model (clear and load)
        void loadDevices();
		//---------- ROS service functions ------------------
    public:
        //! \brief Return the list of available modules
        bool moduleListQuery(skiros_msgs::ModulesListQueryRequest &req, skiros_msgs::ModulesListQueryResponse &res);
        //! \brief Return the list of available skills
        bool skillListQuery(skiros_msgs::ModulesListQueryRequest &req, skiros_msgs::ModulesListQueryResponse &res);
        //! \brief Command a skill execution or stop
        bool skillCommandService(skiros_msgs::ModuleCommandRequest &req, skiros_msgs::ModuleCommandResponse &res);
        //! \brief Command a module execution or stop
        bool moduleCommandService(skiros_msgs::ModuleCommandRequest &req, skiros_msgs::ModuleCommandResponse &res);

        //---------- Threaded functions ------------------
        //! \brief Kill all running modules\skill
        void preemptAll();
    private:
        //! \brief Start a threaded skill execution
        void moduleStart(boost::shared_ptr<skiros::ModuleCore> module, std::string controller);
        //! \brief Kill the execution of a specific module
        void moduleKill(std::string module);
        void skillKill(std::string skill);
        //---------- Other functions ------------------
        /*!
         * \brief Retrieve all the software which the device is dependent to
         * \param device, a skiros device description (see the ontology to get an example)
         * \return the list of dependent drivers (see the ontology to get an example)
         */
        std::list<skiros_wm::Element> getDrivers(skiros_wm::Element device);
        /*!
         * \brief Load a driver using the resource manager node
         * \param driver, the driver description
         * \return
         */
        int loadDriver(skiros_wm::Element driver);
    public:
        //! \brief Prototype. Resource Manager still not used
        void RmMonitorCB(const skiros_msgs::RmMonitor& msg);
        //! \brief Monitor the world model to check if the scene gets reloaded
        void wmMonitorCB(const skiros_msgs::WmMonitor& msg);
	private:
        //Tf updater
        void tfPubTimerCB(const ros::TimerEvent& e);
        ros::Timer tfpub_timer_;
        //Interfaces
        boost::shared_ptr<ros::NodeHandle>  nh_;
        boost::shared_ptr<skiros_wm::WorldModelInterfaceS> wm_;
        boost::shared_ptr<SkillManagerInterface> mi_;
        skiros_common::ResourceManagerInterface * rm_;
        ros::Subscriber rm_monitor_;
        ros::Subscriber wm_monitor_;
        boost::mutex devices_mutex_;
        ros::Publisher state_pub_;

        //---- Plugins variables
        typedef std::map<std::string, boost::shared_ptr<skiros::ModuleMonitor> > ModuleMonitorsMapType;
        typedef std::pair<std::string, boost::shared_ptr<skiros::ModuleMonitor> > ModuleMonitorsPairType;
        //!< Threads to execute the skills
        ModuleMonitorsMapType module_monitors_;

        //Skills
        pluginlib::ClassLoader<skiros_skill::SkillBase> * skills_loader_;
        typedef std::map<std::string, boost::shared_ptr<SkillBase> > SkillsMapType;
        //!< Stores the name of the skills declared as plugins for this skill manager
        SkillsMapType loaded_skills_;
        typedef std::map<std::string, boost::shared_ptr<boost::thread> > RunningSkillsMapType;
        //!< Threads to execute the skills
        RunningSkillsMapType running_skills_;
        //Modules
        pluginlib::ClassLoader<skiros::ModuleBase> * module_loader_;
        typedef std::map<std::string, boost::shared_ptr<skiros::ModuleBase> > ModulesMapType;
        ModulesMapType loaded_modules_;
        typedef std::map<std::string, boost::shared_ptr<boost::thread> > RunningModulesMapType;
        //!< Threads to execute the modules
        RunningModulesMapType running_modules_;
        boost::mutex module_kill_mutex_;

        //Configuration variables
        //!< The robot name
        std::string robot_uri_;
        std::string robot_name_;

        std::vector<skiros_wm::Element> wm_devices_;
        typedef std::pair<std::string, int> DriverDevicePair;
        typedef std::map<std::string, int> DriverDeviceMap;
        //!< A list referencing the resources loaded on the resource managers. Address->Device ID
        DriverDeviceMap driver_device_map_;
	};

} // namespace skiros_skill

#endif // SKILL_MANAGER_H
