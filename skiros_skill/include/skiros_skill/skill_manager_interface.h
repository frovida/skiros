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

#ifndef SKILL_MANAGER_INTERFACE_H
#define SKILL_MANAGER_INTERFACE_H

#include <ros/ros.h>
#include "skiros_world_model/world_model_interface.h"
#include <skiros_msgs/WmMonitor.h>
#include <skiros_skill/skill_manager_interface.h>
#include <skiros_msgs/ModuleStatus.h>

namespace skiros_skill
{
    typedef std::map<std::string, skiros_common::ParamMap> ModuleMapType;
    typedef std::pair<std::string, skiros_common::ParamMap> ModulePairType;
    /*!
     * \brief The result of a module\skill execution exepressed as status-progress_code-progress_description
     */
    struct ExecutionResult
    {
        ExecutionResult(): status(""), progress_code(0), progress_description(""){}
        std::string status;
        int progress_code;
        std::string progress_description;
        skiros_common::ParamMap output;
    };
    typedef boost::function<void (const skiros_msgs::ModuleStatus&)> FeedbackCallback;
    typedef boost::function<void (const skiros_msgs::ModuleStatus&)> DoneCallback;
    /*!
     * \brief Hold the callbacks (feedback and done) to monitor the module's execution
     */
    struct ExecutionCbs
    {
        ExecutionCbs(FeedbackCallback feedbackFct, DoneCallback doneFct) : feedbackCb(feedbackFct), doneCb(doneFct) {}
        ExecutionCbs() : feedbackCb(NULL), doneCb(NULL) {}
        ExecutionResult result;
        FeedbackCallback feedbackCb;
        DoneCallback doneCb;
    };

    /*!
     * \brief The SkillManagerInterface class defines high-level methods to communicate with a skill manager
     */
    class SkillManagerInterface
    {
    public:
        SkillManagerInterface(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                              boost::shared_ptr<ros::NodeHandle> nh,
                              std::string skill_manager_name);
        SkillManagerInterface(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                              boost::shared_ptr<ros::NodeHandle> nh,
                              skiros_wm::Element robot);
        ~SkillManagerInterface(){}

        ModuleMapType getSkillList(bool update=false);
        ModuleMapType getModuleList(bool update=false);
        ExecutionResult waitResult(std::string module_name);
        int exeModule(std::string module_name, skiros_common::ParamMap params, std::string author, DoneCallback doneCb=NULL, FeedbackCallback feedbackCb=NULL);
        int exeSkill(std::string skill_name, skiros_common::ParamMap params, std::string author, DoneCallback doneCb=NULL, FeedbackCallback feedbackCb=NULL);
        bool stopModule(int execution_id, std::string module_name, std::string author);
        bool stopSkill(int execution_id, std::string skill_name, std::string author);
        skiros_common::ParamMap getParams(std::string skill_name);
        int print(std::string indend);
        std::string selectSkill();
        std::string selectModule();

        void wmMonitorCB(const skiros_msgs::WmMonitor& msg);
        void moduleMonitorCB(const skiros_msgs::ModuleStatus& msg);
        inline bool isConnected()
        {
            return ros::service::exists(list_client_.getService(),true);
        }

        inline skiros_wm::Element getRobotElement(){return robot_;}

        bool hasChanged()
        {
            if(new_changes_)
            {
                new_changes_ = !new_changes_;
                return true;
            }
            else
                return false;
        }

    private:
        void addModuleInList(skiros_wm::Element e);
        void addSkillInList(skiros_wm::Element e);
        void init();
        bool new_changes_;
        std::set<std::string> skill_set_;
        std::set<std::string> module_set_;
        skiros_wm::Element robot_;
        std::string skill_mgr_name_;
        ModuleMapType skill_list_;
        ModuleMapType module_list_;
        ros::ServiceClient list_client_;
        ros::ServiceClient skill_exe_client_;
        ros::ServiceClient module_exe_client_;

        ros::Subscriber wm_monitor_sub_;

        boost::condition_variable monitor_cond_;
        boost::mutex monitor_mux_;
        std::map<std::string, ExecutionCbs> monitored_modules_;
        ros::Subscriber module_monitor_sub_;

        boost::shared_ptr<skiros_wm::WorldModelInterface> wm_;
        boost::shared_ptr<ros::NodeHandle> nh_;
    };
    typedef boost::shared_ptr<SkillManagerInterface> SkillManagerInterfacePtr;
}

#endif // SKILL_MANAGER_INTERFACE_H
