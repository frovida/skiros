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

#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "skiros_world_model/world_model_interface.h"
#include "skiros_skill/skill_layer_interface.h"
#include "skiros_msgs/WmMonitor.h"
#include <skiros_msgs/TmTaskExe.h>
#include <skiros_msgs/TmModifyTask.h>
#include <skiros_msgs/TmQueryTask.h>

namespace skiros_task
{

    /*!
     * \brief This struct contain the relevant info to describe a skill. Name, parameters and manager who howns it.
     */
	struct SkillHolder
	{
	  std::string manager;
	  std::string name;
	  skiros_common::ParamMap params;
	};

    /*! \brief Provides methods to monitor and control the Skill Managers present on the network
     *
     * The task manager constantly monitors the world model and , as long as a new skill manager appears,
     * it add it to the list of skill managers that the user can control.
     *
     */
    class TaskManager
    {
        skiros_skill::SkillLayerInterface skill_layer_;
        typedef std::list<SkillHolder> TaskType;
        boost::thread exe_thread_;
        bool running_;
        SkillHolder current_exe_skill_;
        boost::mutex task_modify_mux_;
        boost::mutex exe_mux_;
        TaskType task_;
        boost::shared_ptr<skiros_wm::WorldModelInterface> wm_ptr_;
        boost::shared_ptr<ros::NodeHandle> nh_ptr_;
        std::string save_path_;
        ros::Subscriber task_exe_sub_;
        ros::Publisher tm_monitor_pub_;
        std::string controller_id_ = "task_manager";
    public:
        TaskManager(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterface> wm);


        void setMissionId(std::string mission_id){controller_id_=mission_id;}

        bool startTask(bool iterate=false);
        void stopTask(){stopSkill();}

        void startModifingTask()
        {
            task_modify_mux_.lock();
        }

        void endModifingTask()
        {
            task_modify_mux_.unlock();
        }

        void clearSkillSequence(std::string author);
        bool insertSkillInSequence(std::string author, SkillHolder skill, int index=-1);
        bool removeSkillInSequence(std::string author, int index=-1);

        const skiros_skill::SkillMgrsMap & getRobotsInterface(){return skill_layer_.getSkillMgrsMap();}
        std::list<SkillHolder> & getSkillSequence(){return task_;}
        boost::shared_ptr<skiros_wm::WorldModelInterface> getWorldHandle(){return wm_ptr_;}

        bool exeModule();

        void exeTaskThread(bool iterate);

        void shutdown();
        //------ROS services ----------
        //! \brief
        bool taskQuery(skiros_msgs::TmQueryTaskRequest &req, skiros_msgs::TmQueryTaskResponse &res);
        //! \brief
        bool taskModify(skiros_msgs::TmModifyTaskRequest &req, skiros_msgs::TmModifyTaskResponse &res);

        //----- Save\Load methods ----------------
        bool loadSkillSequence(std::string filename);
        bool saveSkillSequence(std::string filename);
        std::string getSavePath();

        //----- UI methods ----------------
        bool setParams(SkillHolder & skill);
        void printAvailableMgrs(std::string indend = "");
        bool insertSkillInSequence();
        std::string printSkillSequence();
        bool exeSkillSequence();
    private:
        SkillHolder select(std::string what="skill");
        skiros_skill::ExecutionResult exeSkill(SkillHolder skill);
        void stopSkill();
        void taskExeCb(const skiros_msgs::TmTaskExe &msg);
        void publishTaskStatus(std::string author, SkillHolder skill, std::string status, int progress_code=0, std::string progress_description="");
        void advertiseTaskModification(std::string author);
        bool teachParam(skiros_common::Param &p, skiros_wm::Element robot);
    };
}

#endif // TASK_MANAGER_H
