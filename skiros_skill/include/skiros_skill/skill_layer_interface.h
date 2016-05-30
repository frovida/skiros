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

#ifndef SKILL_LAYER_INTERFACE_H
#define SKILL_LAYER_INTERFACE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "skiros_skill/skill_manager_interface.h"
#include "skiros_msgs/WmMonitor.h"

namespace skiros_skill
{

    typedef std::map<std::string, SkillManagerInterfacePtr> SkillMgrsMap;
    typedef std::pair<std::string, SkillManagerInterfacePtr> SkillMgrsPair;
    class SkillLayerInterface
    {
    public:
        SkillLayerInterface();
        ~SkillLayerInterface(){}
        void wmMonitorCB(const skiros_msgs::WmMonitor& msg);
        SkillMgrsMap & getSkillMgrsMap(){return skill_mgrs_list_;}
        SkillMgrsMap::iterator find(std::string skill_manager){return skill_mgrs_list_.find(skill_manager);}

        bool hasChanged();
        void shutdown();
    private:
        bool new_changes_;
        std::set<std::string> agent_types_;
        SkillMgrsMap skill_mgrs_list_;
        ros::Subscriber wm_monitor_sub_;
        boost::shared_ptr<skiros_wm::WorldModelInterface> wm_ptr_;
        boost::shared_ptr<ros::NodeHandle> nh_ptr_;
    };

}

#endif // SKILL_LAYER_INTERFACE_H
