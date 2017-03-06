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

#include <string>
#include <vector>
#include <ros/ros.h>
#include "skiros_common/utility.h"
#include "skiros_config/declared_uri.h"
#include "skiros_config/node_names.h"
#include <skiros_world_model/utility.h>
#include <skiros_skill/skill_layer_interface.h>

using namespace skiros_config::owl;
using namespace skiros_config;
using namespace skiros_skill;
using namespace skiros_wm;

namespace skiros_skill
{

SkillLayerInterface::SkillLayerInterface()
{
    new_changes_ = false;
    nh_ptr_.reset(new ros::NodeHandle());
    wm_ptr_.reset(new skiros_wm::WorldModelInterface(*nh_ptr_));
    agent_types_ = wm_ptr_->getSubClasses(concept::Str[concept::Agent]);
    wm_monitor_sub_ = nh_ptr_->subscribe(std::string(world_model_node_name)+wm_monitor_tpc_name, 20, &SkillLayerInterface::wmMonitorCB,  this);
    std::vector<skiros_wm::Element> v = wm_ptr_->resolveElement(skiros_wm::Element(concept::Str[concept::Agent]));
    if(v.size())
    {
      for(int i=0;i<v.size();i++)
      {
          std::string name = v[i].properties(data::Str[data::SkillMgr]).getValue<std::string>();
          if(skill_mgrs_list_.find(name) == skill_mgrs_list_.end())
          {
              SkillManagerInterfacePtr new_robot;
              FINFO("[SkillLayerInterface] Skill manager detected: " << name);
              new_robot.reset(new SkillManagerInterface(wm_ptr_, nh_ptr_, v[i]));
              skill_mgrs_list_.insert(SkillMgrsPair(name, new_robot));
              new_changes_ = true;
          }
      }
    }
}

void SkillLayerInterface::wmMonitorCB(const skiros_msgs::WmMonitor& msg)
{
  std::string answer = (agent_types_.find(msg.element.type)!=agent_types_.end()) ? "yes" : "no" ;
  FDEBUG("[SkillLayerInterface::wmMonitorCB]" << msg.element.type << " is an agent? : " << answer << ". Action: " << msg.action);
  if((msg.action == "add" || msg.action == "update") && agent_types_.find(msg.element.type)!=agent_types_.end())
  {
     skiros_wm::Element agent = msg2element(msg.element);
     std::string skill_mgr = agent.properties(data::Str[data::SkillMgr]).getValue<std::string>();
     if(skill_mgrs_list_.find(skill_mgr) == skill_mgrs_list_.end())
     {
        SkillManagerInterfacePtr new_robot;
        FINFO("[SkillLayerInterface::wmMonitorCB] New skill manager detected: " << skill_mgr);
        new_robot.reset(new SkillManagerInterface(wm_ptr_, nh_ptr_, wm_ptr_->getElement(msg.element.id)));
        skill_mgrs_list_[skill_mgr] = new_robot;
        //change_cb_(msg);
        new_changes_ = true;
     }
  }
  if(msg.action == "remove" && agent_types_.find(msg.element.type)!=agent_types_.end())
  {
    skiros_wm::Element agent = msg2element(msg.element);
    std::string skill_mgr = agent.properties(data::Str[data::SkillMgr]).getValue<std::string>();
    skill_mgrs_list_.erase(skill_mgrs_list_.find(skill_mgr));
    FINFO("[SkillLayerInterface::wmMonitorCB] Skill manager " << skill_mgr << " removed from control list");
    //change_cb_(msg);
    new_changes_ = true;
  }
}

bool SkillLayerInterface::hasChanged()
{
    if(new_changes_)
    {
        new_changes_ = !new_changes_;
        return true;
    }
    else
        return false;
}

void SkillLayerInterface::shutdown()
{
    skill_mgrs_list_.clear();
}

} //namespace skiros_skill
