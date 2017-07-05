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

#include "skiros_config/node_names.h"
#include "skiros_common/utility.h"
#include "skiros_skill/skill_manager_interface.h"
#include "skiros_msgs/ModulesListQuery.h"
#include "skiros_msgs/ModuleCommand.h"
#include "skiros_config/declared_uri.h"
#include "boost/foreach.hpp"
#include <skiros_world_model/utility.h>

using namespace skiros_config;
using namespace skiros_config::owl;
using namespace skiros_wm;

namespace skiros_skill
{

SkillManagerInterface::SkillManagerInterface(boost::shared_ptr<skiros_wm::WorldModelInterface> wm,
                      boost::shared_ptr<ros::NodeHandle> nh,
                      skiros_wm::Element robot) : wm_(wm), nh_(nh), skill_mgr_name_(robot.label())
{
    robot_ = robot;
    init();
}

void SkillManagerInterface::init()
{
    auto prefix = skill_mgr_name_.find_last_of('#');
    if(prefix!=std::string::npos)
        skill_mgr_name_ = skill_mgr_name_.substr(prefix+1, skill_mgr_name_.length());
    else
    {
        prefix = skill_mgr_name_.find_last_of(':');
        if(prefix!=std::string::npos)
            skill_mgr_name_ = skill_mgr_name_.substr(prefix+1, skill_mgr_name_.length());
    }
    if(robot_.id()<0)
        throw std::runtime_error("[SkillManagerInterface::init] Error during initialization: no valid robot found in world model. ");
    //Activate clients
    list_client_ = nh_->serviceClient<skiros_msgs::ModulesListQuery>(skill_mgr_name_ + skill_list_query_srv_name);
    skill_exe_client_ = nh_->serviceClient<skiros_msgs::ModuleCommand>(skill_mgr_name_ + skill_cmd_srv_name);
    module_exe_client_ = nh_->serviceClient<skiros_msgs::ModuleCommand>(skill_mgr_name_ + module_cmd_srv_name);
    wm_monitor_sub_ = nh_->subscribe(std::string(world_model_node_name)+wm_monitor_tpc_name, 20, &SkillManagerInterface::wmMonitorCB,  this);
    module_monitor_sub_ = nh_->subscribe(skill_mgr_name_+skill_monitor_tpc_name, 20, &SkillManagerInterface::moduleMonitorCB,  this);
    //Get modules&skill types
    module_set_ = wm_->getSubClasses(concept::Str[concept::Module]);
    skill_set_ = wm_->getSubClasses(concept::Str[concept::Skill]);
    //Get the complete skill list
    getSkillList(true);
    getModuleList(true);
}

void SkillManagerInterface::moduleMonitorCB(const skiros_msgs::ModuleStatus& msg)
{
    //Only consider monitored modules
    if(monitored_modules_.find(msg.module.name)!=monitored_modules_.end())
    {
        boost::mutex::scoped_lock lock(monitor_mux_);
        if(msg.status=="error" || msg.status=="preempted" || msg.status=="terminated")
        {
            monitored_modules_.at(msg.module.name).result.status = msg.status;
            monitored_modules_.at(msg.module.name).result.progress_code = msg.progress_code;
            monitored_modules_.at(msg.module.name).result.progress_description = msg.progress_description;
            monitored_modules_.at(msg.module.name).result.output = skiros_common::utility::deserializeParamMap(msg.module.parameters_in);
            if(monitored_modules_.at(msg.module.name).doneCb!=NULL)
                monitored_modules_.at(msg.module.name).doneCb(msg);
            monitor_cond_.notify_all();
        }
        else
        {
            if(monitored_modules_.at(msg.module.name).feedbackCb!=NULL)
                monitored_modules_.at(msg.module.name).feedbackCb(msg);
        }
    }
}

void SkillManagerInterface::wmMonitorCB(const skiros_msgs::WmMonitor& msg)
{
    //I proceed if the modification is done by the skill mgr of interest
    if(msg.author.find(skill_mgr_name_)==std::string::npos)
        return;
    //If the skill mgr adds..
    if(msg.action == "add")
    {
        if(module_set_.find(msg.element.type)!=module_set_.end())
        {
            FDEBUG("[SkillManagerInterface::wmMonitorCB] " << skill_mgr_name_ << " add " << msg.element.label);
            addModuleInList(msg2element(msg.element));
        }
        if(skill_set_.find(msg.element.type)!=skill_set_.end())
        {
            FDEBUG("[SkillManagerInterface::wmMonitorCB] " << skill_mgr_name_ << " add " << msg.element.label);
            addSkillInList(msg2element(msg.element));
        }
    }
    //If the skill mgr remove..
    if(msg.action == "remove")
    {
        if(module_set_.find(msg.element.label)!=module_set_.end())
        {
            FDEBUG("[SkillManagerInterface::wmMonitorCB] Remove " << msg.element.label);
            module_list_.erase(module_list_.find(msg.element.label));
            new_changes_ = true;
        }
        if(skill_set_.find(msg.element.label)!=skill_set_.end())
        {
            FDEBUG("[SkillManagerInterface::wmMonitorCB] Remove " << msg.element.label);
            skill_list_.erase(skill_list_.find(msg.element.label));
            new_changes_ = true;
        }
    }
}

void SkillManagerInterface::addSkillInList(skiros_wm::Element e)
{
    skiros_common::ParamMap params;
    for(skiros_common::ParamMap::value_type it : e.properties())
    {
        if(it.second.specType()!=skiros_common::symbolic)
            params.insert(it);
    }
    skill_list_[e.label()] = params;
    new_changes_ = true;
}

ModuleMapType SkillManagerInterface::getSkillList(bool update)
{
  if(update || skill_list_.empty())
  {
      std::vector<Element> v = wm_->getChildElements(robot_, relation::Str[relation::hasSkill]);
      for(Element e : v)
      {
          FDEBUG("[SkillManagerInterface]" << skill_mgr_name_ << " add " <<  e.label());
          addSkillInList(e);
      }
      if(v.size()>0)
        new_changes_ = true;
  }
  return skill_list_;
}

void SkillManagerInterface::addModuleInList(skiros_wm::Element e)
{
    skiros_common::ParamMap params;
    for(skiros_common::ParamMap::value_type it : e.properties())
    {
        if(it.second.specType()!=skiros_common::symbolic)
            params.insert(it);
    }
    module_list_[e.label()] = params;
    new_changes_ = true;
}

ModuleMapType SkillManagerInterface::getModuleList(bool update)
{
  if(update || module_list_.empty())
  {
      std::vector<Element> v = wm_->getChildElements(robot_, relation::Str[relation::hasModule]);
      for(Element e : v)
      {
          FDEBUG("[SkillManagerInterface] " << skill_mgr_name_ << " add " <<  e.label());
          addModuleInList(e);
      }
      if(v.size()>0)
        new_changes_ = true;
  }
  return module_list_;
}

ExecutionResult SkillManagerInterface::waitResult(std::string module_name)
{
    boost::mutex::scoped_lock lock(monitor_mux_);
    if(monitored_modules_.find(module_name)==monitored_modules_.end())
    {
        std::stringstream ss;
        ss << "[SkillManagerInterface::waitResult] Module " << module_name << " is not in the monitored list.";
        throw std::invalid_argument(ss.str());
    }
    while (monitored_modules_.at(module_name).result.status!="error" && monitored_modules_.at(module_name).result.status!="preempted" && monitored_modules_.at(module_name).result.status!="terminated")
        monitor_cond_.wait(lock);
    return monitored_modules_.at(module_name).result;
}

int SkillManagerInterface::exeSkill(std::string skill_name, skiros_common::ParamMap params, std::string author, DoneCallback doneCb, FeedbackCallback feedbackCb)
{
    skiros_msgs::ModuleCommand msg;
    msg.request.action = msg.request.START;
    msg.request.author = author;
    msg.request.name = skill_name;
    msg.request.s_param_map = skiros_common::utility::serializeParamMap(params);
    if(skill_exe_client_.call(msg))
    {
        if(msg.response.ok)
        {
            monitored_modules_[skill_name] = ExecutionCbs(feedbackCb, doneCb);
            return msg.response.execution_id;
        }
        else
            FERROR("[SkillManagerInterface::exeSkill] Error, can't execute the skill");
    }
    else
      FERROR("[SkillManagerInterface::exeSkill] Failed to call exe service. ");
    return -1;
}

int SkillManagerInterface::exeModule(std::string module_name, skiros_common::ParamMap params, std::string author, DoneCallback doneCb, FeedbackCallback feedbackCb)
{
    skiros_msgs::ModuleCommand msg;
    msg.request.action = msg.request.START;
    msg.request.author = author;
    msg.request.name = module_name;
    msg.request.s_param_map = skiros_common::utility::serializeParamMap(params);
    if(module_exe_client_.call(msg))
    {
        if(msg.response.ok)
        {
            monitored_modules_[module_name] = ExecutionCbs(feedbackCb, doneCb);
            return msg.response.execution_id;
        }
        else
            FERROR("[SkillManagerInterface::exeModule] Error, can't execute the module");

    }
    else
      FERROR("[SkillManagerInterface::exeModule] Failed to call exe service. ");
    return -1;
}

bool SkillManagerInterface::stopSkill(int execution_id, std::string skill_name, std::string author)
{
    skiros_msgs::ModuleCommand msg;
    msg.request.action = msg.request.PREEMPT;
    msg.request.author = author;
    msg.request.name = skill_name;
    msg.request.execution_id = execution_id;
    if(skill_exe_client_.call(msg))
    {
        return msg.response.ok;
    }
    else
    {
      FERROR("[SkillManagerInterface::exeSkill] Failed to call exe service. ");
      return -1;
    }
}

bool SkillManagerInterface::stopModule(int execution_id, std::string module_name, std::string author)
{
    skiros_msgs::ModuleCommand msg;
    msg.request.action = msg.request.PREEMPT;
    msg.request.author = author;
    msg.request.name = module_name;
    msg.request.execution_id = execution_id;
    if(module_exe_client_.call(msg))
    {
        return msg.response.ok;
    }
    else
    {
      FERROR("[SkillManagerInterface::exeSkill] Failed to call exe service. ");
      return false;
    }
}

skiros_common::ParamMap SkillManagerInterface::getParams(std::string skill_name)
{
  //Look for a skill
  ModuleMapType::iterator it = skill_list_.find(skill_name);
  if(it != skill_list_.end()) return it->second;
  //If wasn't found, I try with modules
  it = module_list_.find(skill_name);
  if(it != module_list_.end()) return it->second;
  //If not found, I throw an error
  std::stringstream ss;
  ss << "Module or skill named " << skill_name << " not present at " << skill_mgr_name_ << ".";
  throw std::runtime_error(ss.str());
}

int SkillManagerInterface::print(std::string indend)
{
    std::map<std::string, skiros_common::ParamMap>::iterator it;
    int i = 0;
    std::cout << "Skills: " << std::endl;
    for(it = skill_list_.begin();it != skill_list_.end();it++)
    {
      std::cout << indend << i++ << " " << it->first << "" << std::endl;
    }
    std::cout << "Modules: " << std::endl;
    for(it = module_list_.begin();it != module_list_.end();it++)
    {
      std::cout << indend << i++ << " " << it->first << "" << std::endl;
    }
    return i;
}

//UI method (TODO: this function has to be removed or moved)
std::string SkillManagerInterface::selectSkill()
{
  int i = 1;int num_input = -1;bool go_forward = false;std::string selected_skill;
  std::map<std::string, skiros_common::ParamMap>::iterator it;
  std::vector<std::string> temp;
  if(skill_list_.size())
  {
    while(!go_forward)
    {
            i = 0;temp.clear();
            std::cout << "Choose from the list the skill to load (number)." << std::endl;
            for (it = skill_list_.begin();it != skill_list_.end();++it)
            {
                  std::cout << i++ << " " << it->first.c_str() << std::endl;
                  temp.push_back(it->first);
            }
            std::cin >> num_input;
            if(num_input >= 0 && num_input<skill_list_.size())
            {
                    selected_skill = temp[num_input];
                    std::cout << selected_skill.c_str() << std::endl;
                    go_forward = true;
            }
            else
            {
                std::cout << "Number " << num_input << " is not valid. Try again"  << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
    }
  }
  return selected_skill;
}

//UI method (TODO: this function has to be removed or moved)
std::string SkillManagerInterface::selectModule()
{
  int i = 1;int num_input = -1;bool go_forward = false;std::string selected_skill;
  std::map<std::string, skiros_common::ParamMap>::iterator it;
  std::vector<std::string> temp;
  if(module_list_.size())
  {
    while(!go_forward)
    {
            i = 0;temp.clear();
            std::cout << "Choose a module from the list (number)." << std::endl;
            for (it = module_list_.begin();it != module_list_.end();++it)
            {
                  std::cout << i++ << " " << it->first.c_str() << std::endl;
                  temp.push_back(it->first);
            }
            std::cin >> num_input;
            if(num_input >= 0 && num_input<module_list_.size())
            {
                    selected_skill = temp[num_input];
                    std::cout << selected_skill.c_str() << std::endl;
                    go_forward = true;
            }
            else
            {
                std::cout << "Number " << num_input << " is not valid. Try again"  << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
    }
  }
  return selected_skill;
}

}
