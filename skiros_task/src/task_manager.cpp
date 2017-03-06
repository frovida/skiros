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
#include <tinyxml.h>
#include <ros/ros.h>
#include "skiros_common/utility.h"
#include "skiros_task/task_manager.h"
#include "skiros_config/declared_uri.h"
#include "boost/foreach.hpp"
#include "skiros_config/node_names.h"
#include <ctime>
#include <skiros_world_model/utility.h>
#include <skiros_msgs/TmMonitor.h>
#include <skiros_common/utility.h>

using namespace skiros_config::owl;
using namespace skiros_config;
using namespace skiros_skill;
using namespace skiros_wm;


namespace skiros_task
{

//----- Ros interface ----------------
void TaskManager::taskExeCb(const skiros_msgs::TmTaskExe& msg)
{
    if(msg.exe)
    {
        startTask(msg.iterate);
    }
    else
    {
        stopTask();
    }
}

bool TaskManager::startTask(bool iterate)
{
    if(!running_)
        exe_thread_ = boost::thread(boost::bind(&TaskManager::exeTaskThread, this, iterate));
    else
        return false;
    return true;
}

bool TaskManager::taskQuery(skiros_msgs::TmQueryTaskRequest &req, skiros_msgs::TmQueryTaskResponse &res)
{
    for(SkillHolder skill : task_)
    {
        res.task_robots.push_back(skill.manager);
        skiros_msgs::ModuleDescription msg;
        msg.name = skill.name;
        msg.parameters_in = skiros_common::utility::serializeParamMap(skill.params);
        res.task_skills.push_back(msg);
    }
    return true;
}

bool TaskManager::taskModify(skiros_msgs::TmModifyTaskRequest &req, skiros_msgs::TmModifyTaskResponse &res)
{
    startModifingTask();
    if(req.action==req.ADD)
    {
        SkillHolder skill;
        skill.manager = req.robot;
        skill.name = req.skill.name;
        skill.params = skiros_common::utility::deserializeParamMap(req.skill.parameters_in);
        res.return_code = insertSkillInSequence(req.author, skill, req.index);
    }
    else if(req.action==req.REMOVE)
    {
        res.return_code = removeSkillInSequence(req.author, req.index);
    }
    else if(req.action==req.UPDATE)
    {
        //TODO
    }
    endModifingTask();
    FINFO("Current task: \n" << printSkillSequence());
    return true;
}

TaskManager::TaskManager(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<WorldModelInterface> wm) : nh_ptr_(nh), wm_ptr_(wm), running_(false)
{
    tm_monitor_pub_ = nh_ptr_->advertise<skiros_msgs::TmMonitor>(ros::this_node::getName()+task_monitor_tpc_name, 10);
    task_exe_sub_ = nh_ptr_->subscribe(ros::this_node::getName()+task_exe_tpc_name,10, &TaskManager::taskExeCb, this);
    endModifingTask();
}

ExecutionResult TaskManager::exeSkill(SkillHolder skill)
{
  boost::mutex::scoped_lock lock(exe_mux_);
  //Check the presence of the skill_mgr_name, then call the appropiate function
  SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().find(skill.manager);
  if(it != skill_layer_.getSkillMgrsMap().end())
  {
      for(auto & pair : skill.params)
      {
          skiros_common::Param & p = pair.second;
          if(p.type()==typeid(Element))
          {
              Element e = p.getValue<Element>();
              if(e.id()>=0)
              {
                  e = getWorldHandle()->getElement(e.id());
                  p.setValue(e);
              }
          }
      }
      if(it->second->exeSkill(skill.name, skill.params, controller_id_)<0)
          return ExecutionResult();
      lock.unlock();
      return it->second->waitResult(skill.name);
  }
  else return ExecutionResult();
}

void TaskManager::stopSkill()
{
    if(running_)
    {
        boost::mutex::scoped_lock lock(exe_mux_);
        SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().find(current_exe_skill_.manager);
        if(it != skill_layer_.getSkillMgrsMap().end())
        {
            it->second->stopSkill(0, current_exe_skill_.name, controller_id_);
        }
    }
}

void TaskManager::clearSkillSequence(std::string author)
{
    task_.clear();
    advertiseTaskModification(author);
}

bool TaskManager::insertSkillInSequence(std::string author, SkillHolder skill, int index)
{
    SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().find(skill.manager);
    if(it==skill_layer_.getSkillMgrsMap().end())
    {
        FERROR("[insertSkillInSequence] The robot " << skill.manager << " doesn't exist. Can't insert skill " << skill.name);
        return false;
    }
    ModuleMapType skill_map = it->second->getSkillList();
    ModuleMapType::iterator it_skill = skill_map.find(skill.name);
    if(it_skill==skill_map.end())
    {
        FERROR("[insertSkillInSequence] The skill " << skill.name << " is not a skill of " << skill.manager);
        return false;
    }
    if(index<0)
        task_.push_back(skill);
    else
    {
        TaskType::iterator it = task_.begin();
        for(int i=0;i<=index;i++)
            it++;
        task_.insert(it, skill);
    }
    advertiseTaskModification(author);
    return true;
}

bool TaskManager::removeSkillInSequence(std::string author, int index)
{
    if(!task_.size())return false;
    if(index<0)
        task_.pop_back();
    else
    {
        TaskType::iterator it = task_.begin();
        for(int i=0;i<index;i++)
        {
            if(it==task_.end())return false;
            it++;
        }
        task_.erase(it);
    }
    advertiseTaskModification(author);
    return true;
}

std::string TaskManager::getSavePath()
{
  if(save_path_.empty()) save_path_ = skiros_common::utility::getSkirosSaveDirectory() + "tasks/";
  return save_path_;
}

void TaskManager::exeTaskThread(bool iterate)
{
    do
    {
        if(!exeSkillSequence())break;
    }
    while(iterate);
}


void TaskManager::shutdown()
{
    stopSkill();
    skill_layer_.shutdown();
}

//-------------- UI METHODS (Should be discriminated soon) ----------------------

SkillHolder TaskManager::select(std::string what)
{
  SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().begin();
  SkillHolder to_ret;
  if(skill_layer_.getSkillMgrsMap().size() == 1) to_ret.manager = it->first;
  else
  {
      int i = 1;int num_input = 1;bool go_forward = false;std::string selected_skill_mgr;
      std::vector<std::string> temp;
      while(!go_forward)
      {
              i = 0;temp.clear();
              std::cout << "Choose from the list the robot to use (number). Type 0 to close the program." << std::endl;
              for (it = skill_layer_.getSkillMgrsMap().begin();it != skill_layer_.getSkillMgrsMap().end();++it)
              {
                    std::cout << i++ << " " << it->first.c_str() << std::endl;
                    temp.push_back(it->first);
              }
              std::cin >> num_input;
              if(num_input >= 0 && num_input<skill_layer_.getSkillMgrsMap().size())
              {
                      selected_skill_mgr = temp[num_input];
                      //std::cout << selected_skill_mgr.c_str() << std::endl;
                      go_forward = true;
              }
              else
              {
                  if(num_input==0) return to_ret;
                  std::cout << "Number " << num_input << " is not valid. Type again"  << std::endl;
                  std::cin.clear();
                  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
              }
      }
      to_ret.manager = selected_skill_mgr;
      it = skill_layer_.getSkillMgrsMap().find(to_ret.manager);
  }
  if(what=="skill") to_ret.name = it->second->selectSkill();
  if(what=="module") to_ret.name = it->second->selectModule();
  return to_ret;
}

//TODO: discriminate between online params and offline params and create ways to set precisely every parameter
bool TaskManager::setParams(SkillHolder & skill)
{
  //Check the presence of the skill_mgr_name
  SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().find(skill.manager);
  if(it != skill_layer_.getSkillMgrsMap().end())
  {
     try
     {
          skill.params = it->second->getParams(skill.name);
     }
     catch(std::runtime_error e)
     {return false;}
     skiros_common::ParamMap::iterator itt;
     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
     for (itt = skill.params.begin();itt != skill.params.end();++itt)
     {
         if(itt->second.specType() == skiros_common::offline)
         {
             //If there is any problem in the teaching I abort the procedure
             if(!this->teachParam(itt->second, it->second->getRobotElement())) return false;
         }
         if(itt->second.specType() == skiros_common::online ||
                 itt->second.specType() == skiros_common::hardware ||
                 itt->second.specType() == skiros_common::optional)
         {
             //If there is any problem in the teaching I abort the procedure
             if(!this->teachParam(itt->second, it->second->getRobotElement())) return false;
         }
     }
     return true;
  }
  else return false;
}

void TaskManager::printAvailableMgrs(std::string indend)
{
    for(auto r : skill_layer_.getSkillMgrsMap())
    {
      std::cout << indend << r.first << ":" << std::endl;
      r.second->print("- ");
    }
}

bool TaskManager::insertSkillInSequence()
{
  SkillHolder skill = this->select("skill");
  if(skill.name != "")
  {
      if(!this->setParams(skill))
      {
          FERROR("Skill insertion aborted.");
          return false;
      }
      else
      {
          std::cout << "Skill parametrized correctly." << std::endl;
      }
  }
  else
  {
      FERROR("No valid skill selected. Aborted.");
      return false;
  }
  insertSkillInSequence(controller_id_, skill);
  return true;
}

bool TaskManager::exeModule()
{
  SkillHolder module = this->select("module");
  if(module.name != "")
  {
      if(!this->setParams(module))
      {
          FERROR("Module not parametrized correctly.");
          return false;
      }
  }
  else
  {
      FERROR("No valid module selected. Aborted.");
      return false;
  }
  //Check the presence of the skill_mgr_name, then call the appropiate function
  SkillMgrsMap::iterator it = skill_layer_.getSkillMgrsMap().find(module.manager);
  if(it != skill_layer_.getSkillMgrsMap().end())
  {
      if(it->second->exeModule(module.name, module.params, controller_id_)<0)
          return false;
      ExecutionResult result = it->second->waitResult(module.name);
      if(result.progress_code>0)return true;
      else return false;
  }
  else return false;
}

std::string TaskManager::printSkillSequence()
{
  std::stringstream ss;
  for(std::list<SkillHolder>::iterator it = task_.begin(); it!=task_.end(); ++it)
  {
      ss << it->manager << ": " << it->name << "." << std::endl;
  }
  return ss.str();
}

bool TaskManager::exeSkillSequence()
{
    running_ = true;
    FINFO("[TaskManager] Start task");
    ros::Time start_time = ros::Time::now();
    std::string save_path = skiros_common::utility::getSkirosSaveDirectory() + "skill_exe_log.txt";
    FILE * f = fopen(save_path.c_str(), "a");
    std::stringstream ss;
    if(!f) FERROR("[TaskManager::exeSkillSequence] Failed to open log file.");
    for(SkillHolder skill : task_)
    {
        current_exe_skill_ = skill;
        start_time = ros::Time::now();
        publishTaskStatus(controller_id_, skill, "started");
        ExecutionResult result = this->exeSkill(skill);
        publishTaskStatus(controller_id_, skill, result.status, result.progress_code, result.progress_description);

        time_t current_time = time(NULL);
        struct tm * now = localtime( & current_time);
        std::stringstream time_date_stamp;
        time_date_stamp << (now->tm_year+1900) << '-'
                << (now->tm_mon +1) << '-'
                << (now->tm_mday) << '_'
                << now->tm_hour << ':'
                << now->tm_min << ':'
                << now->tm_sec;

        ss << time_date_stamp.str() << " " << skill.manager << " "
           << skill.name << " " <<  result.status << " "
           << result.progress_code  << " "
           << "'" << result.progress_description << "'"
           << " " << (ros::Time::now()-start_time).toSec() << "\n" ;

        if(f)
            fputs(ss.str().c_str(), f);
        if(result.status!="terminated")
        {
            FERROR("Error while executing skill: " << skill.name << ".");
            fclose(f);
            running_ = false;
            return false;
        }
        else FINFO(skill.name << " skill finished with code " << result.progress_code << " in " << (ros::Time::now()-start_time).toSec() << " second(s).");
    }
    fclose(f);
    running_ = false;
    return true;
}

void TaskManager::publishTaskStatus(std::string author, SkillHolder skill, std::string status, int progress_code, std::string progress_description)
{
    skiros_msgs::TmMonitor msg;
    msg.header.stamp = ros::Time::now();
    msg.author = author;
    msg.robot = skill.manager;
    msg.action = status;
    msg.skill.name = skill.name;
    msg.skill.parameters_in = skiros_common::utility::serializeParamMap(skill.params);
    msg.progress_code = progress_code;
    msg.progress_description = progress_description;
    tm_monitor_pub_.publish(msg);
}

void TaskManager::advertiseActivity(std::string author, std::string activity, int code, std::string message, ros::Duration duration)
{
    skiros_msgs::TmMonitor msg;
    msg.header.stamp = ros::Time::now();
    msg.author = author;
    msg.action = activity;
    msg.progress_code = code;
    msg.progress_description = message;
    msg.progress_seconds = duration.toSec();
    tm_monitor_pub_.publish(msg);
}

void TaskManager::advertiseTaskModification(std::string author)
{
    skiros_msgs::TmMonitor msg;
    msg.header.stamp = ros::Time::now();
    msg.author = author;
    msg.action = "modified";
    tm_monitor_pub_.publish(msg);
}

//! Allows user to set an online parameter
bool TaskManager::teachParam(skiros_common::Param &p, skiros_wm::Element robot)
{
    if(p.type() == typeid(skiros_wm::Element)) //A world model element
    {
        std::vector<skiros_wm::Element> v;
        if(p.state() == skiros_common::specified)
        {
            skiros_wm::Element temp = p.getValue<skiros_wm::Element>();
            if(p.specType()==skiros_common::hardware)
            {
                std::vector<skiros_wm::Element> tempv = wm_ptr_->getBranchElements(robot, relation::Str[relation::hasA], "");
                tempv.push_back(robot);
                BOOST_FOREACH(skiros_wm::Element e, tempv)
                {
                    if(e.type()==temp.type())
                        v.push_back(e);
                }
            }
            else
                v = wm_ptr_->resolveElement(temp);
        }
        else
        {
              v = wm_ptr_->resolveElement(skiros_wm::Element(concept::Str[concept::Unknown]));
        }
        bool ok = false;
        int num_input = -1;
        std::cout << p.name() << ": " << std::endl;
        if(p.specType()==skiros_common::optional)
            v.push_back(skiros_wm::Element("No input"));
        if(v.size())
        {
            char str[40];
            for(int i=0; i<v.size();i++)
            {
                std::stringstream ss(v[i].printState());
                ss.getline(str, 40);
                std::cout << "  " << i << ": " << str << std::endl;
            }
            //If I have only one element, I straightforward select that
            if(v.size()!=1)
            {
              while(!ok)
              {
                  std::cout << "Select from the list the object you would like to pass to the skill" << std::endl;
                  std::cin >> num_input;
                  if(num_input>=0 && num_input<v.size())
                  {
                      ok = true;
                      std::cin.clear();
                      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                  }
                  else
                  {
                      std::cout << "Not a valid input. Try again";
                      std::cin.clear();
                      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                  }
              }
            }
            else num_input = 0;
            if(p.specType()!=skiros_common::optional || v[num_input].type()!="No input")
                p.setValue(v[num_input]);
        }
        else
        {
            FERROR("No valid objects to fill the skill");
            return false;
        }
    }
    else //A standard parameter
    {
        //Print the description and default value
        std::cout << p.name() << " (type: " <<  p.type().name() << ", default value: ";
        std::vector<std::string> v;
        p.getValuesStr(v);
        BOOST_FOREACH(std::string str, v)
        {
            std::cout << str << " ";
        }
        std::cout << ") ";
        if(p.specType()==skiros_common::optional)
            std::cout << "(Optional)";
        std::cout <<  std::endl;
        //std::cin.clear();
        //Get input
        for(int i=0;i<v.size();i++)
        {
            //Ask the input
            std::string input;
            std::getline(std::cin, input);
            if(input.size()>0) p.setValue(input, i);
        }
        //Print out the assigned value
        std::cout << "Value set to: ";
        p.getValuesStr(v);
        BOOST_FOREACH(std::string str, v)
        {
            std::cout << str << " ";
        }
        std::cout << std::endl;
    }
    return true;
}


} //namespace skiros_task
