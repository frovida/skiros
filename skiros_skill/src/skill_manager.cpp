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

//ELEMENT_POLLUTION_TAG (in this file there is at least 1 element got by type
#include "skiros_skill/skill_manager.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/condition.h"
#include "skiros_config/declared_uri.h"
#include "skiros_config/node_names.h"
#include "boost/foreach.hpp"
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

#include <skiros_world_model/reasoners_loading_func.h>

using namespace skiros_config::owl;
using namespace skiros_config;
using namespace skiros;
using namespace skiros::state;
using namespace skiros_wm;


namespace skiros_skill
{

SkillManager::SkillManager(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model, std::string prefix, std::string robot_uri)
: nh_(nh), wm_(world_model), robot_name_(robot_uri), robot_uri_(prefix+robot_uri)
{
        //Initialize the plugin loader
        skills_loader_ =
            new pluginlib::ClassLoader<skiros_skill::SkillBase>("skiros_skill", "skiros_skill::SkillBase");
        module_loader_ =
            new pluginlib::ClassLoader<skiros::ModuleBase>("skiros_skill", "skiros::ModuleBase");
        //Initialize the interface to the resource manager
        rm_ = new skiros_common::ResourceManagerInterface(*nh_, robot_name_);
        rm_monitor_ = nh_->subscribe(robot_name_+"_rm/monitor", 5, &SkillManager::RmMonitorCB, this);
        wm_monitor_ = nh_->subscribe(std::string(world_model_node_name)+wm_monitor_tpc_name, 5, &SkillManager::wmMonitorCB, this);
        state_pub_ = nh_->advertise<skiros_msgs::ModuleStatus>(robot_name_+skill_monitor_tpc_name, 20);
        //rm_->waitConnection();
}

SkillManager::~SkillManager()
{
}

void SkillManager::init()
{
    module_monitors_.clear();
    loaded_modules_.clear();
    loaded_skills_.clear();
    mi_.reset(new SkillManagerInterface(wm_, nh_, wm_->getRobot()));
    ros::Time start_time;
    double total_time = 0;
    // Load shared libraries names
    ros::NodeHandle nh("~");
    std::vector<std::string> declared_skills = skills_loader_->getDeclaredClasses();
    std::vector<std::string> declared_modules = module_loader_->getDeclaredClasses();
    if(nh.hasParam("module_list"))
    {
        std::string temp;
        nh.getParam("module_list", temp);
        if(temp!="")
        {
            std::stringstream ss(temp);
            declared_modules.clear();
            while(!ss.eof())
            {
                ss >> temp;
                if(temp!="nomodules")declared_modules.push_back(temp);
            }
        }
    }
    if(nh.hasParam("skill_list"))
    {
        std::string temp;
        nh.getParam("skill_list", temp);
        if(temp!="")
        {
            std::stringstream ss(temp);
            declared_skills.clear();
            while(!ss.eof())
            {
                ss >> temp;
                if(temp!="noskills") declared_skills.push_back(temp);
            }
        }
    }
    //Istanciate and initialize all declared modules
    for(std::string module : declared_modules)
    {
        start_time = ros::Time::now();
        try
        {
            loaded_modules_[module] = module_loader_->createInstance(module);
            configureModule(module);
            boost::shared_ptr<skiros::ModuleMonitor> monitor(new skiros::ModuleMonitor(loaded_modules_.at(module), module, robot_name_+skill_monitor_tpc_name, false));
            module_monitors_.insert(ModuleMonitorsPairType(module, monitor));
            loaded_modules_[module]->init(nh_, wm_, mi_);
            if(loaded_modules_[module]->isInitialized())
            {
                FINFO("[SkillManager::init] Module " << module << " loaded in " << (ros::Time::now() - start_time).toSec() << " second(s)." );
            }
            else
            {
                FWARN("[SkillManager::init] Module " << module << " failed to initialize and wasn't added to robot available modules." );
                module_monitors_.erase(module);
                loaded_modules_.erase(module);
            }
        }
        catch(pluginlib::PluginlibException& ex)
        {
            FERROR("[SkillManager::init]The plugin " << module << " failed to load. Error: " << ex.what());
            loaded_modules_.erase(module);
        }
        catch(...)
        {
            FERROR("[SkillManager::init]The plugin " << module << " failed to load for an unhandled exception.");
            module_monitors_.erase(module);
            loaded_modules_.erase(module);
        }
        total_time += (ros::Time::now() - start_time).toSec();
    }
    //Istanciate and initialize all declared skills
    for(std::string skill : declared_skills)
    {
        start_time = ros::Time::now();
        try
        {
            loaded_skills_[skill] = skills_loader_->createInstance(skill);
            boost::shared_ptr<skiros::ModuleMonitor> monitor(new skiros::ModuleMonitor(loaded_skills_.at(skill), skill, robot_name_+skill_monitor_tpc_name, true));
            module_monitors_.insert(ModuleMonitorsPairType(skill, monitor));
            loaded_skills_[skill]->init(nh_, wm_, mi_); //TODO: think if it is good to init here
            if(loaded_skills_[skill]->isInitialized())
            {
                FINFO("[SkillManager::init] Skill " << skill << " loaded in " <<  (ros::Time::now() - start_time).toSec() << " second(s)." );
            }
            else
            {
                FWARN("[SkillManager::init] Skill " << skill << " failed to initialize and wasn't added to robot available skills."  );
                module_monitors_.erase(skill);
                loaded_skills_.erase(skill);
            }
        }
        catch(pluginlib::PluginlibException& ex)
        {
            FERROR("[SkillManager::init]The plugin " << skill << " failed to load. Error: " << ex.what());
            loaded_skills_.erase(skill);
        }
        catch(...)
        {
            FERROR("[SkillManager::init]The plugin " << skill << " failed to load for an unhandled exception.");
            module_monitors_.erase(skill);
            loaded_skills_.erase(skill);
        }
        total_time += (ros::Time::now() - start_time).toSec();
    }
    FINFO("[SkillManager::init] Initialized in " << total_time << " second(s).");
    //Sleep to ensure all ROS connection are enstablished
    //sleep(0.5);
}

//---------- ROS service functions ------------------

bool SkillManager::moduleListQuery(skiros_msgs::ModulesListQueryRequest &req, skiros_msgs::ModulesListQueryResponse &res)
{
    for(auto m : loaded_modules_)
    {
        if(!m.second->isInitialized()) continue;
        skiros_msgs::ModuleDescription module;
        module.name = m.first;
        //Extract the parameter map from the module
        skiros_common::ParamHandler ph = loaded_modules_[m.first]->getParamHandlerCopy();
        skiros_common::ParamMap map = ph.getParamMap();
        module.parameters_in = skiros_common::utility::serializeParamMap(map);
        //Extract the parameter map from the module
        ph = loaded_modules_[m.first]->getResultParamHandlerCopy();
        map = ph.getParamMap();
        module.parameters_out = skiros_common::utility::serializeParamMap(map);
        res.modules_list.push_back(module);
    }
    return true;
}

bool SkillManager::skillListQuery(skiros_msgs::ModulesListQueryRequest &req, skiros_msgs::ModulesListQueryResponse &res)
{
    for(auto skill : loaded_skills_)
    {
        if(skill.second->isInitialized()) continue;
        skiros_msgs::ModuleDescription module;
        module.name = skill.first;
        //Extract the parameter map from the skill
        skiros_common::ParamHandler ph = loaded_skills_[skill.first]->getParamHandlerCopy();
        skiros_common::ParamMap map = ph.getParamMap();
        module.parameters_in = skiros_common::utility::serializeParamMap(map);
        res.modules_list.push_back(module);
    }
    return true;
}

bool SkillManager::skillCommandService(skiros_msgs::ModuleCommandRequest &req, skiros_msgs::ModuleCommandResponse &res)
{
    //Check the module exists
    if(loaded_skills_.find(req.name)==loaded_skills_.end())
    {
        FERROR("[skillCommandService] Skill " << req.name.c_str() << " doesn't exist.");
        res.ok = false;
        return true;
    }

    boost::shared_ptr<skiros_skill::SkillBase> skill_ptr = loaded_skills_[req.name.c_str()];

    if(req.action==req.START)
    {
        //Check the module isn't already running
        if(running_skills_.find(req.name.c_str())!=running_skills_.end())
        {
            if(skill_ptr->isRunning())
            {
                FERROR("[skillCommandService] The skill " << req.name.c_str() << " is already running, can't run until it finish.");
                res.ok = false;
                return true;
            }
        }
        //Set the parameter map of the skill
        skiros_common::ParamMap param_map = skiros_common::utility::deserializeParamMap(req.s_param_map);
        if(!skill_ptr->setParams(param_map))
        {
            FERROR("Not all parameters have been specified.");
            res.ok = false;
            return true;
        }
        running_skills_[req.name.c_str()].reset(new boost::thread(boost::bind(&SkillManager::moduleStart, this, skill_ptr, req.author)));
        res.execution_id = 0;//TODO: 0 is enough as long as I have only one running istance for module
    }
    else if(req.action == req.PREEMPT)
    {
        skill_ptr->preempt("Preempted from " + req.author);
        skillKill(req.name.c_str());
    }
    else if(req.action==req.PAUSE)
    {
        //skill_ptr->pause();
    }
    else if(req.action==req.KILL)
    {
        //Kill the module thread
        skillKill(req.name.c_str()); //TODO: the module key should change from the name to an execution_id
    }
    res.ok = true;
    return true;
}

bool SkillManager::moduleCommandService(skiros_msgs::ModuleCommandRequest &req, skiros_msgs::ModuleCommandResponse &res)
{
    //Check the module exists
    if(loaded_modules_.find(req.name)==loaded_modules_.end())
    {
        FERROR("[moduleExe] Module " << req.name.c_str() << " doesn't exist.");
        res.ok = false;
        return true;
    }

    boost::shared_ptr<skiros::ModuleBase> module_ptr = loaded_modules_[req.name.c_str()];

    if(req.action==req.START)
    {
        //Check the module isn't already running
        if(running_modules_.find(req.name.c_str())!=running_modules_.end())
        {
            if(module_ptr->isRunning())
            {
                FERROR("[moduleExe] The module " << req.name.c_str() << " is already running, can't run until it finish.");
                res.ok = false;
                return true;
            }
        }
        //Set the parameter map
        skiros_common::ParamMap param_map = skiros_common::utility::deserializeParamMap(req.s_param_map);
        if(!module_ptr->setParams(param_map))
        {
            FERROR("[SkillManager::moduleExe] Not all parameters have been specified.");
            res.ok = false;
            return true;
        }
        running_modules_[req.name.c_str()].reset(new boost::thread(boost::bind(&SkillManager::moduleStart, this, module_ptr, req.author)));
        res.execution_id = 0;//TODO: 0 is enough as long as I have only one running istance for module
    }
    else if(req.action == req.PREEMPT)
    {
        module_ptr->preempt("Preempted from " + req.author);
        moduleKill(req.name.c_str()); //TODO: remove from here when i'm ready...
    }
    else if(req.action==req.PAUSE)
    {
        //module_ptr->pause();
    }
    else if(req.action==req.KILL)
    {
        //Kill the module thread
        moduleKill(req.name.c_str()); //TODO: the module key should change from the name to an execution_id
    }
    res.ok = true;
    return true;
}

//---------- Thread functions ------------------

void SkillManager::moduleStart(boost::shared_ptr<skiros::ModuleCore> module, std::string controller)
{
    try
    {
        module->start(controller);
    }
    catch(std::runtime_error e)
    {
        std::stringstream ss;
        ss << "Execution failed: " << e.what();
        FERROR(ss.str());
        module->preempt(ss.str());
    }
    catch(std::invalid_argument e)
    {
        std::stringstream ss;
        ss << "Execution failed: " << e.what();
        FERROR(ss.str());
        module->preempt(ss.str());
    }

}

void SkillManager::preemptAll()
{
    boost::mutex::scoped_lock lock(module_kill_mutex_);
    //Stop all running modules
    for(RunningModulesMapType::value_type pair : running_modules_)
    {
        loaded_modules_[pair.first]->preempt("Preempted from user.");
    }

    for(RunningSkillsMapType::value_type pair : running_skills_)
    {
        loaded_skills_[pair.first]->preempt("Preempted from user.");
    }
    lock.unlock();
}

void SkillManager::skillKill(std::string skill)
{
    boost::mutex::scoped_lock lock(module_kill_mutex_);
    void *dont_care;
    //TODO: wait for change of state (with timeout)
    RunningSkillsMapType::iterator it = running_skills_.find(skill);
    if(it==running_skills_.end())
        return;
    pthread_cancel(it->second->native_handle());
    pthread_join(it->second->native_handle(), &dont_care);
    running_skills_.erase(it);
}

void SkillManager::moduleKill(std::string module)
{
    boost::mutex::scoped_lock lock(module_kill_mutex_);
    void *dont_care;
    //TODO: wait for change of state (with timeout)
    RunningModulesMapType::iterator it = running_modules_.find(module);
    if(it==running_modules_.end())
        return;
    pthread_cancel(it->second->native_handle());
    pthread_join(it->second->native_handle(), &dont_care);
    running_modules_.erase(it);
}

//---------- Other functions ------------------

void SkillManager::wmMonitorCB(const skiros_msgs::WmMonitor& msg)
{
    if(msg.action=="loadScene")
        reload(false);
}

void SkillManager::RmMonitorCB(const skiros_msgs::RmMonitor& msg)
{
    boost::mutex::scoped_lock lock(devices_mutex_);
    if(msg.event_type == skiros_msgs::RmMonitor::TERMINATED)
    {
        FWARN("[SkillManager::RmMonitorCB] Driver (address: " << msg.resource_address << ") terminated.");
        DriverDeviceMap::iterator it = driver_device_map_.find(msg.resource_address);
        wm_->removeElement(it->second);
        driver_device_map_.erase(it);
    }
}

bool SkillManager::registerRobot()
{
  FINFO("Registering robot on world model...");
  skiros_wm::Element location, robot;
  std::vector<skiros_wm::Element> devices;
  try
  {
      robot = wm_->getDefaultElement(robot_uri_);
      //--------------------------------------------------
      //Check if the robot location is already registered in the world model
      if(robot.hasProperty(relation::Str[relation::hasStartLocation]))
      {
          std::string start_location=robot.properties(relation::Str[relation::hasStartLocation]).getValue<std::string>();
          location = wm_->getDefaultElement(start_location);
          std::vector<skiros_wm::Element> locations = wm_->resolveElement(location);
          for(int i=0;i<locations.size();i++)
          {
              if(locations[i].label() == start_location) location = locations[i];
          }
          //If I didn't found the location I create a new one
          if(location.id() <= 0)
          {
            location = wm_->getDefaultElement(start_location);
          }
      }
      else location = wm_->getElement(0);
      //---------------------------------------------------

      //Overwriting the skill manager info with the node name
      if(!robot.hasProperty(data::Str[data::SkillMgr])) robot.addPropertyString(data::Str[data::SkillMgr], "");
      robot.properties(data::Str[data::SkillMgr]).setValue(ros::this_node::getName());
      wm_->addTfPrefix(robot);
      //Call the registration routine
      wm_->lock();
      if(!(wm_->registerRobot(location, robot)>0)) {
          wm_->unlock();
          return false;
      }
      reload();
      wm_->unlock();

      ///Start a timer to update the world model with relevant hardware's tf modifications
      skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
      if(reasoner)
      {
        tfpub_timer_ = nh_->createTimer(ros::Duration(0.2),&SkillManager::tfPubTimerCB, this);
      }
      else
      {
          FWARN("[SkillManager::registerRobot] The AauSpatialReasoner plugin wasn't found. The robot hardware tf will not get updated");
      }
      return true;
  }
  catch(std::runtime_error err)
  {
      FWARN("[SkillManager::registerRobot] " << err.what());
      return false;
  }
}

void SkillManager::tfPubTimerCB(const ros::TimerEvent& e)
{
    ros::Time now = ros::Time::now();
    tf::StampedTransform transform;
    skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner("AauSpatialReasoner");
    Element robot = wm_->getRobot();
    try
    {
        if(robot.hasProperty("LinkedToFrameId"))
        {
            std::string frame_id = robot.properties("LinkedToFrameId").getValue<std::string>();
            std::string parent_frame_id = wm_->getFirstParentFrameId(robot);
            if(wm_->waitForTransform(parent_frame_id, frame_id, now, ros::Duration(1.0)))
            {
                wm_->lookupTransform(parent_frame_id, frame_id, now, transform);
                //Compare to the old transform (if it is the same doesn't republish)
                bool publish = true;
                try
                {
                    tf::Transform old_t = reasoner->getData<tf::Transform>(robot, "Transform");
                    if(old_t.getOrigin().distance(transform.getOrigin())<=0.001 &&
                       tf::angleShortestPath(old_t.getRotation(), transform.getRotation())/M_PI<=0.001  ) //HARDCODE!
                    {
                        publish = false;
                    }
                }
                catch(...){}
                if(publish)
                {
                    robot = wm_->getElement(robot.id());
                    reasoner->storeData(robot, transform.getOrigin(), "Position");
                    reasoner->storeData(robot, transform.getRotation(), "Orientation");
                    wm_->updateElement(robot);
                    while(!robot.hasProperty(data::Str[data::FrameId]))
                        robot = wm_->getElement(robot.id());
                }
            }
        }
        boost::mutex::scoped_lock lock(devices_mutex_);
        for(Element & e : wm_devices_)
        {
            if(e.hasProperty("LinkedToFrameId"))
            {
                now = ros::Time::now();
                std::string frame_id = e.properties("LinkedToFrameId").getValue<std::string>();
                std::string parent_frame_id = wm_->getFirstParentFrameId(e);
                if(wm_->waitForTransform(parent_frame_id, frame_id, now, ros::Duration(1.0)))
                {
                    wm_->lookupTransform(parent_frame_id, frame_id, now, transform);
                    //Compare to the old transform (if it is the same doesn't republish)
                    bool publish = true;
                    try
                    {
                        tf::Transform old_t = reasoner->getData<tf::Transform>(e, "Transform");
                        if(old_t.getOrigin().distance(transform.getOrigin())<=0.001 &&
                           tf::angleShortestPath(old_t.getRotation(), transform.getRotation())/M_PI<=0.001 ) //HARDCODE!
                        {
                            publish = false;
                        }
                        //else FINFO(e.label() << " frame difference: pos " << old_t.getOrigin().distance(transform.getOrigin()) << " orientation: " << tf::angleShortestPath(old_t.getRotation(), transform.getRotation())/M_PI);
                    }
                    catch(...){}
                    if(publish)
                    {
                        e = wm_->getElement(e.id());
                        reasoner->storeData(e, transform.getOrigin(), "Position");
                        reasoner->storeData(e, transform.getRotation(), "Orientation");
                        wm_->updateElement(e);
                        while(!e.hasProperty(data::Str[data::FrameId]))
                            e = wm_->getElement(e.id());
                    }
                }
            }
        }
    }
    //If a transform is not found, or the three is not connected just don't publish
    catch(tf2::LookupException e){}
    catch(tf2::ConnectivityException e){}
    //This can happen if the world model is loading a new scene, just don't publish
    catch(std::invalid_argument e){}
    //Avoid crash, but print out the extrapolation error(that should never happen, unless of great lag on the network)
    catch(tf2::ExtrapolationException e){FERROR("[tfPubTimerCB]" << e.what());}
}

void SkillManager::reload(bool reload_plugins)
{
    //Load drivers
    boost::mutex::scoped_lock lock(devices_mutex_);
    loadDevices();
    wm_devices_ = wm_->getRobotHardware();
    lock.unlock();

    //Init skills and modules. Dependent on which devices are available...
    if(reload_plugins)
        init();

    //Load modules on world model
    loadModules();

    //Load skills on world model
    loadSkills();
}

void SkillManager::loadSkills()
{
    skiros_wm::Element robot = wm_->getRobot();
    if(robot.id()==-1)return;
    //Remove old
    std::vector<skiros_wm::Element> existing_skills = wm_->getChildElements(robot.id(), relation::Str[relation::hasSkill]);
    for(auto s : existing_skills)
    {
        wm_->removeElement(s.id());
    }
    //Add new
    for(SkillsMapType::value_type skill : loaded_skills_)
    {
        skiros_wm::Element wm_skill(skill.second->skillType());
        //TODO?: if(skill.second->isInitialized())

        //Add specific skill info (description, version, parameters)
        wm_skill.label() = skill.first;
        wm_skill.addPropertyString("description", skill.second->description());
        wm_skill.addPropertyString("version", skill.second->version());
        skiros_common::ParamHandler ph = skill.second->getParamHandlerCopy();
        skiros_common::ParamMap map = ph.getParamMap();
        for(skiros_common::ParamMap::value_type param : map)
        {
            wm_skill.addProperty(param.second);
        }
        wm_->addElement(wm_skill, robot.id(), relation::Str[relation::hasSkill]);
        std::vector<boost::shared_ptr<skiros::condition::ConditionBase> > v = skill.second->getPreConditions();
        for(boost::shared_ptr<skiros::condition::ConditionBase> c : v)
        {
            skiros_wm::Element e = c->toElement();
            wm_->addElement(e, wm_skill.id(), relation::Str[relation::hasPreCondition]);
        }
        auto post_conds = skill.second->getPostConditions();
        for(auto pair : post_conds)
        {
            skiros_wm::Element e = pair.second->toElement();
            wm_->addElement(e, wm_skill.id(), relation::Str[relation::hasPostCondition]);
        }
    }
}

//TODO: to be done properly
void SkillManager::configureModule(std::string name)
{
    skiros_common::ParamMap config_map = loaded_modules_[name]->getParamHandlerCopy().getParamMapFiltered(skiros_common::config);
    if(config_map.size()==0) return;
    skiros_common::ParamMap::iterator it;
    for(it=config_map.begin();it!=config_map.end();it++)
    {
        //TODO: now suppose only Elements in input. Extend
        if(it->second.type()!=typeid(skiros_wm::Element))continue;
        skiros_wm::Element temp = it->second.getValue<skiros_wm::Element>();
        //Get the first defined device that match the type
        BOOST_FOREACH(skiros_wm::Element e, wm_devices_)
        {
            if(e.type()==temp.type())
            {
                temp = e;
                break;
            }
        }
        //FINFO("Setting: " << it->second.key() << " to " << temp.printState("", true));
        it->second.setValue(temp);
    }
    loaded_modules_[name]->setParams(config_map);
}

void SkillManager::loadModules()
{
    skiros_wm::Element robot = wm_->getRobot();
    if(robot.id()==-1)return;
    //Remove old
    std::vector<skiros_wm::Element> existing_modules = wm_->getChildElements(robot.id(), relation::Str[relation::hasModule]);
    for(auto m : existing_modules)
    {
        wm_->removeElement(m.id());
    }
    //Add new
    for(ModulesMapType::value_type module : loaded_modules_)
    {
        skiros_wm::Element wm_module(module.second->skillType());
        //TODO?: if(module.second->isInitialized())

        //Add specific skill info (description, version, parameters)
        wm_module.label() = module.first;
        wm_module.addPropertyString("description", module.second->description());
        wm_module.addPropertyString("version", module.second->version());
        skiros_common::ParamHandler ph = module.second->getParamHandlerCopy();
        skiros_common::ParamMap map = ph.getParamMap();
        for(skiros_common::ParamMap::value_type param : map)
        {
            wm_module.addProperty(param.second);
        }
        wm_->addElement(wm_module, robot.id(), relation::Str[relation::hasModule]);
    }
}

std::list<skiros_wm::Element> SkillManager::getDrivers(skiros_wm::Element device)
{
    std::list<skiros_wm::Element> to_ret;
    std::list<skiros_wm::Element> temp;
    for(int i=0;i<device.properties("dependsOn").size();i++)
    {
        skiros_wm::Element driver = wm_->getDefaultElement(device.properties("dependsOn").getValue<std::string>(i));
        //Overwrite the address field
        if(device.hasProperty("DriverAddress") && driver.hasProperty("DriverAddress")) driver.removeProperty("DriverAddress");
        if(device.hasProperty("DriverAddress")) driver.addProperty(device.properties("DriverAddress"));
        temp.clear();
        if(driver.hasProperty("dependsOn")) temp = getDrivers(driver);
        to_ret.push_back(driver);
        to_ret.insert(to_ret.end(), temp.begin(), temp.end());
    }
    return to_ret;
}

//TODO: to be revised, will never work with Resource Manager active!
void SkillManager::loadDevices()
{
    skiros_wm::Element robot = wm_->getRobot();
    wm_devices_ = wm_->getRobotHardware(); //Pre-get HW (In case there the robot was already registered)
    if(robot.id()==-1)return;
    try
    {
            //Load the hardware list
            std::list<skiros_wm::Element> devices_list;
            {
                if(robot.hasProperty("hasA"))
                {
                    for(int i = 0;i<robot.properties("hasA").size();i++)
                    {
                        //Check if the HW is already loaded on the world model (to avoid adding it twice)
                        bool already_present = false;
                        BOOST_FOREACH(skiros_wm::Element e, wm_devices_)
                        {
                            if(e.label()==robot.properties("hasA").getValue<std::string>(i))already_present = true;
                        }
                        if(!already_present)
                            devices_list.push_back(wm_->getDefaultElement(robot.properties("hasA").getValue<std::string>(i)));
                    }
                }
            }
            //Load the driver list
            std::list<skiros_wm::Element> driver_list;
            std::list<skiros_wm::Element> loaded_driver_list;
            BOOST_FOREACH(skiros_wm::Element device, devices_list)
            {
                bool error = false;
                driver_list.clear();
                if(device.hasProperty("dependsOn")) driver_list = getDrivers(device);
                //Starts the drivers taking into consideration inter-dependencies
                std::map<std::string, int> fail_count_map;
                int device_id = wm_->addBranch(device, robot.id(), relation::Str[relation::hasA]);
                while(!driver_list.empty())
                {
                    skiros_wm::Element driver = driver_list.front();
                    //Looks for a missing dependency
                    bool missing_dep = false;
                    if(driver.hasProperty("dependsOn"))
                    {
                        for(int i=0;i<driver.properties("dependsOn").size();i++)
                        {
                            bool missing_dep_temp = true;
                            std::string temp = driver.properties("dependsOn").getValue<std::string>(i);
                            BOOST_FOREACH(skiros_wm::Element e, loaded_driver_list)
                            {
                                if(e.label()==temp) missing_dep_temp = false;
                            }
                            if(missing_dep_temp) missing_dep = true;
                        }
                    }

                    //Missing a dependency - push the device last and continue
                    if(missing_dep)
                    {
                        if(fail_count_map.find(driver.label())==fail_count_map.end()) fail_count_map[driver.label()] = 0;
                        fail_count_map[driver.label()]++;
                        //Anti-deadlock - max 3 tentative for driver
                        if(fail_count_map[driver.label()]<3)
                        {
                            //Rotate
                            driver_list.push_back(driver_list.front());
                            driver_list.pop_front();
                        }
                        else
                        {
                            error = true;
                            driver_list.pop_front();
                        }
                        continue;
                    }

                    //Start the driver
                    int ret_code = loadDriver(driver);
                    //Check the success
                    if(ret_code>0) //Success - add the driver to robot description
                    {
                        wm_->addElement(driver, device_id, relation::Str[relation::dependsOn]);
                        driver_device_map_.insert(DriverDevicePair(driver.properties("DriverAddress").getValue<std::string>(), driver.id()));
                        loaded_driver_list.push_back(driver);
                        driver_list.pop_front();
                    }
                    else //Fail - discard the driver
                    {
                        error = true;
                        driver_list.pop_front();
                    }
                }
                if(error) wm_->removeBranch(device_id);
            }
    }
    catch(std::runtime_error err)
    {
        FWARN("[SkillManager::loadDevices]" << err.what());
    }
}

int SkillManager::loadDriver(skiros_wm::Element driver)
{
    // Actual policy: If the resource_manager is not available we skip the loading and return a success
    if(!rm_->isConnected())
    {
        return 1;
    }
    std::stringstream ss;
    //std::string proxy = "";
    std::string address, pkg, name;
    if(driver.hasProperty("DriverParam"))
    {
        for(int i=0;i<driver.properties("DriverParam").size();i++)ss << driver.properties("DriverParam").getValue<std::string>(i);
    }
    address = driver.properties("DriverAddress").getValue<std::string>();
    pkg = driver.properties("DriverPackage").getValue<std::string>();
    name = driver.properties("DriverName").getValue<std::string>();
    FINFO("[SkillManager::loadDriver] Loading driver " << driver.label());
    if(rm_->loadDriver(driver.label(), address, pkg, name, ss.str()))
    {
        FINFO("[SkillManager::loadDriver] Driver loaded successfully.");
        return 1;
    }
    else
    {
        FERROR("[SkillManager::loadDriver] Failed to load driver. ");
        return 0;
    }
}

void SkillManager::shutdown()
{
    tfpub_timer_.stop();
    try
    {
      std::vector<skiros_wm::Element> v = wm_->getChildElements(wm_->getRobot());
      for(skiros_wm::Element e : v)
      {
          try
          {
              if(!rm_->unloadDriver(e.properties("DriverAddress").getValue<std::string>()))
                  if(rm_->isConnected())
                      FWARN("[SkillManager::shutdown] Failed to unload driver for device " << e.label());
          }
          catch(std::invalid_argument err)
          {
              //FWARN("[SkillManager::shutdown] Got error: " << err.what());
          }
      }
      wm_->unregisterRobot();
    }
    catch(std::runtime_error err)
    {
      //FWARN("[SkillManager::shutdown] Got error: " << err.what());
    }
    preemptAll();
    module_monitors_.clear();
    loaded_modules_.clear();
    loaded_skills_.clear();
}

} // namespace skiros_skill
