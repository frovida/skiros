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

#ifndef BASE_SKILL_H
#define BASE_SKILL_H

#include "skiros_world_model/module_core.h"
#include "skiros_skill/modules_interface.h"

namespace skiros_skill
{




/*!
    \brief Base class for all skills.

    The skills are the atomic software blocks that operate a modification on the world model.
    The skills are derived from this base class and imported in SkiROS using the pluginlib package system.
    Programming inside a skill is not much different from any other C++ program, except for few rules.
    Each skill must define its constructor and overwrite the abstract methods: onInit, preSense, execute and postSense.
    In the constructor and onInit methods the skill should define:
        -A pre- and post-condition set
        -A set of input parameters
    To interact with the rest of the system, the skills are provided with:
        -The ROS node handle, advertise/submit to ROS topics, services and actions
        -The world model interface, offers I\O with the SkiROS world model
        -The parameters handle, to manage parameters
    Internally, the skills can connect to all other ROS nodes as usual.
*/
class SkillBase : public skiros::ModuleCore
{
protected:
    //--------- Polymorphic methods --------------
    //These methods must be specialized inside the specific skill
    /*! \brief Initialize the skill
      Here should be defined:
        -pre-/post- conditions
        -persistents ROS listeners/advertiser
      */
    virtual bool onInit() = 0;
    //! \brief specialized preempt routine
    virtual void onPreempt() {}
    //! \brief Skill's main execution routine
    virtual int execute() = 0;
    //! \brief Execute a sensing routine before the pre-condition check
    virtual int preSense() {return 1;}
    //! \brief Execute a sensing routine before the post-condition check
    virtual int postSense() {return 1;}

public:
    virtual ~SkillBase() {}

    //--------- SkiROS system methods --------------
    //! \brief Check that the parameters are all defined, check pre conditions, set the state to running, then calls the preSense, execute and postSense functions
    void start();
    //! \brief Initializes skill's resources, then call the onInit() function
    void init(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model, boost::shared_ptr<skiros_skill::SkillManagerInterface> mi);

    inline std::vector<boost::shared_ptr<skiros::condition::ConditionBase> > getPreConditions(){return pre_conditions_;}
    inline std::map<std::string, boost::shared_ptr<skiros::condition::ConditionBase> > getPostConditions(){return post_conditions_;}

    void printStatus(bool publish=true);
protected:
    inline boost::shared_ptr<skiros::condition::ConditionBase> newCondition(std::string condition_name, bool desired_state, std::string subject_key, std::string object_key="")
    {
        return skiros::condition::loadCondition(getWorldHandle(), getParamHandle(), condition_name, desired_state, subject_key, object_key);
    }
    //! \brief Add a precondition check
    inline void addPrecondition(boost::shared_ptr<skiros::condition::ConditionBase>  condition) { pre_conditions_.push_back(condition); }
    //! \brief Add a postcondition check
    void addPostcondition(std::string key, boost::shared_ptr<skiros::condition::ConditionBase>  condition){post_conditions_.insert(std::pair<std::string, boost::shared_ptr<skiros::condition::ConditionBase> >(key, condition));}
    //! \brief Set a postcondition to the desired value
    bool setPostcondition(std::string condition)
    {
        if(post_conditions_.at(condition)->set()) return true;
        else return false;
    }
    //! \brief Set all postconditions to the desired value
    void setAllPostConditions()
    {
        for(auto pair : post_conditions_)
            pair.second->set();
    }

    //! \brief Interface to modules. There is no need to use this interface directly, see the Module class in ModuleInterface.h for more information.
    inline boost::shared_ptr<skiros_skill::SkillManagerInterface>  getModulesHandler(){return mi_;}
public:
    SkillBase();
private:
    //! \brief Check pre-conditions
    bool checkPreConditions();
    //! \brief Check post-conditions
    bool checkPostConditions();
    //!< Vectors store Pre and Post conditions
    std::vector<boost::shared_ptr<skiros::condition::ConditionBase> > pre_conditions_;
    std::map<std::string, boost::shared_ptr<skiros::condition::ConditionBase> > post_conditions_;
    boost::shared_ptr<skiros_skill::SkillManagerInterface> mi_;
    ros::Publisher state_pub_;
};

} //namespace skiros_skill

#endif // BASE_SKILL_H
