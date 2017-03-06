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

#include "skiros_skill/skill_base.h"
#include "skiros_common/logger_sys.h"
#include "skiros_world_model/condition.h"
#include "skiros_config/param_types.h"
#include "skiros_msgs/ModuleStatus.h"

using namespace skiros::state;

namespace skiros_skill
{
    SkillBase::SkillBase()
    {
        //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
        skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    }

    //---------------------------------------------------------------------------

    void SkillBase::init(boost::shared_ptr<ros::NodeHandle> nh, boost::shared_ptr<skiros_wm::WorldModelInterfaceS> world_model, boost::shared_ptr<SkillManagerInterface> mi)
    {
        nh_ = nh;
        world_model_ = world_model;
        mi_ = mi;
        //state_pub_ = nh_->advertise<skiros_msgs::ModuleStatus>(world_model_->getRobot().properties("SkillMgr").getValue<std::string>()+skill_monitor_tpc_name, 20);
        this->setResultCode(0);
        //Skill set to stanby if initialization return true
        if(this->onInit())this->setState(skiros::state::idle);
        else this->setState(skiros::state::uninitialized);
    }

	//---------------------------------------------------------------------------
	//Check pre-conditions, set the skill state to running and start the execution routine. Then check post-conditions.
    void SkillBase::start()
    {
        int ret;
        if(param_are_specified_ && isInitialized())
        {
              initProgress();
              setState(running, false);
                ret = preSense();
              if(ret<0)
              {
                  this->setResultCode(ret);
                  this->setState(error);
                  return;
              }
			  if(this->checkPreConditions())
			  {
                  initProgress();
                  setState(running, false);
                  ret = this->execute();
                  if(ret<0)
                  {
                      this->checkPostConditions();
                      this->setResultCode(ret);
                      this->setState(error);
                      return;
                  }
                  ret = this->postSense();
                  if(ret<0)
                  {
                      this->checkPostConditions();
                      this->setResultCode(ret);
                      this->setState(error);
                      return;
                  }
                  auto final_progress = this->getProgress();
                  if(!this->checkPostConditions())
                  {
                      this->setResultCode(-1);
                      this->setState(error);
                  }
                  else
                  {
                      setState(terminated, false);
                      setProgress(final_progress);
                  }
                  this->setResultCode(ret);
                  return;
			  }
			  else
              {
                  this->setResultCode(-1);
                  this->setState(error);
                  return;
			  }
		}
		else
		{
              this->setState(error);
              FERROR("[SkillBase::start]Parameters haven't been specified or skill is not initialized. Specify the parameters before executing the skill.");
              setResultCode(-1);
              return;
		}
	}

	//---------------------------------------------------------------------------
	//Return true if all pre conditions are met, false otherwise
	bool SkillBase::checkPreConditions()
	{
	  bool ok = true;
      setProgress("Evaluating preconditions.");
	  for(int i=0;i<pre_conditions_.size();i++)
	  {
          if(pre_conditions_[i]->evaluate()) {setProgress(pre_conditions_[i]->getDescription()+" : OK");}
          else {setProgress(-1, pre_conditions_[i]->getDescription() + " : FAIL");ok = false;}
      }
	  return ok;
	}

	//---------------------------------------------------------------------------
	//Return true if all post conditions are met, false otherwise
	bool SkillBase::checkPostConditions()
	{
	  bool ok = true;
      setProgress("Evaluating postconditions.");
      for(auto pair : post_conditions_)
	  {
          if(pair.second->evaluate()) {setProgress(pair.second->getDescription() + " : OK");}
          else {setProgress(-1, pair.second->getDescription() + " : FAIL");ok = false;}
      }
	  return ok;
	}


    void SkillBase::printStatus(bool publish)
    {
        skiros_msgs::ModuleStatus msg;
        std::ostringstream output;
        if (getProgress().id == 0) output << "\033[1;32m";  // Bold green
        else if (getState() == preempted) output << "\033[0;31m"; // Red
        else if (getState() == error) output << "\033[1;31m"; // Bold red
        else if (getState() == running) output << "\033[0;32m"; // green
        else if (getState() == terminated) output << "\033[1;32m"; // Bold green
        output << skillType() << "[" << StateStr[getState()]
                << "]: "  << "[" << getProgress().id << "]" <<  getProgress().description;
        output << "\033[0m";
        FINFO(output.str());
        //FINFO(moduleType() << "[" << StateStr[getState()]
        //        << "]: "  << "[" << getProgress().id << "]" <<  getProgress().description);
        msg.header.stamp = ros::Time::now();
        msg.module.name = moduleType();
        msg.status = StateStr[getState()];
        //msg.module.parameters_in = getParamHandle().getParamMap();
        msg.progress_code = getProgress().id;
        msg.progress_description = getProgress().description;
        state_pub_.publish(msg);
    }

}
