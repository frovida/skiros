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
#include "skiros_skill/modules_interface.h"
#include "skiros_msgs/ModulesListQuery.h"
#include "skiros_msgs/ModuleCommand.h"
#include "skiros_config/declared_uri.h"
#include "boost/foreach.hpp"
#include <skiros_world_model/utility.h>

using namespace skiros_config;
using namespace skiros_config::owl;
using namespace skiros_wm;
using namespace skiros_skill;

namespace skiros
{

Module::Module(boost::shared_ptr<SkillManagerInterface> mi, std::string name, std::string controller) : mi_(mi), name_(name), controller_name_(controller), execution_id_(-1)
{
    setDefault();
}

void Module::setDefault()
{
    skiros::ModuleMapType modules_map = mi_->getModuleList(true);
    if(modules_map.find(name_)==modules_map.end())
    {
        std::stringstream ss;
        ss << "[Module::Module] Module with name " << name_ << " not found.";
        throw std::invalid_argument(ss.str());
    }
    input_values_ = modules_map[name_];
}

bool Module::exe(DoneCallback doneCb, FeedbackCallback feedbackCb)
{
    execution_id_ = mi_->exeModule(name_, input_values_, controller_name_, doneCb, feedbackCb);
    if(execution_id_<0)return false;
    else return true;
}

bool Module::stop()
{
    if(execution_id_<0)return false;
    if(mi_->stopModule(execution_id_, name_, controller_name_))
    {
        execution_id_ = -1;
        return true;
    }
    else return false;
}

bool Module::waitResult()
{
    ExecutionResult exe_r = mi_->waitResult(name_);
    return_values_ = exe_r.output;
    if(return_values_["Return"].getValue<bool>()) return true;
    else return false;
}

}
