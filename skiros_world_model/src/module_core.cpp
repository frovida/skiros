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

#include "skiros_world_model/module_core.h"
#include "skiros_common/logger_sys.h"
#include <boost/foreach.hpp>

namespace skiros
{

using namespace state;

void ModuleCore::start()
{
    int ret;
    if(param_are_specified_ && isInitialized())
    {
        setState(running);
        ret = execute();
        if(ret<0)
        {
          setState(error);
          setResultCode(ret);
          //FERROR("Skill failed on execution with error code: " << ret);
          return;
        }
    }
    else
    {
          setState(error);
          FERROR("[" << this->moduleType() << "]Parameters haven't been specified or the module is not initialized. Specify the parameters before executing the module.");
          setResultCode(-1);
          return;
    }
    setState(terminated);
}

//---------------------------------------------------------------------------
bool ModuleCore::setParams(skiros_common::ParamMap input_param_map)
{
  //TODO a real, detailed check on all parameters validity
  std::map<std::string,skiros_common::Param> param_map = ph_ptr_->getParamMap();
  std::map<std::string,skiros_common::Param>::iterator it = param_map.begin();
  std::map<std::string,skiros_common::Param>::iterator it2 = param_map.begin();
  std::vector<skiros_common::any> v;
  param_are_specified_ = true;
  //Scan all the parameter of the skill
  for (it=param_map.begin(); it!=param_map.end(); ++it)
  {
      //Look for the parameter in the input map
      it2 = input_param_map.find(it->second.key());
      //Case: parameter not found in input
      if(it2 == param_map.end())
      {
          //If the param is already specified or is optional the loop goes forward normally
          if(it->second.state()==skiros_common::specified || it->second.specType() == skiros_common::optional)continue;
          else //Otherwise set the ready flag to 0 and go forward normally
          {
              FWARN("Element " << it->second.key() << " not found.");
              //Not all parameters are specified, the flag goes to 0
              param_are_specified_ = false;
              continue;
          }
      }
      else
      {
          //Case: parameter found in input
          if(it2->second.state() == skiros_common::specified)
          {
                  it2->second.getValues(v);
                  ph_ptr_->specify(it->second.key(), v);
          }
          else
          {
              //If the param is already specified or is optional loop goes forward normally
              if(it->second.state()==skiros_common::specified || it->second.specType() == skiros_common::optional)continue;
              else //Otherwise set the ready flag to 0 and go forward normally
              {
                  FWARN("Element " << it->second.key() << " hasn't been specified.");
                  //Not all parameters are specified, the flag goes to 0
                  param_are_specified_ = false;
                  continue;
              }
          }
      }

  }
  return param_are_specified_;
}

//---------------------------------------------------------------------------

bool ModuleCore::setParams(skiros_common::ParamHandler ph)
{
    std::map<std::string,skiros_common::Param> input_param_map = ph.getParamMap();
    return this->setParams(input_param_map);
}

//---------------------------------------------------------------------------
void ModuleCore::initProgress()
{
    std::stringstream ss;
    ss << " Params - \n";
    for(skiros_common::ParamMap::value_type pair : ph_ptr_->getParamMap())
    {
        std::string temp = pair.second.printState();
        temp = temp.substr(0, temp.find("Type"));
        ss << temp << "\n";
    }
    this->setState(started, false);
    setProgress(0, ss.str());
}



}
