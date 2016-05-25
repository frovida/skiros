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

#include "skiros_config/param_types.h"
#include "skiros_common/logger_sys.h"
#include <pluginlib/class_list_macros.h>
#include <boost/foreach.hpp>

namespace skiros_config
{

      ParamTypes::ParamTypes()
      {
          param_loader_ = new pluginlib::ClassLoader<skiros_config::ParamBase>("skiros_config", "skiros_config::ParamBase");
          std::vector<std::string> declared_types = param_loader_->getDeclaredClasses();
          boost::shared_ptr<skiros_config::ParamBase> param_ptr;
          for(int i=0;i<declared_types.size();i++)
          {
              param_ptr = param_loader_->createInstance(declared_types[i].c_str());
              declared_types_.insert(ParamTypesPair(declared_types[i], param_ptr));
          }
      }

      ParamTypes::~ParamTypes(){}

      const std::vector<std::string> ParamTypes::types() const { return param_loader_->getDeclaredClasses(); }

      bool ParamTypes::isStdParam(std::string std_param_type){ return declared_types_.find(std_param_type)!=declared_types_.end(); }

      skiros_common::Param ParamTypes::getDefault(std::string std_param_type)
      {
          ParamTypesMap::iterator it = declared_types_.find(std_param_type);
          if(it == declared_types_.end())
          {
              std::stringstream ss;
              ss << "[skiros_config::ParamTypes] Standard parameter type: " << std_param_type << " not found. Available types are: ";
              for(ParamTypesPair pair : declared_types_)
              {
                  ss << pair.first << ", ";
              }
              throw std::runtime_error(ss.str().c_str());
          }
          return it->second->getParam();
      }

      std::string ParamTypes::getDataTypeStr(skiros_common::Param p)
      {
          if(p.type()==typeid(double))
              return XSD_DOUBLE;
          else if(p.type()==typeid(int))
              return XSD_INT;
          else if(p.type()==typeid(bool))
              return XSD_BOOL;
          else if(p.type()==typeid(std::string))
              return XSD_STRING;
          else
              return XSD_STRING;
      }

      skiros_common::Param ParamTypes::getFromString(std::string key,
                                                     std::string data_type,
                                                     std::string name,
                                                     skiros_common::ParamSpecType param_type,
                                                     int size)
      {
          if(data_type==XSD_DOUBLE)
          {
              return skiros_common::Param(key, name, typeid(double), param_type, size);
          }
          else if(data_type==XSD_STRING)
          {
              return skiros_common::Param(key, name, typeid(std::string), param_type, size);
          }
          else if(data_type==XSD_INT)
          {
              return skiros_common::Param(key, name, typeid(int), param_type, size);
          }
          else if(data_type==XSD_BOOL)
          {
              return skiros_common::Param(key, name, typeid(bool), param_type, size);
          }
          else
          {
              return skiros_common::Param(key, name, typeid(std::string), param_type, size);
          }
      }

}
