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

//Some predefined params

namespace skiros_param
{
    class DiscreteReasoner : public skiros_config::ParamBase
    {
    public:
        DiscreteReasoner()
        {
            registerAny<std::string>();
        }
        ~DiscreteReasoner(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("DiscreteReasoner", "Associated discreate reasoners", typeid(std::string), skiros_common::symbolic, 1);
            return p;
        }
    };

    class SkillMgr : public skiros_config::ParamBase
    {
    public:
        SkillMgr()
        {
            registerAny<std::string>();
        }
        ~SkillMgr(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("SkillMgr", "ROS name of the skill manager", typeid(std::string), skiros_common::symbolic, 1);
            return p;
        }
    };

    class Name : public skiros_config::ParamBase
    {
    public:
        Name()
        {
            registerAny<std::string>();
        }
        ~Name(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("Name", "Name", typeid(std::string), skiros_common::symbolic, 1);
            return p;
        }
    };

    class Position : public skiros_config::ParamBase
    {
    public:
        Position()
        {
        }
        ~Position(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("Position", "Position", typeid(double), skiros_common::discrete, 3);
            return p;
        }
    };

    class Orientation : public skiros_config::ParamBase
    {
    public:
        Orientation()
        {
        }
        ~Orientation(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("Orientation", "Orientation", typeid(double), skiros_common::discrete, 4);
            return p;
        }
    };

    class Size : public skiros_config::ParamBase
    {
    public:
        Size()
        {
        }
        ~Size(){}
        skiros_common::Param getParam()
        {
            skiros_common::Param p;
            p.reset("Size", "Size", typeid(double), skiros_common::discrete, 3);
            return p;
        }
    };
}

PLUGINLIB_EXPORT_CLASS(skiros_param::SkillMgr, skiros_config::ParamBase)
PLUGINLIB_EXPORT_CLASS(skiros_param::Name, skiros_config::ParamBase)
PLUGINLIB_EXPORT_CLASS(skiros_param::DiscreteReasoner, skiros_config::ParamBase)
PLUGINLIB_EXPORT_CLASS(skiros_param::Size, skiros_config::ParamBase)
PLUGINLIB_EXPORT_CLASS(skiros_param::Orientation, skiros_config::ParamBase)
PLUGINLIB_EXPORT_CLASS(skiros_param::Position, skiros_config::ParamBase)
