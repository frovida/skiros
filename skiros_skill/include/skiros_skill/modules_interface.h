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

#ifndef MODULES_INTERFACE_H
#define MODULES_INTERFACE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include "skiros_world_model/world_model_interface.h"
#include <skiros_msgs/WmMonitor.h>
#include <skiros_skill/skill_manager_interface.h>

namespace skiros
{
    typedef std::map<std::string, skiros_common::ParamMap> ModuleMapType;
    typedef std::pair<std::string, skiros_common::ParamMap> ModulePairType;
    /*!
     * \brief Front-end class for users. The Module class wraps the SkillManagerInterface to offer a clean interface to modules
     *
     * Modules are similar to C++ functions or ROS services. User sets some parameters in input and command the execution.
     * The module respond with "success/fail" and a set of return parameters (if any is defined)
     *
     */
    class Module
    {
    public:
        Module(boost::shared_ptr<skiros_skill::SkillManagerInterface> mi, std::string name, std::string controller);
        ~Module(){}

        /*!
         * \brief Command the module execution
         * \param doneCb Optional callback when execution ends
         * \param feedbackCb Optional callback whenever a feedback from the module is received
         * \return True on success. It is possible to access the final return values with the getReturnValue function.
         */
        bool exe(skiros_skill::DoneCallback doneCb=NULL, skiros_skill::FeedbackCallback feedbackCb=NULL);

        /*!
         * \brief Stop the module execution
         * \return true if the module was successfully stopped, false otherwise
         */
        bool stop();

        /*!
         * \brief Wait until the end of the module execution
         * \return  True if the module was executed correctly, false otherwise
         */
        bool waitResult();

        template<class T>
        /*!
         * \brief Set an input parameter
         * \param key, the parameter id
         * \param value, the value to assign to the param. It must be of the same datatype as defined in the param description
         */
        void setParam(std::string key, T value)
        {
            input_values_.at(key).setValue(value);
        }

        //! \brief Unset an input parameter
        inline void unsetParam(std::string key){input_values_.at(key).unsetValue(); }

        //! \brief set the default value for all input parameters
        void setDefault();

        template<class T>
        /*!
         * \brief Get the value of an input parameter
         * \param key, the parameter id
         * \return The parameter value
         */
        T getParamValue(std::string key)
        {
            return input_values_.at(key).getValue<T>();
        }
        template<class T>
        /*!
         * \brief Get the value of a return parameter
         * \param key, the parameter id
         * \return The parameter value
         */
        T getReturnValue(std::string key)
        {
            if(return_values_.find(key)==return_values_.end())
            {
                for(auto p : return_values_)
                    ROS_INFO_STREAM(p.second.printState());
                throw std::invalid_argument(key + " parameter is not in the map. ");
            }
            return return_values_.at(key).getValue<T>();
        }

    private:
        boost::shared_ptr<skiros_skill::SkillManagerInterface> mi_;
        skiros_common::ParamMap input_values_;
        skiros_common::ParamMap return_values_;
        int execution_id_;
        std::string controller_name_;
        std::string name_;
    };
}

#endif // MODULES_INTERFACE_H
