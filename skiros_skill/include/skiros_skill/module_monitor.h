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

#ifndef MODULE_MONITOR_H
#define MODULE_MONITOR_H

#include <skiros_skill/module_base.h>
#include <skiros_msgs/ModuleStatus.h>
#include <skiros_common/utility.h>
#include <boost/thread.hpp>

namespace skiros
{
    /*!
     * \brief The ModuleMonitor class monitor the state of a module and publish it on the ROS network
     *
     * Roadmap: the monitor class should be extended to publish also on an event channel in case of configurable conditions
     *
     */
    class ModuleMonitor
    {
        ros::Time start_time;
    public:
        ModuleMonitor(boost::shared_ptr<skiros::ModuleCore> module, std::string module_name, std::string pub_topic_name_, bool is_skill=false, bool start=true): is_skill_(is_skill), nh_(""), module_(module), topic_name_(pub_topic_name_), module_name_(module_name)
        {
            if(start)
                this->start();
        }

        ~ModuleMonitor()
        {
            stop();
        }

        void saveLog(ProgressAndState & out)
        {

            std::stringstream ss;
        }

        void start()
        {
            state_pub_ = nh_.advertise<skiros_msgs::ModuleStatus>(topic_name_, 20);
            working_thread_ = boost::thread(boost::bind(&ModuleMonitor::monitorExe, this));
        }

        void stop()
        {
            void *dont_care;
            pthread_cancel(working_thread_.native_handle());
            pthread_join(working_thread_.native_handle(), &dont_care);
        }

    private:
        void monitorExe()
        {
            while(ros::ok())
            {
              ProgressAndState out = module_->waitOutput();
              printStatus(out);
            }
        }

        void printStatus(ProgressAndState out);

        ros::NodeHandle nh_;
        boost::shared_ptr<skiros::ModuleCore> module_;
        std::string topic_name_;
        std::string module_name_;
        ros::Publisher state_pub_;
        boost::thread working_thread_;
        bool is_skill_;
    };
}

#endif // MODULE_MONITOR_H
