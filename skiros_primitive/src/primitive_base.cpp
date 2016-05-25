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

#include <skiros_primitive/primitive_base.h>
#include "skiros_common/logger_sys.h"
#include "skiros_common/utility.h"

using namespace skiros;

namespace skiros_primitive
{
    void BasePrimitive::onInit()
    {
        max_queue_size_ = 10;
        //------------------- Publishers --------------------
        state_pub_ = getMTNodeHandle().advertise<skiros_msgs::PrimitiveEvent> (STATUS_TOPIC, max_queue_size_);
        event_pub_ = getMTNodeHandle().advertise<skiros_msgs::PrimitiveEvent> (EVENT_TOPIC, max_queue_size_);
        //------------------- Subscribers --------------------
        trigger_sub_ = getMTPrivateNodeHandle().subscribe(TRIGGER_TOPIC, 1, &BasePrimitive::triggerCB, this);
        config_sub_ = getMTPrivateNodeHandle().subscribe(SET_CONFIG_TOPIC, 1, &BasePrimitive::setConfigCB, this);
        config_pub_ = getMTPrivateNodeHandle().advertise<skiros_msgs::ParamMapSerialized>(GET_CONFIG_TOPIC, max_queue_size_);
        //publish_state_timer_ = private_nh_->createTimer(ros::Duration(1.0), &BasePrimitive::statePubCB, this, false, true);
    }

    void BasePrimitive::setConfigCB(skiros_msgs::ParamMapSerialized msg)
    {
        if(setParams(skiros_common::utility::deserializeParamMap(msg))) publishEvent(CONFIG_EVENT);
        else publishEvent(BAD_CONFIG_EVENT);
    }

    void BasePrimitive::triggerCB(std_msgs::Bool msg)
    {
        if(state_!=state::uninitialized)
        {
            this->start();
        }
    }

    void BasePrimitive::setStateAndPublish(state::StateType new_state, bool publish)
    {
        state::StateType prev_state = getState();
        setState(new_state, false);
        if(publish && prev_state!=new_state) publishState();
    }

    void BasePrimitive::publishEvent(std::string event)
    {
        skiros_msgs::PrimitiveEventPtr ptr(new skiros_msgs::PrimitiveEvent);
        ptr->header.stamp = ros::Time::now();
        ptr->primitive_name = this->getName();
        ptr->status = state_;
        ptr->event_type = event;
        event_pub_.publish(ptr);
    }

    void BasePrimitive::publishState()
    {
        skiros_msgs::PrimitiveEventPtr ptr(new skiros_msgs::PrimitiveEvent);
        ptr->header.stamp = ros::Time::now();
        ptr->primitive_name = this->getName();
        ptr->status = state_;
        state_pub_.publish(ptr);
    }
}
