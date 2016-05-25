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

#ifndef PARAM_BASE_H
#define PARAM_BASE_H

#include "skiros_common/param.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"

//------------- Definition of ros serialization. ----------------------------
//The serialization policy must be defined for every non-trivial type we want to store as parameter
//There are 2 way of define the serialization, documentation at: http://wiki.ros.org/roscpp/Overview/MessagesSerializationAndAdaptingTypes
namespace ros
{
namespace serialization
{
  template<>
  struct Serializer<tf::Pose>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const tf::Pose& t)
    {
      geometry_msgs::Pose msg;
      tf::poseTFToMsg(t, msg);
      stream.next(msg);
    }

    template<typename Stream>
    inline static void read(Stream& stream, tf::Pose& t)
    {
      geometry_msgs::Pose msg;
      stream.next(msg);
      tf::poseMsgToTF(msg, t);
    }

    inline static uint32_t serializedLength(const tf::Pose& t)
    {
        uint32_t size = 0;
        geometry_msgs::Pose msg;
        tf::poseTFToMsg(t, msg);
        size += serializationLength(msg);
        return size;
    }
  };

  template<>
  struct Serializer<tf::Vector3>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const tf::Vector3& t)
    {
      geometry_msgs::Vector3 msg;
      tf::vector3TFToMsg(t, msg);
      stream.next(msg);
    }

    template<typename Stream>
    inline static void read(Stream& stream, tf::Vector3& t)
    {
      geometry_msgs::Vector3 msg;
      stream.next(msg);
      tf::vector3MsgToTF(msg, t);
    }

    inline static uint32_t serializedLength(const tf::Vector3& t)
    {
      uint32_t size = 0;
      geometry_msgs::Vector3 msg;
      tf::vector3TFToMsg(t, msg);
      size += serializationLength(msg);
      return size;
    }
  };

  template<>
  struct Serializer<tf::Quaternion>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const tf::Quaternion& t)
    {
        geometry_msgs::Quaternion msg;
        tf::quaternionTFToMsg(t, msg);
        stream.next(msg);
    }

    template<typename Stream>
    inline static void read(Stream& stream, tf::Quaternion& t)
    {
        geometry_msgs::Quaternion msg;
        stream.next(msg);
        tf::quaternionMsgToTF(msg, t);
    }

    inline static uint32_t serializedLength(const tf::Quaternion& t)
    {
        uint32_t size = 0;
        geometry_msgs::Quaternion msg;
        tf::quaternionTFToMsg(t, msg);
        size += serializationLength(msg);
        return size;
    }
  };
}
}

namespace skiros_config
{
 /* enum DefaultParams
  {
          APPROACH_DIRECTION ,
          APPROACH_DISTANCE ,
          COMMON_APPR_LEAVE ,
          FRAME_TYPE ,
          LEAVING_DIRECTION ,
          LEAVING_DISTANCE,
          MOTION_TYPE ,
          ORIENTATION_FREE ,
          SEARCH_VELOCITY,
          STIFFNESS ,
          TRIGGER_FORCE ,
          VELOCITY,
  };*/

  class ParamBase
  {
  protected:
      ParamBase(){}

      template<typename T>
      void registerAny()
      {
        T temp;
        //For the serialization, It is necessary to istanciate at least 1 time the any associate to a type
        skiros_common::any a(temp);
      }

  public:
      virtual ~ParamBase() {}
      virtual skiros_common::Param getParam() = 0;
  };
}
#endif //PARAM_BASE_H
