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

#ifndef COMMON_UTILITY_HPP
#define COMMON_UTILITY_HPP

#include "skiros_common/param.h"
#include "skiros_msgs/ParamMapSerialized.h"

namespace skiros_common
{
namespace utility
{
    struct SerialData
    {
    std::vector<uint8_t> data;
    uint32_t length;
    };

    std::vector<skiros_common::any> deserializeAnyVector(SerialData any_ser);
    SerialData serializeAnyVector(std::vector<skiros_common::any> any_v);

    skiros_common::ParamMap deserializeParamMap(skiros_msgs::ParamMapSerialized ph_ser);
    skiros_msgs::ParamMapSerialized serializeParamMap(skiros_common::ParamMap map);

    //! Return the full path of the default SkiROS save folder
    std::string getSkirosSaveDirectory();

    std::vector<std::string> getFilesInFolder(std::string folder_name, std::string filter);

    //! Scan a folder and query an input from the user using cin<<
    std::string browseFilesInFolder(std::string folder_name, std::string filter);
}
}

#endif // COMMON_UTILITY_HPP
