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

#include <ros/ros.h>
#include <typeinfo>
#include <signal.h>
#include "skiros_common/logger_sys.h"
#include "skiros_common/param_handler.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_config/param_types.h"
#include "skiros_config/declared_uri.h"

using namespace skiros_wm;
using namespace skiros_config;
using namespace skiros_wm::owl;
using namespace skiros_config::owl;


//Assure unregister the robot before shutting down
bool KEEP_RUNNING;

/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////


void sigIntHandler(int sig)
{
  KEEP_RUNNING = false;
}

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////


int main (int argc, char **argv)
{
	// Initialise the ros node
    ros::init(argc, argv, "populate_wm", ros::init_options::NoSigintHandler);
    // Override default exit handlers for roscpp
    signal(SIGINT, sigIntHandler);
    KEEP_RUNNING = true;
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */

    if(argc<2) return -1;
    std::string object_to_add = argv[1];
    ros::NodeHandle nh;
    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

    //Wait the world model to be up
    skiros_wm::WorldModelInterface wm(nh);
    if(!wm.isConnected()) FWARN("World model seems down, waiting...");

    while(!wm.isConnected() && KEEP_RUNNING) sleep(0.5);

    FINFO("Connected to wm, adding object: " << object_to_add);

    Element e = wm.getDefaultElement(object_to_add);

    e.id() = wm.addBranch(e, 0, relation::Str[relation::contain]);

    //Infinite ROS queue spin
    while(ros::ok() && KEEP_RUNNING)
    {
        ros::spinOnce();
        sleep(1);
    }
    wm.removeBranch(e.id());

    ros::shutdown();
    return 0;

} // main
