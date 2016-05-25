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

#include "ros/ros.h"
#include <map>
#include <vector>
#include "skiros_common/node_spawner.h"
#include <boost/foreach.hpp>
#include <sstream>


/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle node_h;
    skiros_common::RosNodeSpawner nd;
    /*if(argc<4)
    {
        std::cout << "Usage: pkg_name, executable_name, parameters. ";
        return -1;
    }
    nd.execute(argv[1], argv[2], argv[3]);*/

    while(true)
    {
        char c = 'n';
        std::cout << "Type: 'l' list processes, 't' terminate process, 'e' execute process, 'c' close." << std::endl;
        std::cin >> c;
        if(c=='e')
        {
            std::string pkg_name, exe_name, params;
            std::cout << "Insert package:" << std::endl;
            std::getline(std::cin, pkg_name);
            std::getline(std::cin, pkg_name);
            std::cout << "Insert executable:" << std::endl;
            std::getline(std::cin, exe_name);
            std::cout << "Insert parameters:" << std::endl;
            std::getline(std::cin, params);
            nd.execute(pkg_name, exe_name, params);
        }
        else if(c=='l')
        {
            std::vector<skiros_common::SpawnerProcessData> list = nd.listLoaded();
            BOOST_FOREACH(skiros_common::SpawnerProcessData data, list)
            {
                std::cout << data.id << " " << data.exe_name << " " << data.status << std::endl;
            }
        }
        else if(c=='t')
        {
            int pid;std::string input;
            std::cout << "Insert process pid:" << std::endl;
            std::getline(std::cin, input);
            std::getline(std::cin, input);
            // This code converts from string to number safely.
            std::stringstream myStream(input);
            myStream >> pid;
            if(pid>0) nd.shutdown(pid);
        }
        else if(c=='c')break;
    }
    FINFO("Closing");
    //----------------------- Close --------------------------------
	return 0;
} // main
