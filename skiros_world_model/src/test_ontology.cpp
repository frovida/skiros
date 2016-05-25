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

#include <dirent.h>
#include <ros/ros.h>
#include "skiros_msgs/WmSetRelation.h"
#include "skiros_msgs/WmQuery.h"
#include "skiros_msgs/WmElementGet.h"
#include "skiros_msgs/WmElementModify.h"
#include "skiros_msgs/WmMonitor.h"
#include "skiros_msgs/WmIdentify.h"
#include "skiros_common/utility.h"
#include "skiros_config/param_types.h"
#include "skiros_world_model/utility.h"
#include "skiros_world_model/owl_world_model.h"
#include "skiros_world_model/owl_world_ontology.h"
#include "skiros_world_model/reasoners_loading_func.h"
#include <stdio.h>
#include <boost/foreach.hpp>

using namespace skiros_wm;

//Global
//static skiros_wm::WorldOntology ontology(model);
ros::Publisher monitor;

/////////////////////////////////////////////////////////////////////////////
// Query\Modify services
/////////////////////////////////////////////////////////////////////////////

//CONSTRAINT LIST (TODO: find a place to formally define constraints in the world model)
/*
A skill manager name should be related to only one robot in the WM
Cannot remove a contain relation directly.

  */
#include<time.h>



/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////

int main (int argc, char **argv)
{

	// Initialise the ros node
    ros::init(argc, argv, "test_node"); //ros::this_node::getName()
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */

	ros::NodeHandle node_h;
	//Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
	skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    //model.init();
	//world model has to inizialize the services for the interface
	//Create services:
	//ROS_INFO("%s %s", ros::this_node::getNamespace().c_str(), node_h.resolveName("world_model").c_str());

	// Use multiple threads to handle callbacks
	ros::AsyncSpinner spinner(2);
	spinner.start();

    skiros_wm::owl::Ontology ontology;
    skiros_wm::owl::WorldModel model(ontology);
    /*ros::WallTime t = ros::WallTime::now();
    ros::Time t2 = ros::Time::now();
    char timestamp[50];
    struct tm * timeinfo;
    time_t ttt = (time_t) t.toSec();
    timeinfo = localtime(&ttt);
    strftime(timestamp, sizeof(timestamp), "%FT%X", timeinfo);
    FINFO("ROS time: " << t.toSec()  << "ROS sim time: " << t2.toSec()   << " Timestamp " <<  timestamp);*/
    std::vector<int> id_list;

	while(ros::ok())
	{
	    char c = 'n';
        std::cout << "Char: 'l' load a different world model. 'p' print the world. 'q' query. 'v' visualize element tree. 'e' add element. 'r' remove. 'c' close." << std::endl;
	    std::cin >> c;
	    if (c == 'l') //Load a file
	    {
            //Load an ontology
            //TODO: if argc > 1 then load argv[1]
            while(ros::ok() && !ontology.isInitialized())
            {
                std::string filename = skiros_common::utility::browseFilesInFolder(ontology.getWorkspacePath(), ".owl");
                try
                {
                    model.loadMainOntology(filename);
                    std::cout << "File loaded successfully." << std::endl;
                }
                catch(std::runtime_error e)
                {
                    FERROR(e.what());
                }
            }
        } else if (c == 'p')
        {
            ontology.printOntology();
        } else if (c == 'q')
        {

        } else if (c == 'v')
        {
            /*std::list<std::string> list = ontology.getBranchIndividuals(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, owl::std_uri::STMN+"#PhysicalObject"));
            BOOST_FOREACH(std::string str, list)
            {
                FINFO(str);
            }*/
            std::cout << model.getSceneTree() << std::endl;

        } else if (c == 'e')
        {
            try
            {
                skiros_wm::Element e = model.getDefaultElement("uauaua");
            }
            catch(std::invalid_argument err){FERROR(err.what());}

            RelationsVector relations;
            skiros_wm::Element e = model.getDefaultElement("aau_stamina_robot");
            std::cout << e.printState() << std::endl;
            skiros_wm::Element e2 = model.getDefaultElement("large_box");
            model.addElement(e2, relations);
            relations.push_back(RelationType(-1, "ueue", e2.id()));
            relations.push_back(RelationType(e2.id(), "contain", -1));
            model.addElement(e, relations);
            FINFO(e.printState());
            if(e.id()>=0) id_list.push_back(e.id());
            if(e2.id()>=0) id_list.push_back(e.id());

            /*e.type() = "PhysicalObj";
             owl::ClassifyVector v = model.classify(e, 0.1);
            BOOST_FOREACH(skiros_msgs::WmTypeLikelihood msg, v)
            {
                FINFO(msg.type << ":" << msg.likelihood);
            }*/
            owl::IdentifyVector v = model.identify(e, 0);
            BOOST_FOREACH(skiros_msgs::WmObjLikelihood msg, v)
            {
                FINFO(msg.id << ":" << msg.likelihood);
            }
        } else if (c == 'r')
        {
            while(id_list.size())
            {
                model.removeElement(id_list.back());
                id_list.pop_back();
            }
        } else if (c == 'c') break;
	}


	spinner.stop();
	ros::shutdown();
	return 0;
} // main
