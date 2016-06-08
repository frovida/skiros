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
#include <skiros_world_model/world_model_interface.h>
#include <skiros_config/param_types.h>
#include <skiros_config/declared_uri.h>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

using namespace skiros_wm;
using namespace skiros_config;
using namespace skiros_config::owl;
using namespace std;

boost::shared_ptr<skiros_wm::WorldModelInterface> wm_ptr;

/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////

boost::thread_group threadpool;

struct TestResult
{
    TestResult(std::string name_in, double mean_time_in, int it_in) : name(name_in), mean_time(mean_time_in), iterations(it_in) {}
    std::string name;
    double mean_time;
    int iterations;
};

//Test add-remove individuals from ontology
void removePermanentTest()
{
    FINFO("=======TESTING ONTOLOGY EDIT========");
    Element e(concept::GraspingPose);
    e.label() = "test";
    int i = 10;
    e.addProperty("TestProp", i);
    std::string query_string = "SELECT ?x WHERE { ?x rdf:type stmn:GraspingPose. } ";
    while(i>0)
    {
        FINFO("STORING: " << i);
        wm_ptr->addIndividual(e);
        i--;
        e.properties("TestProp").setValue(i);
        std::stringstream ss(wm_ptr->queryOntology(query_string));
        std::list<std::string> set1;
        {
            std::string temp;
            ss >> temp;
            while(!ss.eof())
            {
                set1.push_back(temp);
                FINFO(temp);
                ss >> temp;
            }
        }
        Element temp = wm_ptr->getDefaultElement("test");
        FINFO(temp.printState("", true));
        sleep(2);
    }
    std::stringstream ss(wm_ptr->queryOntology(query_string));
    std::list<std::string> set1;
    {
        std::string temp;
        ss >> temp;
        while(!ss.eof())
        {
            set1.push_back(temp);
            FDEBUG(temp);
            ss >> temp;
        }
    }
}

//Test speed of interactions with world model
void testingThread(boost::barrier & barrier)
{
    double average_time;int iterations;ros::Time start_time;
    std::vector<TestResult> tests;
    std::string query_string;
    FINFO("=======TESTING QUERY========");
    average_time = 0; iterations = 0;
    query_string = "SELECT ?x WHERE { ?x rdfs:subClassOf stmn:Product. } ";
    start_time = ros::Time::now();
    std::stringstream ss(wm_ptr->queryOntology(query_string));
    average_time += (ros::Time::now() - start_time).toSec();
    iterations++;
    average_time = average_time/iterations;
    tests.push_back(TestResult("Query", average_time, iterations));
    std::set<std::string> set1, set2;
    {
        std::string temp;
        ss >> temp;
        while(!ss.eof())
        {
            set1.insert(temp);
            FDEBUG(temp);
            ss >> temp;
        }
    }

    BOOST_FOREACH(std::string name, set1)
    {
        query_string = "SELECT ?x WHERE { ?x rdf:type stmn:" + name + ". } ";
        std::stringstream ss(wm_ptr->queryOntology(query_string));
        {
            std::string temp;
            ss >> temp;
            while(!ss.eof())
            {
                set2.insert(temp);
                FDEBUG(temp);
                ss >> temp;
            }
        }
    }

    Element surface(concept::Str[concept::Surface]);
    vector<Element> surfaces = wm_ptr->resolveElement(surface);
    if(surfaces.size()>0)
    {
        surface = surfaces[0];
        wm_ptr->updateElement(surface);
    }
    else
        wm_ptr->addElement(surface, 0, relation::Str[relation::contain]);
    vector<Element> v;

    barrier.wait();

    FINFO("=======ADDING DEFAULT PRODUCTS=======");

    average_time = 0; iterations = 0;
    BOOST_FOREACH(std::string name, set2)
    {
        start_time = ros::Time::now();
        Element e = wm_ptr->getDefaultElement(name);
        if(e.label().find("-")==std::string::npos)
        {
            wm_ptr->addElement(e, surface.id(), relation::Str[relation::contain]);
            v.push_back(e);
            average_time += (ros::Time::now() - start_time).toSec();
            iterations++;
        }
    }
    average_time = average_time/iterations;
    tests.push_back(TestResult("AddElement", average_time, iterations));

    barrier.wait();
    FINFO("=======STORING ELEMENTS IN ONTOLOGY=======");
    average_time = 0; iterations = 0;
    BOOST_FOREACH(Element e, v)
    {
        start_time = ros::Time::now();
        wm_ptr->addIndividual(e);
        average_time += (ros::Time::now() - start_time).toSec();
        iterations++;
    }
    average_time = average_time/iterations;
    tests.push_back(TestResult("StoreElementPermanet", average_time, iterations));


    barrier.wait();
    FINFO("=======TESTING OTHER FUNCTIONS (Classify, Identify, Object Update) =======");
    FDEBUG("Classify test");
    average_time = 0; iterations = 0;
    BOOST_FOREACH(Element e, v)
    {
        start_time = ros::Time::now();
        ClassLkhoodListType listc = wm_ptr->classify(e, 0);
        average_time += (ros::Time::now() - start_time).toSec();
        iterations++;
        typedef pair<string, double> LkPair;
        BOOST_FOREACH(LkPair p, listc)
        {
            FDEBUG(p.first << p.second);
        }
    }
    average_time = average_time/iterations;
    tests.push_back(TestResult("Classify", average_time, iterations));
    FDEBUG("Identify test");
    average_time = 0; iterations = 0;
    BOOST_FOREACH(Element e, v)
    {
        start_time = ros::Time::now();
        IdLkhoodListType listid = wm_ptr->identify(e, 0, 0);
        average_time += (ros::Time::now() - start_time).toSec();
        iterations++;
        typedef pair<int, double> IdPair;
        BOOST_FOREACH(IdPair p, listid)
        {
            FDEBUG(p.first << p.second);
        }
    }
    average_time = average_time/iterations;
    tests.push_back(TestResult("Identify", average_time, iterations));
    FDEBUG("updateElement test");
    average_time = 0; iterations = 0;
    BOOST_FOREACH(Element e, v)
    {
        int random = rand() % v.back().id() + (v.back().id()- v.size());
        FDEBUG("Move object " << e.id() << " to random position: " << random);
        start_time = ros::Time::now();
        wm_ptr->updateElement(e, random);
        average_time += (ros::Time::now() - start_time).toSec();
        iterations++;
    }
    average_time = average_time/iterations;
    tests.push_back(TestResult("updateElement", average_time, iterations));

    barrier.wait();

    /*FINFO("=======REMOVING ELEMENTS IN ONTOLOGY=======");
    BOOST_FOREACH(Element e, v)
    {
        wm_ptr->removeElementPermanently(e);
    }*/
    /*FINFO("=======REMOVING DEFAULT PRODUCTS=======");
    average_time = 0; iterations = 0;
    while(!v.empty())
    {
        start_time = ros::Time::now();
        Element e = v.back();
        wm_ptr->removeElement(e.id());
        v.pop_back();
        average_time += (ros::Time::now() - start_time).toSec();
        iterations++;
    }
    FINFO("Removed " << iterations << " elements. Average time: " << average_time);*/

    BOOST_FOREACH(TestResult result, tests)
    {
        barrier.wait();
        FINFO(result.name << " tested " << result.iterations << " times with average time: " << result.mean_time << " s ");
    }
}

// Get keyboard input from user
void input()
{
    while(ros::ok())
    {
        std::cout << "Type: 't' - generic test, 'r' - test ontology remove " << std::endl;
        std::cin.clear();
        //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        char c = 'n';
        std::cin >> c;
        if (c == 't') //test
        {
            std::cout << "Type the number of working threads (simulating concurrent access to wm) " << std::endl;
            int num_of_threads;
            std::cin >> num_of_threads;
            boost::barrier barrier(num_of_threads);
            for(int i=0; i<num_of_threads;i++)
                threadpool.create_thread(boost::bind(&testingThread, boost::ref(barrier)));
            threadpool.join_all();
        }
        else if (c == 'r')
        {
            removePermanentTest();
        }
    }
}



/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////


int main (int argc, char **argv)
{
	// Initialise the ros node
    ros::init(argc, argv, "test_world_model"); //ros::this_node::getName()
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;

    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

    // Istanciate a skill manager class
    wm_ptr.reset(new skiros_wm::WorldModelInterface(nh));

    //Wait the world model to be up and register the robot presence
    if(!wm_ptr->isConnected()) FWARN("World model seems down, waiting...")
    while(!wm_ptr->isConnected() && ros::ok()) {ros::spinOnce();}
    FINFO("Connected to world model");

    //Start the keyboard input
    boost::thread get_input(input);

    //Infinite ROS queue spin
    ros::spin();
	ros::shutdown();
	return 0;
} // main
