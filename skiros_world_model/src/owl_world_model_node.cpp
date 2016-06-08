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
#include "skiros_msgs/WmSetRelation.h"
#include "skiros_msgs/WmQuery.h"
#include "skiros_msgs/WoQuery.h"
#include "skiros_msgs/WoModify.h"
#include "skiros_msgs/WmElementGet.h"
#include "skiros_msgs/WmElementModify.h"
#include "skiros_msgs/WmMonitor.h"
#include "skiros_msgs/WmIdentify.h"
#include "skiros_msgs/WmClassify.h"
#include "skiros_msgs/WmSceneLoadAndSave.h"
#include "skiros_msgs/SetBool.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/utility.h"
#include "skiros_world_model/owl_world_ontology.h"
#include "skiros_world_model/owl_world_model.h"
#include "skiros_config/param_types.h"
#include "skiros_config/node_names.h"
#include <boost/foreach.hpp>
#include "skiros_world_model/reasoners_loading_func.h"
#include "skiros_config/declared_uri.h"


using namespace skiros_wm;
using namespace skiros_wm::owl;
using namespace skiros_config;
using namespace skiros_config::owl;

//GLOBAL
std::string owl_ws;
bool verbose = false;
const char ONTOLOGY_SAVE_FILE[] = "learned_concepts.owl";
skiros_wm::owl::Ontology ontology;
skiros_wm::owl::Ontology learned_ont;
skiros_wm::owl::WorldModel model(ontology);
ros::Publisher monitor;

//Model/ontology access variables
typedef std::map<std::string, boost::shared_ptr<DiscreteReasoner> > ModulesMapType;
ModulesMapType loaded_reasoners;
bool ontology_modification_flag = false;
boost::mutex extern_wm_mutex;


/////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////////////////////////////////


// Save the ontology when necessary
void saveOntology()
{
    auto lock = model.startChange();
    ros::Duration waitTime(5.0);
    while(ros::ok())
    {
        lock->unlock();
        while(!ontology_modification_flag && ros::ok())
            ros::Duration(0.1).sleep();
        if(!ros::ok())
            break;
        lock->lock();
        learned_ont.saveOntology(owl_ws+ONTOLOGY_SAVE_FILE);
        ontology_modification_flag = false;
        lock->unlock();
        FINFO("[saveOntology] Saved file: " << owl_ws+ONTOLOGY_SAVE_FILE);
        waitTime.sleep();
        lock->lock();
    }
    //std::cout << "[saveOntology] Exit" << std::endl;
}

// Get keyboard input from user
void input()
{
    auto lock = model.startChange();
    while(ros::ok())
    {
        lock->unlock();
        std::cout << "Type:  'p'/'o' print the world/ontology. 't' toggle verbose print mode. 's'/'l' save/load a scene. 'r' remove an element. " << std::endl;
        std::cin.clear();
        //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        char c = 'n';
        //'s' save the world state. 'l' load a world state.
        std::cin >> c;
        if(!ros::ok())break;
        lock->lock();
        if (c == 'p') //Print the world state
        {
            std::string save_path = model.getWorkspacePath() + "instance_log.txt";
            FILE * f = fopen(save_path.c_str(), "w");

            std::string output = model.getSceneTree(verbose, relation::Str[relation::sceneProperty]);
            std::cout << output << std::endl;
            if(f)
                fputs(output.c_str(), f);
            else
                FERROR("Failed to open log file.");
            fclose(f);
        } else if (c == 'o') //Print the ontology
        {
            ontology.printOntology();
        } else if (c == 't') //Toggle output verbosity
        {
            verbose = !verbose;
        } else if (c == 'r') // Remove an element
        {
            std::cout << "Insert the id of the element to remove: " << std::endl;
            int id;
            std::cin >> id;
            model.removeElement(id);
        } else if (c == 's') //Save world istance
        {
            model.saveScene("my_world_state.xml");
        }  else if (c == 'l') //Load world istance
        {
            model.loadScene("my_world_state.xml");
            skiros_msgs::WmMonitor msg;
            msg.action = "loadScene";
            msg.author = ros::this_node::getName();
            monitor.publish(msg);
        } else if (c == 'z')
        {
            //librdf_query_results * results = ontology.query("SELECT ?x WHERE { ?x   <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> <http://www.w3.org/2002/07/owl#NamedIndividual> }");
            //FINFO("tranquillo");
            //FINFO(ontology.results2string(results));
        }else if (c == 'x')
        {
            //std::string results = ontology.queryAsString("SELECT ?x    { ?x   <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> <http://www.w3.org/2002/07/owl#NamedIndividual> }");
            //FINFO(results);
            //model.queryTest("SELECT ?x    { ?x   <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> <http://www.w3.org/2002/07/owl#NamedIndividual> }");
        }
    }
    //std::cout << "[input] Exit" << std::endl;
}

void exeReasoner(std::string name)
{
    FINFO("Starting reasoner: " << name);
    loaded_reasoners[name]->execute(&model);
}

/////////////////////////////////////////////////////////////////////////////
// Query\Modify services
/////////////////////////////////////////////////////////////////////////////



bool lockUnlock(skiros_msgs::SetBoolRequest  &req,
         skiros_msgs::SetBoolResponse &res)
{
    if(req.set)
    {
        extern_wm_mutex.lock();
        FINFO("[lockUnlock] Locked by: " << req.author);
    }
    else
    {
        extern_wm_mutex.unlock();
        FINFO("[lockUnlock] Unlocked by: " << req.author);
    }
    res.success = true;
    return true;
}


//------------------------------------------------------------

bool queryOntology(skiros_msgs::WoQueryRequest  &req,
         skiros_msgs::WoQueryResponse &res)
{
    auto lock = model.startChange();
    res.answer =  ontology.queryAsString(req.query_string.c_str(), req.cut_prefix);
    FINFO("[Relation query] - Answer: \n" << res.answer);
    return true;
}



bool modifyOntology(skiros_msgs::WoModifyRequest  &req,
         skiros_msgs::WoModifyResponse &res)

{
    skiros_msgs::WmMonitor msg;
    msg.author = req.author;
    if(req.action==req.ADD)
        msg.action = req.action; //ADD
    else
        msg.action = req.REMOVE; //REMOVE_CONTEXT and REMOVE
    msg.header.stamp = ros::Time::now();
    msg.statements = req.statements;
    auto lock = model.startChange();
    if(req.action == req.ADD)
    {
        for(skiros_msgs::WoStatement statement : req.statements)
        {
            learned_ont.addStatement(msgToOwlNode(statement.subject),
                                     msgToOwlNode(statement.predicate),
                                     msgToOwlNode(statement.object));
        }
    }
    else if(req.action == req.REMOVE)
    {
        for(skiros_msgs::WoStatement statement : req.statements)
        {
            learned_ont.removeStatement(msgToOwlNode(statement.subject),
                                        msgToOwlNode(statement.predicate),
                                        msgToOwlNode(statement.object));
        }
    }
    else if(req.action == req.REMOVE_CONTEXT)
    {
        for(skiros_msgs::WoStatement statement : req.statements)
        {
            //Remove any hold definition
            if(!learned_ont.removeContextStatements(msgToOwlNode(statement.subject)) ||
                    !ontology.removeContextStatements(msgToOwlNode(statement.subject)))
            {
                FWARN("Error while removing element " << statement.subject.uri);
                res.result = -1;
                return true;
            }
        }
    }
    model.importOntology(learned_ont);
    ontology_modification_flag = true;
    monitor.publish(msg);
    FINFO("[Ontology modify srv] - " << req.action.c_str() << " " << req.statements.size() << " statement(s) " << " by " << req.author);
    res.result = 1;
    return true;
}

//------------------------------------------------------------


bool queryModel(skiros_msgs::WmQueryRequest  &req,
         skiros_msgs::WmQueryResponse &res)
{
    auto lock = model.startChange();
    //Skiros query
    RelationsVector matches = model.findStatements(req.relation.subject_id, req.relation.predicate, req.relation.object_id);
    for(RelationType rel : matches)
    {
      res.matches.push_back(relation2msg(rel));
    }
    FDEBUG("[Relation query] - " << req.relation.subject_id <<  req.relation.predicate << req.relation.object_id << " returned " <<  res.matches.size() << " match(es) ");
    return true;
}


//-------------------------------------------------------------


bool setRelation(skiros_msgs::WmSetRelationRequest  &req,
         skiros_msgs::WmSetRelationResponse &res)
{
  if(req.relation.subject_id<0 || req.relation.object_id<0)
  {
      FERROR("[Set Relation] - Invalid statement: " << req.relation.subject_id <<  req.relation.predicate << req.relation.object_id << req.set ? " True" : " False");
      res.return_code = -1;
      return true;
  }
  auto lock = model.startChange();
  if(!req.set)
  {
       //FERROR("Cannot remove a contain relation directly. ");//TODO: this is a constraint. Where should we collect constraints?
      res.return_code = model.removeStatement(req.relation.subject_id, req.relation.predicate, req.relation.object_id);
  }
  else
  {
      res.return_code = model.addStatement(req.relation.subject_id, req.relation.predicate, req.relation.object_id);
  }
  //TODO: publish in the monitor also the change of relations
  FDEBUG("[Set Relation] - " << req.relation.subject_id <<  req.relation.predicate << req.relation.object_id << req.set ? " True" : " False"  );
  return true;
}

//-------------------------------------------------------------


bool elementGet(skiros_msgs::WmElementGetRequest  &req,
            skiros_msgs::WmElementGetResponse &res)
{
  if(req.action == "getDefault")
  {
      skiros_wm::Element e;
      try
      {
          auto lock = model.startChange();
          e = model.getDefaultElement(req.e.type);
          lock->unlock();
          skiros_msgs::WmElement to_send = element2msg(e);
          res.element_list.push_back(to_send);
      }
      catch(std::invalid_argument err)
      {
          FWARN("[Element getDefault]: " << err.what());
      }
  }
  else if(req.action == "getByID")
  {
      auto lock = model.startChange();
      skiros_wm::Element e = model.getElement(req.e.id);
      lock->unlock();
      skiros_msgs::WmElement to_send = element2msg(e);
      res.element_list.push_back(to_send);
  }
  else if(req.action == "resolve")
  {
      skiros_wm::Element e = msg2element(req.e);
      auto lock = model.startChange();
      std::vector<skiros_wm::Element> match_list = model.resolveElement(e);
      lock->unlock();
      std::stringstream ss;
      ss << "[Element resolve] " << e.type() << " matches " << match_list.size() << " elements: ";
      //Convert to msg
      BOOST_FOREACH(skiros_wm::Element match, match_list)
      {
          ss << match.label() << " ";
          res.element_list.push_back(element2msg(match));
      }
      FDEBUG(ss.str());
  }
  else if(req.action == "getBranch")
  {
      auto lock = model.startChange();
      skiros_wm::Element e = model.getElement(req.e.id);
      skiros_wm::ExportedGraphType pair = model.getBranch(e, req.relation_filter, req.type_filter);
      lock->unlock();
      res.element_list = elements2msgs(pair.first);
      res.arc_list = relations2msgs(pair.second);
  }
  else if(req.action == "getChilds")
  {
      auto lock = model.startChange();
      skiros_wm::Element e = model.getElement(req.e.id);
      std::list<ChildsPair> childs = model.getChilds(e, req.relation_filter, req.type_filter);
      lock->unlock();
      BOOST_FOREACH(ChildsPair pair, childs)
      {
        res.element_list.push_back(element2msg(pair.second));
        res.arc_list.push_back(relation2msg(RelationType(req.e.id, pair.first, pair.second.id())));
      }
  }
  else
  {
      FERROR("Action '"<< req.action << "'' is not a valid input.");
      return false;
  }
  FDEBUG("[Element Query] - " << req.action << " returned " << res.element_list.size() << " elements. ");
  return true;
}



//-------------------------------------------------------------
bool elementModify(skiros_msgs::WmElementModifyRequest  &req,
            skiros_msgs::WmElementModifyResponse &res)
{
  skiros_wm::Element e = msg2element(req.e);
  e.lastUpdate() = ros::Time::now();//TODO: think carefully about how to handle the time stamps
  //ROS_INFO_STREAM(e.printState());
  //ontology.parseElementModify(req.action, e, req.parent_id);
  //Msg for the monitor topic
  skiros_msgs::WmMonitor msg;
  msg.author = req.author;
  msg.action = req.action;
  msg.header.stamp = ros::Time::now();
  auto lock = model.startChange();

  RelationsVector relations;

  if(req.action == "add")
  {
      for(skiros_msgs::WmRelation rel : req.relations)
      {
          relations.push_back(msg2relation(rel));
      }
      res.return_code = model.addElement(e, relations);
  }
  else if(req.action == "remove")
  {
      e = model.getElement(e.id());
      if(e.id()>-1)
      {
          res.return_code = model.removeElement(e.id());
      }
  }
  else if(req.action == "update")
  {
      for(skiros_msgs::WmRelation rel : req.relations)
      {
          relations.push_back(msg2relation(rel));
      }
      res.return_code = model.updateElement(e, relations);
  }
  else
  {
        FERROR("[elementModify] Unknown action requested.")
        return false;
  }
  //FINFO(model.getSceneTree(verbose));
  msg.element = element2msg(e);
  monitor.publish(msg);
  FINFO("[Element modify srv] - " << msg.action.c_str() << " element " << msg.element.label.c_str() << " type " <<  msg.element.type.c_str() << " by " << msg.author);
  return true;
}

//-------------------------------------------------------------

bool sceneLoadSave(skiros_msgs::WmSceneLoadAndSaveRequest  &req,
            skiros_msgs::WmSceneLoadAndSaveResponse &res)
{
    auto lock = model.startChange();
    std::string path;
    ros::NodeHandle skiros_nh(skiros_namespace);
    skiros_nh.param<std::string>(skiros_config::scene_workspace, path, skiros_common::utility::getSkirosSaveDirectory()+"scenes/");
    if(req.action == req.LOAD)
    {
        res.ok = false;
        if(model.loadScene(path+req.filename))
        {
            skiros_nh.setParam(skiros_config::scene_name, req.filename);
            skiros_msgs::WmMonitor msg;
            msg.action = "loadScene";
            msg.author = ros::this_node::getName();
            monitor.publish(msg);
            res.ok = true;
        }
    }
    else
    {
        res.ok = model.saveScene(path+req.filename);
    }
    return true;
}

//-------------------------------------------------------------

bool classify(skiros_msgs::WmClassifyRequest  &req,
            skiros_msgs::WmClassifyResponse &res)
{
  FINFO("[Element classification srv] - Start.");
  skiros_wm::Element e = msg2element(req.e);
  auto lock = model.startChange();
  res.matches = model.classify(e, req.threshold);
  FINFO("[Element classification srv] - Found " << res.matches.size() << " match(es).");
  return true;
}

//-------------------------------------------------------------
bool identify(skiros_msgs::WmIdentifyRequest  &req,
            skiros_msgs::WmIdentifyResponse &res)
{
  FINFO("[Element identification srv] - Start.");
  skiros_wm::Element e = msg2element(req.e);
  auto lock = model.startChange();
  res.matches = model.identify(e, req.parent_id);
  FINFO("[Element identification srv] - Found " << res.matches.size() << " match(es).");
  return true;
}

//-------------------------------------------------------------

void generateLearnedConceptsFile(std::string file)
{
    FILE * f = fopen(file.c_str(), "w");
    if(!f)throw std::runtime_error("[generateLearnedConceptsFile] Couldn't create the file.");

    fputs (    "<?xml version=\"1.0\"?>\n<!DOCTYPE rdf:RDF [\n"
                   "<!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n"
                   "<!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\" >\n"
                   "<!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\" >\n"
                   "<!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n"
              " ]>\n"


              " <rdf:RDF xmlns=\"http://www.semanticweb.org/francesco/ontologies/2015/8/learned-concepts#\"\n"
                    "xml:base=\"http://www.semanticweb.org/francesco/ontologies/2015/8/learned-concepts\"\n"
                    "xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\"\n"
                    "xmlns:owl=\"http://www.w3.org/2002/07/owl#\"\n"
                    "xmlns:xsd=\"http://www.w3.org/2001/XMLSchema#\"\n"
                    "xmlns:rdfs=\"http://www.w3.org/2000/01/rdf-schema#\">\n"
                   "<owl:Ontology rdf:about=\"http://www.semanticweb.org/francesco/ontologies/2015/8/learned-concepts\">\n"
                       "<owl:imports rdf:resource=\"http://www.semanticweb.org/francesco/ontologies/2014/9/stamina\"/>\n"
                   "</owl:Ontology>\n"
               "</rdf:RDF>\n",f);
    fclose (f);
}

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////



int main (int argc, char **argv)
{
    ///ROS INIT
    ros::init(argc, argv, world_model_node_name);
    ros::NodeHandle nh;
    ros::NodeHandle skiros_nh(skiros_namespace);
    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    // Use multiple threads to handle callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ///SAFETY CHECKS
    //check if the directory for the DB is created
    std::string db_path = ontology.getWorkspacePath()+"db";
    if (!(boost::filesystem::exists(db_path)))
    {
      FINFO("Skiros database directory does not exist, creating it");
      boost::filesystem::path dir(db_path);
      if (!(boost::filesystem::create_directory(db_path))) FERROR("Could not create folder at " << db_path << "!");
    }
    //check if the directory for the scenes is created
    skiros_nh.param<std::string>(skiros_config::scene_workspace, db_path, skiros_common::utility::getSkirosSaveDirectory()+"scenes/");
    if (!(boost::filesystem::exists(db_path)))
    {
      FINFO("Skiros scenes directory does not exist, creating it");
      boost::filesystem::path dir(db_path);
      if (!(boost::filesystem::create_directory(db_path))) FERROR("Could not create folder at " << db_path << "!");
    }
    skiros_nh.param<std::string>(skiros_config::owl_workspace, owl_ws, ontology.getWorkspacePath());
    //check if the owl workspace folder is located
    std::string owl_path = owl_ws;
    if (!(boost::filesystem::exists(owl_path)))
    {
      FINFO("OWL directory does not exist, creating it");
      boost::filesystem::path dir(owl_path);
      if (!(boost::filesystem::create_directory(owl_path))) FERROR("Could not create folder at " << owl_path << "!");
    }
    //check if the owl learned_concept file exists
    std::string file_name = owl_path+"/"+ONTOLOGY_SAVE_FILE;
    if(!(boost::filesystem::exists(file_name)))
    {
        FINFO("OWL file does not exist, creating it: " << file_name);
        generateLearnedConceptsFile(file_name);
    }

    ///Load the ontology
    std::string scene_name = "no_name";
    while(ros::ok() && !ontology.isInitialized())
    {
        std::string main_ontology;bool first_try = true;
        std::vector<std::string> all_files = skiros_common::utility::getFilesInFolder(owl_ws, ".owl");
        //Get the name of the main ontology
        if(argc > 1 && first_try) {main_ontology = argv[1];first_try = false;}
        else main_ontology = ontology.getWorkspacePath()+"stamina.owl";
        //Loads the main ontology
        try
        {
            model.loadMainOntology(main_ontology, scene_name);
            learned_ont.init(ontology.getDefaultUri(), "learned_concepts", ontology.getPrefixMap());
            FINFO("Main ontology: " << main_ontology << ", loaded successfully.");
        }
        catch(std::runtime_error e)
        {FERROR(e.what());}
        //Loads all other .owl files found as sub ontologies
        for(std::string file : all_files)
        {
            try
            {
                if(main_ontology.find(file)==std::string::npos)
                {
                    model.loadSubOntology(owl_ws+file);
                    if(file.find("learned_concepts")!=std::string::npos) //Only the learned_concept.owl file is imported in learned_ont
                        learned_ont.loadSubOntology(owl_ws+file);
                    FINFO("Sub ontology: " << owl_ws+file << ", loaded successfully.");
                }
            }
            catch(std::runtime_error e)
            {FERROR(e.what());}
        }
    }
    if(!ontology.isInitialized()) return -1;

    ///If has input, load a scene
    std::string scene_file_name;
    skiros_nh.param<std::string>(skiros_config::scene_name, scene_file_name, "");
    if(scene_file_name!="")
        if(!model.loadScene(db_path+scene_file_name))
            FERROR("Failed to load scene: " << scene_file_name);


    ///Create services
    monitor = nh.advertise<skiros_msgs::WmMonitor>(ros::this_node::getName()+"/monitor", 5);
    ros::ServiceServer query_ontology_ = nh.advertiseService(ros::this_node::getName()+"/query_ontology", queryOntology);
    ros::ServiceServer modify_ontology = nh.advertiseService(ros::this_node::getName()+"/modify_ontology", modifyOntology);
    ros::ServiceServer query_model_ = nh.advertiseService(ros::this_node::getName()+"/query_model", queryModel);
    ros::ServiceServer set_relation = nh.advertiseService(ros::this_node::getName()+"/set_relation", setRelation);
    ros::ServiceServer element_get = nh.advertiseService(ros::this_node::getName()+"/element_get", elementGet);
    ros::ServiceServer element_modify = nh.advertiseService(ros::this_node::getName()+"/element_modify", elementModify);
    ros::ServiceServer element_classify = nh.advertiseService(ros::this_node::getName()+"/classify", classify);
    ros::ServiceServer element_identify = nh.advertiseService(ros::this_node::getName()+"/identify", identify);
    ros::ServiceServer save_load = nh.advertiseService(ros::this_node::getName()+"/scene_load_and_save", sceneLoadSave);
    ros::ServiceServer lock_unlock = nh.advertiseService(ros::this_node::getName()+"/lock_unlock", lockUnlock);

    ///Start sub-threads
    boost::thread_group threadpool;
    //Start the keyboard input
    threadpool.create_thread(input);
    //Start the ontology saver
    threadpool.create_thread(saveOntology);

    ///Start reasoners sub-threads
    pluginlib::ClassLoader<DiscreteReasoner> * interfaces_loader = new pluginlib::ClassLoader<DiscreteReasoner>("skiros_world_model", "skiros_wm::DiscreteReasoner");

    std::vector<std::string> reasoner_list = interfaces_loader->getDeclaredClasses();

    for(std::string reasoner : reasoner_list)
    {
        try
        {
            loaded_reasoners[reasoner] = interfaces_loader->createInstance(reasoner);
            threadpool.create_thread(boost::bind(exeReasoner, reasoner));
        }
        catch(pluginlib::PluginlibException& ex)
        {
            FERROR("[WorldModel]The reasoner " << reasoner << " failed to load. Error: " << ex.what());
        }
    }

    ///Configure the logger (based on ROS param value)
    bool save_log;
    skiros_nh.param(skiros_config::save_log, save_log, false);
    skiros_common::InitLogger("skiros_world_model", save_log, save_log, save_log);

    ///Infinite ROS queue spin
    ros::spin();

    ///Unload
    //std::cout << "Lets close" << std::endl;
    spinner.stop();
    threadpool.interrupt_all();
    ros::Duration(0.1).sleep();
    ros::shutdown();
	return 0;
} // main
