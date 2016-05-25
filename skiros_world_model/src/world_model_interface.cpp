#include "redland.h"
#include "skiros_common/logger_sys.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_world_model/owl_world_model.h"
#include "skiros_world_model/utility.h"
#include "skiros_common/utility.h"
#include "skiros_msgs/WmRelation.h"
#include "skiros_msgs/WmElementGet.h"
#include "skiros_msgs/WmQuery.h"
#include "skiros_msgs/WoQuery.h"
#include "skiros_msgs/WmElementModify.h"
#include "skiros_msgs/WmSetRelation.h"
#include "skiros_msgs/WmIdentify.h"
#include "skiros_msgs/WmClassify.h"
#include "skiros_msgs/SetBool.h"
#include "skiros_msgs/WoModify.h"
#include "boost/foreach.hpp"
#include "skiros_config/declared_uri.h"
#include "skiros_config/node_names.h"
#include "skiros_config/param_types.h"
#include "skiros_world_model/reasoners_loading_func.h"

using namespace skiros_config::owl;
using namespace skiros_config;
using namespace skiros_msgs;
using namespace skiros_wm::owl;


namespace skiros_wm
{

//------------------------------------------ Utils -----------------------------------------------

WorldModelInterface::WorldModelInterface(ros::NodeHandle nh, bool monitor_changes): OntologyInterface(nh), new_changes_(true)
{
    //Foundamental initialization necessary to avoid problems with serialization of params (see "skiros_config/param_types.h" for more details)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();

    query_model_      = nh_.serviceClient<skiros_msgs::WmQuery>(std::string(world_model_node_name)+wm_query_mdl_srv_name);
    set_relation_     = nh_.serviceClient<skiros_msgs::WmSetRelation>(std::string(world_model_node_name)+wm_set_rel_srv_name);
    element_get_      = nh_.serviceClient<skiros_msgs::WmElementGet>(std::string(world_model_node_name)+wm_get_elm_srv_name);
    element_modify_   = nh_.serviceClient<skiros_msgs::WmElementModify>(std::string(world_model_node_name)+wm_mdf_elm_srv_name);
    element_classify_ = nh_.serviceClient<skiros_msgs::WmClassify>(std::string(world_model_node_name)+wm_classify_srv_name);
    element_identify_ = nh_.serviceClient<skiros_msgs::WmIdentify>(std::string(world_model_node_name)+wm_identify_srv_name);
    lock_unlock_ = nh_.serviceClient<skiros_msgs::SetBool>(std::string(world_model_node_name)+wm_lock_unlock_srv_name);
    if(monitor_changes)
        wm_monitor_sub_ = nh_.subscribe(std::string(world_model_node_name)+wm_monitor_tpc_name, 10, &WorldModelInterface::wmMonitorCB,  this);
    //Test connection
    if(!isConnected()) FWARN("[WorldModelInterface] The world model is not connected.");
    robot_registered_id_ = 0;
}

void WorldModelInterface::lock()
{
    skiros_msgs::SetBool msg;
    msg.request.set = true;
    msg.request.author = ros::this_node::getName();
    if(!lock_unlock_.call(msg))
    {
        this->connectionFailed("lock");
    }
}

void WorldModelInterface::unlock()
{
    skiros_msgs::SetBool msg;
    msg.request.set = false;
    msg.request.author = ros::this_node::getName();
    if(!lock_unlock_.call(msg))
    {
        this->connectionFailed("unlock");
    }
}

bool WorldModelInterface::isConnected()
{
   if(element_get_.exists()) connected_ = true;
    else  connected_ = false;
   return connected_;
}

bool WorldModelInterface::waitConnection(ros::Duration timeout)
{
    ros::Time start_time = ros::Time::now();
    while(!isConnected() && ros::Time::now()-start_time<=timeout)
    {
        ros::Duration(0.1).sleep();
    }
    return isConnected();
}

void OntologyInterface::connectionFailed(std::string service_name)
{
    std::stringstream ss;
    ss << "[WorldModelInterface::"<< service_name << "] ROS connection to world model failed.";
    FWARN(ss.str());
    connected_ = false;
    throw std::runtime_error(ss.str()); //TODO: change to waiting routine...
}

//--------------------------------- Ontology methods -------------------------------------

OntologyInterface::OntologyInterface(ros::NodeHandle nh) : nh_(nh)
{
    query_ontology_  = nh_.serviceClient<skiros_msgs::WoQuery>(std::string(world_model_node_name)+wm_query_ont_srv_name);
    modify_ontology_ = nh_.serviceClient<skiros_msgs::WoModify>(std::string(world_model_node_name)+wm_modify_ont_srv_name);
}

void OntologyInterface::addStdPrefix(std::string & uri)
{
    if(uri.find(':')==std::string::npos)
        uri = "stmn:"+uri;
}

std::string OntologyInterface::queryOntology(std::string query_string, bool cut_prefix)
{
    //Prepare the service message
    skiros_msgs::WoQuery msg;
    msg.request.query_string = query_string;
    msg.request.cut_prefix = cut_prefix;
    //Call the service
    if(query_ontology_.call(msg))
    {
        return msg.response.answer;
    }
    else
    {
        this->connectionFailed("query");
        return std::string();
    }
}

int OntologyInterface::addIndividual(Element e)
{
    //Remove completely old definition
    removeIndividual(e);
    //Create the statements
    WoModify msg;
    WoStatement temp;
    msg.request.action = msg.request.ADD;
    msg.request.author = ros::this_node::getName();
    //Register the individual type
    temp.subject.uri = e.label();
    temp.subject.type = temp.subject.RESOURCE;
    temp.predicate.uri = std_uri::TYPE;
    temp.predicate.type = temp.subject.RESOURCE;
    temp.object.uri = e.type();
    temp.object.type = temp.subject.RESOURCE;
    msg.request.statements.push_back(temp);
    //Register a new individual
    temp.object.uri = std_uri::OWL_INDIVIDUAL;
    msg.request.statements.push_back(temp);
    //Save the properties stored by SpatialReasoners
    if(e.hasProperty(data::DiscreteReasoner))
    {
        std::vector<std::string> v = e.properties(data::DiscreteReasoner).getValues<std::string>();
        for(std::string r : v)
        {
            skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner(e.properties(r).getValue<std::string>());
            ReasonerDataMap data = reasoner->extractOwlData(e);
            for(ReasonerDataMap::value_type pair : data)
            {
                temp.predicate.uri = pair.first;
                temp.object.uri = e.type();
                temp.object.type = temp.subject.LITERAL;
                temp.object.literal_type = pair.second.second;
                msg.request.statements.push_back(temp);
            }
        }
    }
    //Save all the remaining properties
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    for(skiros_common::ParamMap::value_type pair : e.properties())
    {
        if(getType(pair.first).find("ObjectProperty")!=std::string::npos)
        {
            for(int i=0; i<pair.second.size();i++)
            {
                std::string value = pair.second.getValueStr(i);
                temp.predicate.uri = pair.first;
                temp.object.uri = value;
                temp.object.type = temp.subject.RESOURCE;
                msg.request.statements.push_back(temp);
            }
        }
        else
        {
            for(int i=0; i<pair.second.size();i++)
            {
                std::string value = pair.second.getValueStr(i);
                temp.predicate.uri = pair.first;
                temp.object.uri = value;
                temp.object.type = temp.subject.LITERAL;
                temp.object.literal_type = param_types.getDataTypeStr(pair.second);
                msg.request.statements.push_back(temp);
            }
        }
    }
    //Call the service
    if(modify_ontology_.call(msg))
    {
        return msg.response.result;
    }
    else
    {
        this->connectionFailed("modify");
        return -100;
    }
}

int OntologyInterface::removeIndividual(Element e)
{
    WoModify msg;
    WoStatement temp;
    msg.request.action = msg.request.REMOVE_CONTEXT;
    msg.request.author = ros::this_node::getName();
    //Register the individual type
    temp.subject.uri = e.label();
    temp.subject.type = temp.subject.RESOURCE;
    temp.predicate.uri = std_uri::TYPE;
    temp.predicate.type = temp.subject.RESOURCE;
    temp.object.uri = e.type();
    temp.object.type = temp.subject.RESOURCE;
    msg.request.statements.push_back(temp);
    //Call the service
    if(modify_ontology_.call(msg))
    {
        return msg.response.result;
    }
    else
    {
        this->connectionFailed("modify");
        return -100;
    }
}

//TODO: make hierarchical
std::set<std::string> OntologyInterface::getSubClasses(std::string parent_class)
{
    addStdPrefix(parent_class);
    std::set<std::string> to_ret;
    std::stringstream ss;
    ss << "SELECT ?x WHERE { ?x rdfs:subClassOf " << parent_class << " . } ";
    return makeSet(queryOntology(ss.str()));
}

std::set<std::string> OntologyInterface::getSuperClasses(std::string sub_class)
{
    addStdPrefix(sub_class);
    std::set<std::string> to_ret;
    std::stringstream ss;
    ss << "SELECT ?x WHERE { " << sub_class << " rdfs:subClassOf ?x . } ";
    return makeSet(queryOntology(ss.str()));
}

std::set<std::string> OntologyInterface::getEquivalents(std::string individual, bool reverse)
{
    addStdPrefix(individual);
    std::set<std::string> to_ret;
    std::stringstream ss;
    if(reverse)
        ss << "SELECT ?x WHERE { " << individual << " owl:sameAs ?x . } ";
    else
        ss << "SELECT ?x WHERE { ?x owl:sameAs " << individual << " . } ";
    return makeSet(queryOntology(ss.str()));
}

std::set<std::string> OntologyInterface::makeSet(std::string query_result)
{
    std::stringstream ss(query_result);
    std::set<std::string> set;
    std::string temp;
    while(!ss.eof())
    {
        ss >> temp;
        if(temp!="")
            set.insert(temp);
    }
    return set;
}

//------------------------------------- Relation methods  ----------------------------------------

int WorldModelInterface::setRelation(int subject_id, std::string predicate, int object_id, bool set)
{
        //Prepare the service message
        skiros_msgs::WmSetRelation msg;
        msg.request.relation.subject_id = subject_id;
        msg.request.relation.predicate = predicate;
        msg.request.relation.object_id = object_id;
        msg.request.set = set;
        //Call the service
        if(set_relation_.call(msg))
        {
            return msg.response.return_code;
        }
        else
        {
            connectionFailed("set_relation");
            return 0;
        }
}

//Query the world model.
//Note: if the object is not defined the function gives back the ID of all elements matching the proposition.
//This implementation is limited.
//TODO: get back the TYPE of relations
RelationsVector WorldModelInterface::queryRelation(int subject_id, std::string predicate, int object_id)
{
        //Prepare the service message
        skiros_msgs::WmQuery msg;
        msg.request.relation.subject_id = subject_id;
        msg.request.relation.predicate = predicate;
        msg.request.relation.object_id = object_id;
        //Call the service
        if(query_model_.call(msg))
        {
            RelationsVector to_ret;
            BOOST_FOREACH(skiros_msgs::WmRelation rel, msg.response.matches )
            {
                to_ret.push_back(msg2relation(rel));
            }
            return to_ret;
        }
        else
        {
            connectionFailed("query");
            return RelationsVector();
        }
}


//------------------------------------------------------------------------------------------------------------

skiros_wm::Element WorldModelInterface::getParentElement(int id, std::string relation)
{
    if(relation=="")relation=relation::Str[relation::spatiallyRelated];
    RelationsVector matches = this->queryRelation(-1, relation, id);
    if(matches.size())
    {
        //FINFO("Parent of " << id << "is " << matches[0].subject_id());
        return this->getElement(matches[0].subject_id());
    }
    else
    {
        FERROR("[WorldModelInterface::getParentElement] No parent found for object id " << id);
        return skiros_wm::Element();
    }
}

//------------------------------------------------------------------------------------------------------------

std::vector<skiros_wm::Element> WorldModelInterface::getChildElements(int id, std::string relation, std::string type)
{
    std::vector<skiros_wm::Element> to_ret;
    skiros_msgs::WmElementGet msg;
    msg.request.e.id = id;
    msg.request.action = "getChilds";
    msg.request.relation_filter = relation;
    msg.request.type_filter = type;
    skiros_wm::Element e;
    if(element_get_.call(msg))
    {
        to_ret = msgs2elements(msg.response.element_list);
    }
    else
    {
        connectionFailed("element_get");
    }
    return to_ret;
}


//------------------------------------------------------------------------------------------------------------

std::vector<skiros_wm::Element> WorldModelInterface::getBranchElements(int id, std::string relation, std::string type)
{
     std::vector<skiros_wm::Element> to_ret;
     skiros_wm::Element temp;
     if(relation=="")relation=relation::Str[relation::spatiallyRelated];
     RelationsVector matches = this->queryRelation(id, relation, -1);
     if(matches.size())
     {
         BOOST_FOREACH(RelationType rel, matches)
         {
             temp = this->getElement(rel.object_id());
             if(temp.type()==type || temp.label()==type || type=="")
             {
                 to_ret.push_back(temp);
                 std::vector<skiros_wm::Element> temp_v = getBranchElements(temp.id(), relation, type);
                 to_ret.insert(to_ret.end(), temp_v.begin(), temp_v.end());
             }
         }
     }
     return to_ret;
}

//------------------------------------------------------------------------------------------------------------

void WorldModelInterface::getBranch(WorldGraph &graph, int root_id, std::string relation, std::string type)
{
     ros::Time start_time = ros::Time::now();
     skiros_msgs::WmElementGet msg;
     msg.request.e.id = root_id;
     msg.request.action = "getBranch";
     msg.request.relation_filter = relation;
     msg.request.type_filter = type;
     skiros_wm::Element e;
     if(element_get_.call(msg))
     {
         if(msg.response.element_list.size()>0)
         {
             graph.importGraph(msgs2elements(msg.response.element_list), msgs2relations(msg.response.arc_list), true);
         }
         //FINFO("[getElement] Executed in " << (ros::Time::now() - start_time).toSec()  << " second(s).");
     }
     else
     {
         connectionFailed("element_get");
     }
}

//------------------------------------------------------------------------------------------------------------


void WorldModelInterface::pushBranch(WorldGraph & graph, int root_id)
{
    //Init
    if(root_id==-1)
        root_id = graph.getRootId();
    skiros_wm::Element e = graph.getElement(root_id);
    //Add the root
    RelationType rel = graph.getParentRelation(root_id);
    if(graph.isGlobalElement(root_id))
        updateElement(e, rel.subject_id(), rel.predicate());
    else if(!graph.isGlobalElement(root_id) && rel.subject_id()>=0)
        addElement(e, rel.subject_id(), rel.predicate());
    else throw std::invalid_argument("[WorldModelInterface::pushBranch] The root node of the graph doesn't have connection with the world model.");
    //Raise the global flag (meaning the element is present in the shared world model)
    graph.setGlobalElement(root_id);
    //Goes recursively
    std::vector<ChildsPair> childs = graph.getChildElements(root_id);
    for(ChildsPair pair : childs)
    {
        pushBranch(graph, pair.second.id());
    }
}

//------------------------------------------------------------------------------------------------------------

skiros_wm::Element WorldModelInterface::getElement(int id)
{
      ros::Time start_time = ros::Time::now();
      skiros_msgs::WmElementGet msg;
      msg.request.e.id = id;
      msg.request.action = "getByID";
      skiros_wm::Element e;
      if(element_get_.call(msg))
      {
          if(msg.response.element_list.size()>0)
          {
            e = msg2element(msg.response.element_list[0]);
          }
          //FINFO("[getElement] Executed in " << (ros::Time::now() - start_time).toSec()  << " second(s).");
      }
      else
      {
          connectionFailed("element_get");
      }
      return e;
}

//------------------------------------------------------------------------------------------------------------

skiros_wm::Element WorldModelInterface::getElementFromUri(std::string uri)
{
     int id = -1;
     if(uri.find("-")==std::string::npos)
         return Element();
     uri = uri.substr(uri.find("-")+1);
     try
     {
         id = boost::lexical_cast<int>(uri);
     }
     catch(boost::bad_lexical_cast err)
     {
         FERROR("[WorldModel::uri2id] " << err.what());
         return Element();
     }
     return getElement(id);
}

//------------------------------------------------------------------------------------------------------------

bool WorldModelInterface::isElementUri(std::string uri)
{
    if(uri.find("-")==std::string::npos)
        return false;
    try
    {
        uri = uri.substr(uri.find("-")+1);
        int id = boost::lexical_cast<int>(uri);
    }
    catch(boost::bad_lexical_cast err)
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------------------------------------

//Return a vector of elements matching the given description.
std::vector<skiros_wm::Element> WorldModelInterface::resolveElement(skiros_wm::Element e)
{
      skiros_msgs::WmElementGet msg;
      msg.request.e = skiros_wm::element2msg(e);
      msg.request.action = "resolve";
      std::vector<skiros_wm::Element> ret;
      if(element_get_.call(msg))
      {
          if(msg.response.element_list.size()>0)
          {
              for(int i=0;i<msg.response.element_list.size();i++)
              {
                  skiros_wm::Element temp = msg2element(msg.response.element_list[i]);
                  //If a label is specified, filters the elements not corresponding
                  if(e.label()=="" || e.label()==temp.label()) ret.push_back(temp);
              }
          }
          return ret;
      }
      else
      {
          connectionFailed("resolveElement");
          return ret;
      }
}

//------------------------------------------------------------------------------------------------------------

void WorldModelInterface::removeBranch(int id)
{
    std::vector<skiros_wm::Element> childs = getChildElements(id);
    BOOST_FOREACH(skiros_wm::Element e, childs) removeElement(e.id());
    removeElement(id);
}

//------------------------------------------------------------------------------------------------------------

bool WorldModelInterface::removeElement(int id)
{
        skiros_msgs::WmElementModify msg;
        msg.request.author = ros::this_node::getName();
        msg.request.e.id = id;
        msg.request.action = "remove";
        if(element_modify_.call(msg))
        {
            if(msg.response.return_code > 0) return true;
            else return false;
        }
        else
        {
            connectionFailed("element_modify");
            return false;
        }
}

//------------------------------------------------------------------------------------------------------------

int WorldModelInterface::addElement(Element & e, RelationsVector relations)
{
        ros::Time start_time = ros::Time::now();
        skiros_msgs::WmElementModify msg;
        msg.request.author = ros::this_node::getName();
        msg.request.e = skiros_wm::element2msg(e);
        BOOST_FOREACH(RelationType rel, relations)
        {
            msg.request.relations.push_back(relation2msg(rel));
        }
        msg.request.action = "add";
        if(element_modify_.call(msg))
        {
            //FINFO("[addElement] Executed in " << (ros::Time::now() - start_time).toSec()  << " second(s).");
            if(msg.response.return_code >= 0) e.id() = msg.response.return_code; //Return the id assigned to the obj
            else FERROR("[addElement] Failed to add the element. Ret code: " << msg.response.return_code)
        }
        else
        {
            connectionFailed("element_modify");
        }
        return e.id();
}

//------------------------------------------------------------------------------------------------------------

int WorldModelInterface::updateElement(skiros_wm::Element e, int parent_id, std::string predicate)
{
        RelationsVector relations;
        if(parent_id!=-1)
        {
            if(predicate=="")predicate = relation::Str[relation::contain];
            relations.push_back(RelationType(parent_id, predicate, -1));
        }
        skiros_msgs::WmElementModify msg;
        msg.request.author = ros::this_node::getName();
        msg.request.e = skiros_wm::element2msg(e);
        msg.request.action = "update";
        BOOST_FOREACH(RelationType rel, relations)
        {
            msg.request.relations.push_back(relation2msg(rel));
        }
        if(element_modify_.call(msg))
        {
            if(msg.response.return_code > 0) return msg.response.return_code; //Return the id assigned to the obj
            else return -1;
        }
        else
        {
            connectionFailed("updateElement");
            return -1;
        }
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------ Element creation methods  ---------------------------------------

skiros_wm::Element WorldModelInterface::getDefaultElement(std::string individual) //TODO:make the second variable usable(now always true)
{
    skiros_msgs::WmElementGet msg;
    msg.request.e.type = individual;
    msg.request.action = "getDefault";
    skiros_wm::Element e;
    if(element_get_.call(msg))
    {
        if(msg.response.element_list.size()>0)
        {
          e = msg2element(msg.response.element_list[0]);
        }
        else throw std::invalid_argument(std::string("[WorldModelInterface::getDefaultElement] Type: " + individual + " not found. "));
        return e;
    }
    else
    {
        connectionFailed("getDefaultElement");
        return e;
    }
}

//------------------------------------------------------------------------------------------------------------

std::vector<std::pair<std::string, double> > WorldModelInterface::classify(skiros_wm::Element e, float threshold)
{
  skiros_msgs::WmClassify msg;
  msg.request.e = skiros_wm::element2msg(e);
  msg.request.threshold = threshold;
  std::vector<std::pair<std::string, double> > v;
  std::pair<std::string, double> temp;
  if(element_classify_.call(msg))
  {
        for(int i=0; i<msg.response.matches.size();i++)
        {
            temp.first = msg.response.matches[i].type;
            temp.second = msg.response.matches[i].likelihood;
            v.push_back(temp);
        }
        return v;
  }
  else
  {
      connectionFailed("classify");
      return v;
  }
}

//------------------------------------------------------------------------------------------------------------

std::vector<std::pair<int, double> > WorldModelInterface::identify(skiros_wm::Element e, int parent_id, float threshold)
{
  skiros_msgs::WmIdentify msg;
  msg.request.e = skiros_wm::element2msg(e);
  msg.request.parent_id = parent_id;
  std::vector<std::pair<int, double> > v;
  std::pair<int, double> temp;
  if(element_identify_.call(msg))
  {
      for(int i=0; i<msg.response.matches.size();i++)
      {
          temp.first = msg.response.matches[i].id;
          temp.second = msg.response.matches[i].likelihood;
          if(temp.second>threshold) v.push_back(temp);
      }
      return v;
  }
  else
  {
      connectionFailed("identify");
      return v;
  }
}

//------------------------------------------------------------------------------------------------------------

int WorldModelInterface::addBranch(skiros_wm::Element object, int parent_id, std::string relation)
{
    addElement(object, parent_id, relation);
    if(object.hasProperty("hasA"))
    {
        for(int i=0;i<object.properties("hasA").size();i++)
        {
            skiros_wm::Element e = getDefaultElement(object.properties("hasA").getValue<std::string>(i));
            addBranch(e, object.id(), "hasA");
        }
    }
    return object.id();
}


//------------------------------------------------------------------------------------------------------------

bool WorldModelInterface::hasChanged()
{
    if(wm_monitor_sub_==NULL)
        FWARN("[WorldModelInterface::hasChanged] The interface is not monitoring the world, this function is returning always true.");
    if(new_changes_)
    {
        new_changes_ = !new_changes_;
        return true;
    }
    else
        return false;
}

//------------------------------------------------------------------------------------------------------------

void WorldModelInterface::wmMonitorCB(const skiros_msgs::WmMonitor& msg)
{
    new_changes_ = true;
}



//-------------------------------------------------------------------------------------------------------
//---------------------------------------WorldModelInterfaceS-------------------------------------------
//-------------------------------------------------------------------------------------------------------

std::string WorldModelInterfaceS::getFirstFrameId(skiros_wm::Element e)
{
    skiros_wm::Element parent = e;
    while(!parent.hasProperty(data::Str[data::FrameId]) && parent.id() > 0) parent = getParentElement(parent);
    return parent.properties(data::Str[data::FrameId]).getValue<std::string>();
}

void WorldModelInterfaceS::startTfListener()
{
    tf_listener_.reset(new tf::TransformListener());
}

void WorldModelInterfaceS::stopTfListener()
{
    tf_listener_.reset();
}

bool WorldModelInterfaceS::waitForTransform(std::string target_frame, std::string  source_frame, ros::Time t, ros::Duration d)
{
    if(tf_listener_)
        return tf_listener_->waitForTransform(target_frame, source_frame, t, d);
    else
    {
        FERROR("[waitForTransform]Tf listener is not initialized.");
        return false;
    }
}

void WorldModelInterfaceS::lookupTransform(std::string  target_frame, std::string  source_frame, ros::Time  t, tf::StampedTransform & transform)
{
    if(tf_listener_) tf_listener_->lookupTransform(target_frame, source_frame, t, transform);
    else FERROR("[lookupTransform]Tf listener is not initialized.");
}


bool WorldModelInterfaceS::waitAndLookupTransform(std::string target_frame, std::string  source_frame, ros::Time t, tf::StampedTransform & transform, ros::Duration d)
{
    if(!waitForTransform(target_frame, source_frame, t, d))
        return false;
    lookupTransform(target_frame, source_frame, t, transform);
    return true;
}


bool WorldModelInterfaceS::waitAndLookupFirstTransform(std::string target_frame, std::string  source_frame, tf::StampedTransform & transform, ros::Duration d, ros::Duration interval)
{
    ros::Time start_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    while(!waitForTransform(target_frame, source_frame, now, interval))
    {
        now = ros::Time::now();
        if((now-start_time)>d)
            return false;
    }
    lookupTransform(target_frame, source_frame, now, transform);
    return true;
}

void WorldModelInterfaceS::transformPose(const std::string& target_frame, const tf::Stamped<tf::Pose>& stamped_in, tf::Stamped<tf::Pose>& stamped_out)
{
    if(tf_listener_) tf_listener_->transformPose(target_frame, stamped_in, stamped_out);
    else FERROR("[transformPose]Tf listener is not initialized.");
}

void WorldModelInterfaceS::addTfPrefix(skiros_wm::Element & e)
{
    //Updating the frameID info with tf prefix
    std::string temp = tf_listener_->getTFPrefix();
    if(temp!="" && e.hasProperty(data::Str[data::FrameId]))
    {
        std::string frameid = e.properties(data::Str[data::FrameId]).getValue<std::string>();
        if(frameid.find(temp)!=std::string::npos)return;
        frameid = temp+"/"+frameid;
        e.properties(data::Str[data::FrameId]).setValue(frameid);
    }
}
//------------------------------------------------------------------------------------------------------------

int WorldModelInterfaceS::registerRobot(skiros_wm::Element location, skiros_wm::Element robot)
{
    //Look if a robot with the same skill manager name is already in the world model. In case I don't register it again
    std::vector<skiros_wm::Element> robots = resolveElement(robot);
    for(skiros_wm::Element r : robots)
    {
        if(r.label()==robot.label())
        {
            FINFO("Another robot with name " << robot.label() << " found in the world model. Skipping registration.");
            robot_registered_id_ = r.id();
            return robot_registered_id_;
        }
    }
    //If necessary add the location
    if(location.id()<0)
    {
        addElement(location, 0, relation::Str[relation::contain]);
    }
    //Add the robot
    int container_id = 0;
    if(location.hasProperty(relation::Str[relation::hasA]))
    {
        for(auto s : location.properties(relation::Str[relation::hasA]).getValues<std::string>())
            if(s==robot.label())
                container_id = location.id();
    }
    this->addElement(robot, container_id, relation::Str[relation::contain]);
    setRelation(robot.id(), relation::Str[relation::robotAt], location.id());
    robot_registered_id_ = robot.id();
    return robot_registered_id_;
}

//------------------------------------------------------------------------------------------------------------

skiros_wm::Element WorldModelInterfaceS::getRobot()
{
  return this->getElement(robot_registered_id_);
}

//------------------------------------------------------------------------------------------------------------

skiros_wm::Element WorldModelInterfaceS::getRobotLocation()
{
    return this->getParentElement(robot_registered_id_);
}

//------------------------------------------------------------------------------------------------------------

void WorldModelInterfaceS::unregisterRobot()
{
  if(robot_registered_id_ > 0)
  {
      removeBranch(robot_registered_id_);
  }
}
} // namespace skiros_wm
