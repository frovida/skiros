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

#include "skiros_world_model/owl_world_model.h"
#include "skiros_config/param_types.h"
#include "skiros_config/declared_uri.h"
#include "skiros_common/logger_sys.h"
#include "skiros_common/utility.h"
#include "skiros_world_model/reasoners_loading_func.h"
#include <boost/foreach.hpp>
#include <assert.h>
#include "skiros_common/utility.h"
#include <tinyxml.h>
#include <algorithm>
#include "skiros_world_model/owl_world_ontology.h"
#include "skiros_world_model/world_model_interface.h"
#include "skiros_world_model/utility.h"
#include "skiros_msgs/WoNode.h"

using namespace skiros_config::owl;

namespace skiros_wm
{
namespace owl
{

Node msgToOwlNode(skiros_msgs::WoNode node)
{
    switch(node.type)
    {
    case 0:
        return owl::Node(LIBRDF_NODE_TYPE_RESOURCE, node.uri);
    case 1:
        return owl::Node(LIBRDF_NODE_TYPE_LITERAL, node.uri, node.literal_type);
    case 2:
        return owl::Node(LIBRDF_NODE_TYPE_BLANK, node.uri);
    }
}

std::string LocalOntologyInterface::queryOntology(std::string query_string, bool cut_prefix)
{
    return ontology_->queryAsString(query_string.c_str(), cut_prefix);
}

std::string LocalOntologyInterface::getType(std::string uri){ return queryOntology("SELECT ?x where {"+addPrefix(uri)+" rdf:type ?x}"); }

//-----------------------------------------------------------------------------
//-------------------------------- ElementFactory -----------------------------
//-----------------------------------------------------------------------------

//TODO: look inside
bool ElementFactory::load(std::string type)
{
    loaded_type_ = type;
    //TODO: re-implement with new getContextStatement function
    ResultList statements = ontology_.getContextStatementsOld(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(type)));
    //FINFO(type << " is a " << ontology_.getUri(type));
    if(statements.size())
    {
        for(std::vector<librdf_node*> result : statements)
        {
            assert(result.size()==2);
            //If it's a type statement
            if(librdf_node_equals(result.at(0), LIBRDF_MS_type(ontology_.getWorldPtr())))
            {
                librdf_node * temp = librdf_new_node_from_uri_string(ontology_.getWorldPtr(), (unsigned char*)std_uri::OWL_INDIVIDUAL.c_str());
                if(librdf_node_equals(result.at(1), temp))
                {
                    //Save pair (individual, -)
                    attributes_map_.insert(UrisPair(std_uri::OWL_INDIVIDUAL, ""));
                }
                else
                {
                    //Save pair (type, uri)
                    attributes_map_.insert(UrisPair("type" , (char*)librdf_uri_as_string(librdf_node_get_uri(result.at(1)))));
                }
                librdf_free_node(temp);
            }
            //If it's a data property assertion
            else if(librdf_node_is_literal(result.at(1)))
            {
                std::string datatype = "";
                librdf_uri * temp = librdf_node_get_literal_value_datatype_uri(result.at(1));
                if(temp) datatype = (char*)librdf_uri_as_string(temp);
                //Save pair (literal_type, value, datatype)
                literals_map_.insert(LiteralsPair((char*)librdf_uri_as_string(librdf_node_get_uri(result.at(0))),
                                                  std::pair<std::string, std::string>
                                                  ((char*)librdf_node_get_literal_value(result.at(1)), datatype)));
            }
            //If it's a object property assertion
            else if(librdf_node_is_resource(result.at(1)))
            {
                //Save pair (relation, value)
                std::string temp = (char*)librdf_uri_as_string(librdf_node_get_uri(result.at(1)));
                temp = temp.substr(temp.find("#")+1);
                literals_map_.insert(LiteralsPair((char*)librdf_uri_as_string(librdf_node_get_uri(result.at(0))),
                                                  std::pair<std::string, std::string>
                                                  (temp, "resource")));
            }
            else
            {
                //TODO: save all other kind of specifications
                if(librdf_node_is_blank(result.at(1)))
                {

                }
                else
                {

                }
            }
            librdf_free_node(result.at(0));
            librdf_free_node(result.at(1));
        }
        return true;
    }
    else return false;
}

bool ElementFactory::isIndividual()
{
    if(attributes_map_.find(std_uri::OWL_INDIVIDUAL)!=attributes_map_.end()) return true;
    else return false;
}

std::string ElementFactory::getClassType()
{
    UrisMap::iterator it = attributes_map_.find("type");
    if(it!=attributes_map_.end()) return it->second;
    else throw std::runtime_error("[ElementFactory::getClassType] Type not found. ");
}

//-----------------------------------------------------------------------------
//-------------------------------- WorldModel ---------------------------------
//-----------------------------------------------------------------------------


void WorldModel::reset()
{
    while(!db_.map().empty())
    {
        removeElement(db_.map().begin()->second);
    }
    db_.clear();
}

//-------------------------------- Classification methods ---------------------------------

//TODO: extend with the subtypes...
IdentifyVector WorldModel::identify(skiros_wm::Element e, int root_id)
{
    IdentifyVector to_ret;
    IdentifyVector::iterator it;
    IdentifyList list = identifyRecursive(e, getElement(root_id));
    /// Insert in a sorted vector
    BOOST_FOREACH(skiros_msgs::WmObjLikelihood temp, list)
    {
        for(it = to_ret.begin();it != to_ret.end();it++)
        {
            if(it->likelihood < temp.likelihood)break;
        }
        to_ret.insert(it, temp);
    }
    return to_ret;
}

//TODO: extend with the subtypes...
IdentifyList WorldModel::identifyRecursive(Element e, Element root)
{
    IdentifyList to_ret;
    skiros_wm::Element child;
    skiros_msgs::WmObjLikelihood temp;
    skiros_wm::ReasonerPtrType reasoner;
    try
    {
        //TODO: extend to support multiple reasoners
        reasoner = skiros_wm::getDiscreteReasoner(e.properties(concept::Str[concept::SpatialReasoner]).getValue<std::string>());
    }
    catch(std::invalid_argument err)
    {
        FDEBUG(err.what());
        return to_ret;
    }
    //Get all root childs
    std::list<ChildsPair> childs = getChilds(root, relation::Str[relation::spatiallyRelated]);
    BOOST_FOREACH(ChildsPair pair, childs)
    {
        child = pair.second;
        /// If the child type match the one I need to identify, compute similarity likelihood
        if(child.type()==e.type() || e.type()==concept::Str[concept::Unknown] || child.type()==concept::Str[concept::Unknown])
        {
            try
            {
                temp.likelihood = reasoner->computeIstanceSimilarity(e, child);
                temp.id = child.id();
                FINFO("[WorldModel::identify] Compare object " << e.type() << " to object ID " << child.id() << ". Result likelihood: " << temp.likelihood);
                to_ret.push_back(temp);
            }
            catch(std::invalid_argument err)
            {
                FWARN("[WorldModel::identify] " << err.what());
            }
        }
        /// Go recursive
        IdentifyList temp1 = identifyRecursive(e, child);
        to_ret.insert(to_ret.end(), temp1.begin(), temp1.end());
    }
    return to_ret;
}

//TODO: type or label?
ClassifyVector WorldModel::classify(skiros_wm::Element e, float threshold)
{
  ClassifyVector to_ret;
  skiros_msgs::WmTypeLikelihood temp;
  skiros_wm::ReasonerPtrType reasoner;
  try
  {
    reasoner = skiros_wm::getDiscreteReasoner(e.properties(concept::Str[concept::SpatialReasoner]).getValue<std::string>());
  }
  catch(std::invalid_argument err)
  {
      FDEBUG(err.what());
      return to_ret;
  }
  std::list<std::string> branch = ontology_.getBranchIndividuals(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(e.type())));
  skiros_wm::Element e2;ClassifyVector::iterator it;
  BOOST_FOREACH(std::string individual, branch)
  {
      try
      {
            if(individual.find("-")!=std::string::npos) continue; //I discard the istance individuals, not valid for classification
            e2 = this->getDefaultElement(individual);
            temp.likelihood = reasoner->computeClassSimilarity(e, e2);
            FINFO("[WorldModel::classify] Compare object " << e.type() << " to the model " << e2.label() << ". Result likelihood: " << temp.likelihood);
            temp.type = e2.type();
            //If likelihood > of requested threshold
            if(temp.likelihood > threshold)
            {
              //Insert in a sorted vector
              for(it = to_ret.begin();it != to_ret.end();it++)
              {
                  if(it->likelihood < temp.likelihood)break;
              }
              to_ret.insert(it, temp);
            }
      }
      catch(std::invalid_argument err)
      {
          FWARN("[WorldModel::classify] " << err.what());
      }
  }
  return to_ret;
}

//---------------------------- Parsing methods ------------------------------

bool WorldModel::parseElementModify(std::string action, Element & e, RelationsVector & relations)
{
    //If no constraints are defined, return true
    if(class_constraint_map_.find(e.type()) == class_constraint_map_.end()) return true;
    //Else...
    std::vector<std::string> c_names = class_constraint_map_[e.type()];
    if(action == "add" || action == "update")
    {
        bool ok = true;
        BOOST_FOREACH(std::string name, c_names)
        {
            //Map the constraint name to its definition
            Constraint c = constraints_[name];
            std::string property;
            if(c.hasSpec(uri2lightString(std_uri::OWL_ONPROP))) property = c.getSpec(uri2lightString(std_uri::OWL_ONPROP));
            else continue;

            /*//Irreflexive prop constraint
            if(c.hasSpec(uri2lightString(std_uri::OWL_MINC)))
            {
                BOOST_FOREACH(RelationType relationn, relations)
                {
                    if(subPredicates.find(ontology_.getUri(relationn.predicate()))!=subPredicates.end() && (relationn.subject_id()==relationn.object_id()))
                    {
                        ok = false;
                    }
                }
            }*/
            //If it is a constraint over a relation
            if(ontology_.isDeclaredObjProp(property))
            {
                std::set<std::string> subPredicates = ontology_.getSubPropertiesS(property);
                //Constraint on Minimum cardinality
                if(c.hasSpec(uri2lightString(std_uri::OWL_MINC)))
                {
                    std::string temp = c.getSpec(uri2lightString(std_uri::OWL_MINC));
                    int minC = boost::lexical_cast<int>(temp);
                    BOOST_FOREACH(RelationType relationn, relations)
                    {
                        //FINFO(minC);
                        if(subPredicates.find(ontology_.getUri(relationn.predicate()))!=subPredicates.end()) minC--;
                    }
                    if(minC>0)
                    {
                        relations.push_back(RelationType(0, relation::Str[relation::contain], -1)); //Register wrt scene root
                    }
                }
            }
            //If it is a constraint over a data type
            if(ontology_.isDeclaredDataProp(property))
            {
                if(c.hasSpec(std_uri::OWL_MINC))
                {

                }
            }
        }
    }
    else if(action == "remove")
    {
        //EVAL (supposing the erasing element to be subject of every statement)
        BOOST_FOREACH(RelationType relationn, relations)
        {
            //FINFO(relationn.predicate() << relationn.object_id());
            if(relationn.subject_id() == relationn.object_id())
            {
                FERROR("[parseElementModify] Unexpected reflexive relation found for object " << relationn.subject_id() << ". Skipping.");
                continue;
            }
            if(relationn.predicate()==relation::Str[relation::contain] )
            {
                this->removeElement(relationn.object_id());
            }
            if(relationn.predicate()==relation::Str[relation::hasA])
            {
                this->removeElement(relationn.object_id());
            }
        }
    }
    else
        throw std::invalid_argument("Action not handled.");
    return true;
}

bool WorldModel::parseRelationModify(std::string action, Element subj, std::string predicate, Element obj)
{
    if(action == "add")
    {
        if(ontology_.isSubProperty(ontology_.getUri(predicate),
                                   ontology_.getUri(relation::Str[relation::sceneProperty])))
        {
            //Remove a previous add declaration assert(v.size()==1);//
            RelationsVector v = this->findStatements(-1, uri2lightString(relation::Str[relation::sceneProperty]), obj.id());
            if(v.size()==0) return true; //This appen if I create a new object
            if(v.size()>1) //This should never appen
            {
                std::stringstream ss;
                ss << "[WorldModel::parseRelationModify] World model corrupted, expected 1 relation '" << uri2lightString(predicate) << "' found " << v.size();
                throw std::runtime_error(ss.str().c_str());
            }
            ontology_.removeStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(this->getElement(v[0].subject_id()))),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(v[0].predicate())),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(this->getElement(v[0].object_id()))));
        }
    }
    else if(action == "remove")
    {
        if(ontology_.isSubProperty(ontology_.getUri(predicate),
                                   ontology_.getUri(relation::Str[relation::sceneProperty])))
        {
            //Cannot remove directly
            return false;
        }
    }
    else
    {
        throw std::invalid_argument("Action not handled.");
    }
    return true;
}

//-------------------------------- Element methods ---------------------------------

WorldModel::WorldModel(Ontology & ont) : ontology_(ont)
{
    //TODO: remove this
    ontology_.addPrefix(std::pair<std::string, std::string>("http://www.semanticweb.org/francesco/ontologies/2014/9/stamina","stmn"));
}

void WorldModel::removeElements(std::vector<int> ids)
{
    for(int id : ids)
        removeElement(id);
}

bool WorldModel::removeElement(int id)
{
    DatabaseMap::iterator it = db_.map().find(id);
    if(it!=db_.map().end()) return this->removeElement(it->second);
    else return false;
}

bool WorldModel::removeElement(skiros_wm::Element e)
{
    //FINFO("Remove " << e.printState("", false));
    RelationsVector relations = this->findStatements(e.id(), "", -1);
    if(this->parseElementModify("remove", e, relations))
    {
        //FINFO("Remove: " << e.printState("",false));
        if(removeElementInOntology(e))
        {
            db_.remove(e.id());
            return true;
        }
    }
    else return false;
}

void WorldModel::checkReasoners(Element & e)
{
    auto it = e.properties().find(data::Str[data::DiscreteReasoner]);
    if(it != e.properties().end())
    {
        auto reasoner_names = it->second.getValues<std::string>();
        for(std::string name : reasoner_names)
        {
            skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner(name);
            reasoner->addProperties(e);
        }
    }
}

skiros_wm::Element WorldModel::getElement(int id)
{
    DatabaseMap::iterator it = db_.map().find(id);
    if(it!=db_.map().end()) return it->second;
    else return skiros_wm::Element();
}

void WorldModel::addElementInOntology(Element e)
{
    static LocalOntologyInterface * WO = new LocalOntologyInterface(&ontology_);
    auto statements = e.toMsgStatements(WO, true);
    //Register the individual type
    for(auto statement : statements)
    {
        //FINFO(statement.subject.uri << "(" << std::to_string(statement.subject.type) << ") " << statement.predicate.uri << "(" << std::to_string(statement.predicate.type) << ") "  << statement.object.uri << "(" << std::to_string(statement.object.type) << ") " );
        ontology_.addStatement(msgToOwlNode(statement.subject),
                                msgToOwlNode(statement.predicate),
                                msgToOwlNode(statement.object));
    }
    //Relate to the scene
    ontology_.addStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, e.toUrl()),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(relation::Str[relation::relatedTo])),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(element2uri(db_.map()[0]))));

}

void WorldModel::updateElementInOntology(Element e, Element old)
{
    static LocalOntologyInterface * WO = new LocalOntologyInterface(&ontology_);
    auto new_statements = e.toMapStatements(WO);
    auto old_statements = old.toMapStatements(WO);
    //Remove all old data statements
    for(auto old_statement : old_statements)
    {
        if(ontology_.isDeclaredDataProp(ontology_.getUri(old_statement.first)))
            ontology_.removeStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, e.toUrl()),
                                      owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(old_statement.first)),
                                      owl::Node(LIBRDF_NODE_TYPE_LITERAL, old_statement.second.first, old_statement.second.second));
    }
    //Add new data statements
    for(auto new_statement : new_statements)
    {
        if(ontology_.isDeclaredDataProp(ontology_.getUri(new_statement.first)))
            ontology_.addStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, e.toUrl()),
                                   owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(new_statement.first)),
                                   owl::Node(LIBRDF_NODE_TYPE_LITERAL, new_statement.second.first, new_statement.second.second));
    }
}

bool WorldModel::removeElementInOntology(Element e)
{
    return ontology_.removeContextStatements(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, e.toUrl()));
}

bool uninitializedSubObj(RelationType r)
{
    if(r.subject_id() == -1 && r.subject_id()==r.object_id())
    {
        FWARN("[addElement] A relation with subject and object uninitialized has been ignored.");
        return true;
    }
    return false;
}

int WorldModel::addElement(Element & e, RelationsVector relations)
{
    RelationsVector::iterator pend = std::remove_if(relations.begin(), relations.end(), uninitializedSubObj);
    relations.erase(pend, relations.end());

    std::string class_type_uri = "";
    e.type() = uri2lightString(e.type());
    if(this->isDeclaredClass(e.type())) class_type_uri = e.type();
    if(class_type_uri!="")
    {
        if(!this->parseElementModify("add", e, relations))
            return -1;
        checkReasoners(e);
        //Save the c++ element in a memory database. This assign an ID to the element
        db_.add(e);
        //Register the element in the ontology
        addElementInOntology(e);
        //Add object relations
        for(RelationType relation : relations)
        {
            //-1 is the special number for auto-referencing
            if(relation.subject_id()<0)relation.subject_id() = e.id();
            if(relation.object_id()<0)relation.object_id() = e.id();
            //Add statement
            if(!this->addStatement(relation.subject_id(), relation.predicate(), relation.object_id()))
                FERROR("[WorldModel::addElement]: Failed to add relation: " << relation.subject_id() << "-" << relation.predicate() << "-" << relation.object_id());
        }
        return e.id();
    }
    else
    {
        FERROR("[WorldModel::addElement]: Type '" << e.type() << "' not defined in ontology. Insertion aborted.");
        return -1;
    }
}

bool WorldModel::updateElement(skiros_wm::Element e, RelationsVector v)
{
    DatabaseMap::iterator it = db_.map().find(e.id());
    if(it==db_.map().end()) return false;
    else
    {
        //Add relations
        for(RelationType r : v)
        {
            //-1 is the special number for auto-referencing
            if(r.subject_id()<0)r.subject_id() = e.id();
            if(r.object_id()<0)r.object_id() = e.id();
            //TODO: on failure revert changes
            if(!addStatement(r.subject_id(),r.predicate(), r.object_id()))
            {
                FWARN("[updateElement] Error while updating element " << e.id() << ". Procedure aborted. ");
                if(v.size()>1)
                    FWARN("World model could be corrupted.");
                //TODO: ADD A REVERT ACTION
                return false;
            }
        }
        //Add reasoners default properties
        checkReasoners(e);
        //Update ontology
        updateElementInOntology(e, it->second);
        //Update DB
        it->second = e;
    }
    return true;
}

skiros_wm::Element WorldModel::getDefaultElement(std::string type)
{
    type = uri2lightString(type);
    //TODO: find equivalents termssss
    //Check if already stored in buffer
    if(default_individuals_.find(type)==default_individuals_.end())
    {
        if(!ontology_.isDeclaredIndividual(type))
        {
            std::stringstream ss;
            ss << "[WorldModel::getDefaultElement] Type: " << type << " is not an individual.";
            throw std::invalid_argument(ss.str().c_str());
        }
        default_individuals_[type] = createDefaultIndividual(type);
    }
    return default_individuals_[type];
}

std::vector<skiros_wm::Element> WorldModel::resolveElement(skiros_wm::Element e)
{
  std::vector<skiros_wm::Element> v;
  //I get all subclasses of the input
  std::set<std::string> type_set = ontology_.getSubClassesS(e.type());
  //Make a list of all the elements having the same type (special case on unknown)
  BOOST_FOREACH(DatabaseMap::value_type pair, db_.map())
  {
      bool match = true;
      if(type_set.find(ontology_.getUri(pair.second.type()))==type_set.end()
              && e.type()!=concept::Str[concept::Unknown]
              && pair.second.type()!=concept::Str[concept::Unknown])
          match = false;
      if(e.label()!="" && pair.second.label()!=e.label())
          match = false;
      /*BOOST_FOREACH(skiros_common::ParamMap::value_type pair, e.properties())
      {
          //TODO: need to compare automatically the values of two params
          //if(pair.second.)
              match = false;
      }*/
      if(match)
          v.push_back(pair.second);
  }
  return v;
}

std::list<ChildsPair> WorldModel::getChilds(skiros_wm::Element parent, std::string relation_filter, std::string type_filter)
{
    std::list<ChildsPair> to_ret;
    if(relation_filter=="")
        relation_filter=relation::Str[relation::sceneProperty];
    //Recursively print all sub tree
    relation_filter = uri2lightString(relation_filter);
    RelationsVector v = this->findStatements(parent.id(), relation_filter, -1);
    std::set<std::string> types_set;
    if(type_filter!="")
    {
        types_set = ontology_.getSubClassesS(type_filter);
        //types_set.insert(concept::Str[concept::Unknown]);
    }
    for(RelationType rel : v)
    {
        skiros_wm::Element temp = getElement(rel.object_id());
        if(types_set.find(ontology_.getUri(temp.type()))!=types_set.end() || type_filter=="")
            to_ret.push_back(ChildsPair(rel.predicate(), temp));
    }
    return to_ret;
}

ExportedGraphType WorldModel::getBranch(skiros_wm::Element parent, std::string relation_filter, std::string type_filter)
{
    ExportedGraphType to_ret;
    if(relation_filter=="")relation_filter=relation::Str[relation::sceneProperty];
    to_ret.first.push_back(parent);
    std::list<ChildsPair> childs = getChilds(parent, relation_filter, type_filter);
    BOOST_FOREACH(ChildsPair p, childs)
    {
        if(type_filter=="" || p.second.type()==type_filter)
        {
            to_ret.second.push_back(RelationType(parent.id(), p.first, p.second.id()));
            ExportedGraphType temp = getBranch(p.second, relation_filter, type_filter);
            to_ret.first.insert(to_ret.first.end(), temp.first.begin(), temp.first.end());
            to_ret.second.insert(to_ret.second.end(), temp.second.begin(), temp.second.end());
        }
    }
    return to_ret;
}

//-------------------------------- Relation methods ---------------------------------

bool WorldModel::removeStatement(int sub_id, std::string predicate, int obj_id)
{
    skiros_wm::Element sub = this->getElement(sub_id);
    skiros_wm::Element obj = this->getElement(obj_id);
    if(sub.id()>=0 && obj.id()>=0 && isDeclaredRelation(predicate))
    {
        //Parse
        if(this->parseRelationModify("remove", sub, predicate, obj))
        {
            ontology_.removeStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(sub)),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(predicate)),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(obj)));
            return true;
        }
    }
    else return false;
}

bool WorldModel::addStatement(int sub_id, std::string predicate, int obj_id)
{
    skiros_wm::Element sub = this->getElement(sub_id);
    skiros_wm::Element obj = this->getElement(obj_id);
    predicate = ontology_.getUri(predicate);
    if(db_.isRegisteredId(sub.id()) && db_.isRegisteredId(obj.id()) && isDeclaredRelation(uri2lightString(predicate)))
    {
        //Parse
        if(this->parseRelationModify("add", sub, predicate, obj))
        {
            ontology_.addStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(sub)),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, predicate),
                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(obj)));
            return true;
        }
        else return false;
    }
    else
    {
        FERROR("[WorldModel::addStatement] Statement " << sub.id() << "-" << uri2lightString(predicate) << "-" << sub.id() << " is not valid.");
        return false;
    }
}

RelationsVector WorldModel::findStatements(int sub_id, std::string predicate, int obj_id)
{
    //INIT
    RelationsVector to_ret;
    RelationType relation;
    std::string subj_name(""), obj_name("");
    if(predicate=="")
        predicate = relation::Str[relation::sceneProperty];
    if(sub_id>=0)
        subj_name = element2uri(getElement(sub_id));
    if(obj_id>=0)
        obj_name = element2uri(getElement(obj_id));

    predicate = ontology_.getUri(predicate);

    //INPUT CHECK
    if(sub_id<0 && obj_id<0)
    {
        FERROR("[WorldModel::findStatements] At least 1 argument of the statement must be provided. ");
        return RelationsVector();
    }
    if(!isDeclaredRelation(uri2lightString(predicate)))
    {
        FERROR("[WorldModel::findStatements] Predicate: " << uri2lightString(predicate) << " wasn't declared in the ontology.");
        return RelationsVector();
    }

    //MAIN
    std::list<std::string> subPredicates = ontology_.getSubPropertiesL(predicate);
    for(std::string subPredicate : subPredicates)
    {
        //Make the query
        librdf_stream * stream = NULL;
        stream = ontology_.getStatements(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, subj_name),
                                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, subPredicate),
                                         owl::Node(LIBRDF_NODE_TYPE_RESOURCE, obj_name));
        //Convert the list to RelationsVector
        if(!stream) return to_ret;
        while(!librdf_stream_end(stream))
        {
            Statement s = ontology_.getCppStatement(librdf_stream_get_object(stream));
            relation.subject_id() = uri2id(s.subject.data); //This return -1 if the uri isn't one of an element
            relation.predicate() = uri2lightString(s.predicate.data);
            relation.object_id() = uri2id(s.object.data); //This return -1 if the uri isn't one of an element
            to_ret.push_back(relation);
            //FINFO("Found: " << temp.subject_id() << temp.predicate() << temp.object_id());
            librdf_stream_next(stream);
        }
        //Finish
        librdf_free_stream(stream);
    }
    return to_ret;
}

//-------------------------------- IO methods --------------------------------

std::string WorldModel::getSceneTree(bool verbose, std::string relations)
{
    return this->printTree(0,relations, "", verbose);
}

bool WorldModel::loadScene(std::string filename)
{
  //Open document --------------
  std::string path = filename;
  TiXmlDocument doc(path);
  FINFO("Opening file '" << path << "'...");
  if (!doc.LoadFile())
  {
      FERROR("File not found.");
      return false;
  }
  TiXmlHandle hDoc(&doc);
  TiXmlElement* pElem;
  TiXmlHandle root(0);

  //Get the root element --------------
  pElem=hDoc.FirstChildElement().Element();
  // should always have a valid root but handle gracefully if it does
  if (!pElem)
  {
      FERROR("Format not valid.");
      return false;
  }

  //----- Clear the current world model and create a new one --------
  this->reset();

  this->createScene("aau_lab");

  std::string m_name = pElem->Value();
  // save this for later
  root=TiXmlHandle(pElem);

  //----- Get the world elements --------------
  TiXmlElement* properties;
  for( pElem=root.FirstChild( "element" ).Element(); pElem; pElem=pElem->NextSiblingElement("element"))
  {
      //FINFO(pElem->Value());
      skiros_wm::Element e;
      pElem->QueryIntAttribute("id", &e.id());
      pElem->QueryStringAttribute("type", &e.type());
      pElem->QueryStringAttribute("label", &e.label());
      double sec;
      pElem->QueryDoubleAttribute("last_update", &sec);
      e.lastUpdate().fromSec(sec);
      properties = pElem->FirstChild()->ToElement();
      TiXmlElement* prop = properties->FirstChildElement();
      if(prop!=NULL)
      {
        for( prop; prop; prop=prop->NextSiblingElement())
        {
            int temp_state, temp_specType, temp_length;
            std::string temp_key, temp_name, temp_ash;
            unsigned long int temp_ui;
            temp_key = prop->Value();
            prop->QueryStringAttribute("name",&temp_name);
            prop->QueryIntAttribute("state",&temp_state);
            prop->QueryIntAttribute("spec_type",&temp_specType);
            prop->QueryIntAttribute("array_size",&temp_length);
            prop->QueryStringAttribute("ash",&temp_ash);
            temp_ui = boost::lexical_cast<unsigned long int>(temp_ash);
            skiros_common::Param p(temp_key, temp_name, skiros_common::any::getTypeFromHash((uint64_t)temp_ui), (skiros_common::ParamSpecType) temp_specType, temp_length);
            if((skiros_common::ParameterState) temp_state == skiros_common::specified)
            {
                skiros_common::utility::SerialData sd;
                prop->QueryUnsignedAttribute("payload_length",&sd.length);
                TiXmlText* text = prop->FirstChild()->ToText();
                std::stringstream ss(text->Value());
                int temp_ui;
                //sd.data.resize(smap.length);
                for(int i = 0; i<sd.length; i++)
                {
                    //FINFO(temp_ui);
                    ss >> temp_ui;
                    sd.data.push_back((uint8_t)temp_ui);
                }
                //FINFO(sd.length << " and data " << sd.data.size());
                p.setAllValues(skiros_common::utility::deserializeAnyVector(sd));
            }
            e.addProperty(p);
        }
      }
      if(e.id()!=0)
      {
          db_.add(e, true);
          addElementInOntology(e);
      }
  }
  //----- Get the world relations --------------
  RelationType relation;
  for( pElem=root.FirstChild( "relation" ).Element(); pElem; pElem=pElem->NextSiblingElement("relation"))
  {
      std::string temp;
      pElem->QueryStringAttribute("subject", &temp);
      relation.subject_id() = uri2id(temp);
      pElem->QueryStringAttribute("predicate", &relation.predicate());
      pElem->QueryStringAttribute("object", &temp);
      relation.object_id() = uri2id(temp);
      addStatement(relation.subject_id(), relation.predicate(), relation.object_id());
  }
  return true;
}

bool WorldModel::saveScene(std::string filename)
{
  //---- Document generation + header ----
  TiXmlDocument doc;
  TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
  TiXmlElement * root = new TiXmlElement( "world_state" );
  doc.LinkEndChild( decl );
  doc.LinkEndChild( root );
  TiXmlComment * comment = new TiXmlComment();
  comment->SetValue(" This file was generated from 'skiros_world_model/world_model.h' " );
  root->LinkEndChild( comment );
  std::string scene_name = filename;
  if(filename.find('.')!=std::string::npos)
      scene_name = filename.substr(0, filename.find('.'));
  db_.map().at(0).label() = scene_name;
  //----- Add World-state elements-----
  TiXmlText * text;
  std::stringstream ss;
  for(DatabaseMap::iterator it=db_.map().begin();it!=db_.map().end();it++)
  {
      TiXmlElement * element = new TiXmlElement("element");
      element->SetAttribute("id", it->second.id());
      element->SetAttribute("type", it->second.type());
      element->SetAttribute("label", it->second.label());
      element->SetDoubleAttribute("last_update", it->second.lastUpdate().toSec());
      TiXmlElement * properties = new TiXmlElement("properties");
      element->LinkEndChild( properties );
      //Extract properties
      skiros_common::ParamMap::iterator itt = it->second.properties().begin();
      for(itt=it->second.properties().begin();itt!=it->second.properties().end();itt++)
      {
          TiXmlElement * property = new TiXmlElement(itt->second.key());
          property->SetAttribute("name", itt->second.name());
          property->SetAttribute("state", (int)itt->second.state());
          property->SetAttribute("spec_type", (int)itt->second.specType());
          property->SetAttribute("array_size",itt->second.size());
          uint64_t ttt = skiros_common::any::getHashFromType(itt->second.type());
          ss.str("");
          ss << ttt;
          property->SetAttribute("ash",ss.str());
          ss.str("");
          //FINFO(ttt << " " << str.str());
          if(itt->second.state() == skiros_common::specified)
          {
              skiros_common::utility::SerialData sd = skiros_common::utility::serializeAnyVector(itt->second.getValues());
              property->SetAttribute("payload_length", sd.length);
              ss.str("");
              for(int i=0;i<sd.length;i++) ss << (unsigned short int)sd.data[i] << " ";
              text = new TiXmlText(ss.str());
              property->LinkEndChild(text);
          }
          properties->LinkEndChild( property );
      }
      root->LinkEndChild( element );
      //----- Add World-state relations -----
      auto v = ontology_.getContextStatements(Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(it->second)), 1); //Get active statements
      for(Statement & s : v)
      {
          if(ontology_.isDeclaredObjProp(s.predicate.data))
          {
              TiXmlElement * relation = new TiXmlElement("relation");
              relation->SetAttribute("subject", uri2lightString(s.subject.data));
              relation->SetAttribute("predicate", uri2lightString(s.predicate.data));
              relation->SetAttribute("object", uri2lightString(s.object.data));
              root->LinkEndChild( relation );
          }
      }
  }

  //------- SAVE ----------------
  //Check if the name is valid, or append extention
  if(filename.find(".xml") == std::string::npos)
      filename.append(".xml");
  std::string path = filename;
  ROS_INFO("Saving file: %s", path.c_str());
  doc.SaveFile( path.c_str() );
  return true;
}

//-------------------------------- Utils --------------------------------

std::string WorldModel::uri2lightString(std::string uri)
{
    //If the input is already not an uri, just return
    if(uri.find("#")==std::string::npos) return uri;
    std::string prefix = uri.substr(0, uri.find("#"));
    //If the prefix is the default uri, I just remove it
    if(prefix==ontology_.getDefaultUri())
    {
        uri.replace(uri.find(prefix), prefix.length()+1, "");
    }
    else if(ontology_.hasPrefix(prefix))
    {
        std::string short_prefix = ontology_.getShortPrefix(prefix);
        uri.replace(uri.find(prefix), prefix.length()+1, short_prefix+":");
    }
    return uri;
}

int WorldModel::uri2id(std::string uri)
{
    int to_ret = -1;
    if(uri.find("-")!=std::string::npos)
    {
        uri = uri.substr(uri.find("-")+1);
        try
        {
            to_ret = boost::lexical_cast<int>(uri);
        }
        catch(boost::bad_lexical_cast err)
        {
            FERROR("[WorldModel::uri2id] " << err.what());
        }
    }
    return to_ret;
}

std::string WorldModel::element2uri(skiros_wm::Element e)
{
    std::stringstream URI;
    URI<<ontology_.getUri(e.type())<<"-"<<e.id();
    return URI.str();
}

//-------------------------------- Private methods --------------------------------


LiteralsMap WorldModel::getLiteralsMapLight(LiteralsMap in)
{
    LiteralsMap light_map;
    BOOST_FOREACH(LiteralsMap::value_type v, in)
    {
        light_map.insert(LiteralsPair(uri2lightString(v.first),
                                      std::pair<std::string, std::string>
                                      (uri2lightString(v.second.first), uri2lightString(v.second.second))));
    }
    return light_map;
}

std::string WorldModel::printTree(int root_id, std::string relations, std::string indend, bool verbose)
{
    std::stringstream to_ret;
    //Print element
    skiros_wm::Element e = this->getElement(root_id);
    to_ret << e.printState(indend, verbose) << "\n";
    std::string white_string;
    RelationsVector v = this->findStatements(root_id, relations, -1);
    BOOST_FOREACH(RelationType rel, v)
    {
        std::string temp = uri2lightString(rel.predicate());
        white_string.clear();
        for(int i=0;i<temp.size()+2;i++)white_string.push_back(' ');
        to_ret << indend+temp+"->";
        to_ret << this->printTree(rel.object_id(), relations, indend+white_string, verbose);
    }
    return to_ret.str();
}

skiros_wm::Element WorldModel::createDefaultClass(std::string owl_class)
{
    /*ElementFactory factory(ontology_);
    factory.load(owl_class);*/
    std::string uri = uri2lightString(ontology_.getUri(owl_class));
    if(this->isDeclaredClass(uri))
        return skiros_wm::Element(uri);
    else
    {
        std::stringstream ss;
        ss << "[WorldModel::createDefaultClass] " << uri << " is not a declared class.";
        throw std::invalid_argument(ss.str().c_str());
    }
}

skiros_wm::Element WorldModel::createDefaultIndividual(std::string owl_individual)
{
    ElementFactory element(ontology_);
    element.load(owl_individual);
    if(!element.isIndividual())
    {
        std::stringstream ss;
        ss << "[WorldModel::createDefaultIndividual] " << "Type: " << owl_individual << " is not an individual.(or it is not associated to any class)";
        throw std::invalid_argument(ss.str().c_str());
    }
    skiros_wm::Element e(uri2lightString(element.getClassType())); //Throws runtime_error if not type is defined
    e.label() = owl_individual;
    LiteralsMap prop_map = getLiteralsMapLight(element.getLiteralsMap());
    skiros_config::ParamTypes param_factory = skiros_config::ParamTypes::getInstance();
    for(LiteralsPair pair : prop_map)
    {
        //FINFO("Inserting: " << pair.first << "-" << pair.second.first);
        std::string key = pair.first;
        std::string value = pair.second.first;
        try
        {
            e.addProperty(param_factory.getDefault(key), value);
        }
        catch(std::runtime_error err)
        {
            std::string datatype = pair.second.second;
            //If a standard parameter is not defined I build a parameter accoring to declared datatype
            //FINFO("Aggiungo: " << key << " con " << value);
            e.addProperty(param_factory.getFromString(key, datatype), value);
        }
    }
    LiteralsMap::iterator it = prop_map.find("DiscreteReasoner");
    if(it != prop_map.end())
    {
        skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner(it->second.first);
        if(reasoner)
        {
            reasoner->storeData(e, prop_map);
        }
        else
        {
            FWARN("[createDefaultIndividual] Reasoner " + it->second.first + " not found. In case it exists, try to resource setup.bash. Note: all object's properties normally handled by this reasoner will now be ignored.");
        }
    }
    return e;
}

void WorldModel::setWorld(std::string scene_name)
{
    //TODO: check for the presence of all needed classes\individuals (as defined in declared_uri.h)
    db_.clear();

    //Load constraints
    //Create a (multi) map Class->constraint
    ResultList results = ontology_.getClassToConstraintsMap();
    std::string class_name, constraint;
    for(std::vector<librdf_node *> v : results)
    {
        class_name = (char *) librdf_uri_as_string(librdf_node_get_uri(v[0]));
        std::list<std::string> sub_classes = ontology_.getSubClassesL(class_name);
        constraint = (char *) librdf_node_get_blank_identifier(v[1]);
        for(std::string subC : sub_classes)
        {
            subC = uri2lightString(subC);
            class_constraint_map_[subC].push_back(constraint);
            //FINFO(subC << " class has constraint: " << constraint);
        }
    }

    //Get all constraint-related statements
    results = ontology_.getConstraintsDescription();
    typedef std::map<std::string, std::vector<std::pair<std::string, std::string> > > ConstraintMap;
    typedef std::pair<std::string, std::vector<std::pair<std::string, std::string> > > ConstraintPair;
    std::pair<std::string, std::string> statement;
    ConstraintMap temp_c_map;
    //Group the statements by constraint name
    for(std::vector<librdf_node *> v : results)
    {
        constraint = (char *) librdf_node_get_blank_identifier(v[0]);
        statement.first = uri2lightString((char *) librdf_uri_as_string(librdf_node_get_uri(v[1])));
        //Can have a literal or a resource as second
        if(librdf_node_is_resource(v[2])) statement.second = uri2lightString((char *) librdf_uri_as_string(librdf_node_get_uri(v[2])));
        if(librdf_node_is_literal(v[2])) statement.second = (char *) librdf_node_get_literal_value(v[2]);
        temp_c_map[constraint].push_back(statement);
        //FINFO(constraint << " " << uri2lightString(statement.first) << " " << statement.second);
    }
    //Create a map constraint->class constraint
    for(ConstraintMap::iterator it = temp_c_map.begin(); it!=temp_c_map.end();it++)
    {
        Constraint c(&ontology_, &db_);
        c.load(it->second);//Load a list of statements regarding 1 spefic constraint
        constraints_[it->first] = c;
    }
    //Register a new scene
    createScene(scene_name);
    //Finish
}

void WorldModel::createScene(std::string name)
{
    scene_name_ = name;
    skiros_wm::Element root;
    try
    {
        root = getDefaultElement(name);
    }
    catch(std::invalid_argument e)
    {
        root = createDefaultClass(concept::Str[concept::Scene]);
    }
    root.label() = name;
    db_.addRoot(root);
    ontology_.addStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(root)),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, ontology_.getUri(root.type())));
    //Register a new individual
    ontology_.addStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, element2uri(root)),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE),
                 owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_INDIVIDUAL));
}
//NOTE THAT THE BELOW ARE NEVER USED INSIDE THE WM (We use isDeclaredIndividual(), much faster)
std::set<std::string> WorldModel::getDeclaredIndividuals()
{
    std::set<std::string> declared_individuals;
    std::list<std::string> classes = ontology_.getAllIndividuals();
    BOOST_FOREACH(std::string str, classes)
    {
        //FINFO(uri2lightString(str));
        declared_individuals.insert(uri2lightString(str));
    }
    return declared_individuals;
}

std::set<std::string> WorldModel::getDeclaredClasses()
{
    std::set<std::string> declared_classes;
    std::list<std::string> classes = ontology_.getAllClasses();
    BOOST_FOREACH(std::string str, classes)
    {
        //FINFO(uri2lightString(str));
        declared_classes.insert(uri2lightString(str));
    }
    return declared_classes;
}

std::set<std::string> WorldModel::getDeclaredData()
{
    std::set<std::string> declared_data;
    std::list<std::string> classes = ontology_.getAllDataProp();
    BOOST_FOREACH(std::string str, classes)
    {
        //FINFO(uri2lightString(str));
        declared_data.insert(uri2lightString(str));
    }
    return declared_data;
}

std::set<std::string> WorldModel::getDeclaredRelations()
{
    std::set<std::string> declared_relations;
    std::list<std::string> classes = ontology_.getAllObjProp();
    BOOST_FOREACH(std::string str, classes)
    {
        //FINFO(uri2lightString(str));
        declared_relations.insert(uri2lightString(str));
    }
    return declared_relations;
}

bool WorldModel::isDefaultIndividual(std::string name){return ontology_.isDeclaredIndividual(name);}
bool WorldModel::isDeclaredRelation(std::string name){return ontology_.isDeclaredObjProp(name);}
bool WorldModel::isDeclaredClass(std::string name){return ontology_.isDeclaredClass(name);}
void WorldModel::loadMainOntology(std::string filename, std::string scene_name)
{
    //TODO: minor fix: clear the previous ontology_ before loading a new one.
    ontology_.loadMainOntology(filename, "skiros");
    default_individuals_.clear();
    setWorld(scene_name);
}

void WorldModel::loadSubOntology(std::string filename)
{
    ontology_.loadSubOntology(filename);
    default_individuals_.clear();
}

void WorldModel::importOntology(owl::Ontology & other_ontology)
{
    ontology_.importOntology(other_ontology);
    default_individuals_.clear();
}

std::string WorldModel::getWorkspacePath() const { return ontology_.getWorkspacePath();}
}
}
