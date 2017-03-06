#include <boost/any.hpp>
#include "skiros_common/param.h"
#include "skiros_world_model/world_element.h"
#include "skiros_common/logger_sys.h"
#include <exception>      // std::exception
#include <boost/foreach.hpp>
#include <skiros_world_model/reasoners_loading_func.h>
#include <skiros_msgs/WoStatement.h>
#include "skiros_world_model/std_uris.h"
#include "skiros_config/param_types.h"

using namespace skiros_config::owl;
using namespace skiros_msgs;
using namespace skiros_wm::owl;

namespace skiros_wm
{
skiros_common::Param & Element::properties(std::string key)
{
    skiros_common::ParamMap::iterator it = this->properties_.find(key);
    if(it != this->properties_.end()) return it->second;
    else
    {
        std::stringstream ss;
        ss << "[Element::properties] Property '" << key << "' not found in element type '" << this->type() << "'. ID: " << this->id_;
        //FERROR(ss.str());
        std::invalid_argument e(ss.str());
        throw e;
    }
}

std::string Element::printState(std::string indend, bool verbose) const
{
    std::stringstream ss;
    if (this->id()>=0)
        ss << this->label() << " (" << this->type() << "-" << this->id() << ")";
    else
        ss << this->label() << " (abstract)";
    if(verbose)
    {
        ss  << " \n";
        std::map<std::string, skiros_common::Param>::const_iterator it = properties_.begin();

        for(it=properties_.begin();it!=properties_.end();it++)
        {
            ss << indend << it->second.printState() << " \n";
        }
    }
    return ss.str();
}

bool Element::addProperty(skiros_common::Param p)
{
    std::pair<std::map<std::string, skiros_common::Param>::iterator,bool> ret = properties_.insert(std::pair<std::string, skiros_common::Param >(p.key(), p));
    return ret.second;
}

bool Element::addProperty(skiros_common::Param p, skiros_common::any default_value)
{
    if(addProperty(p))
    {
        properties_[p.key()].setValue(default_value);
    }
    else
    {
        properties_[p.key()].push_back(default_value);
    }
    return true;
}

bool Element::addPropertyString(std::string key, std::string default_value)
{
    if(addProperty(skiros_common::Param(key, "", typeid(std::string), skiros_common::symbolic, 1)))
    {
        properties_[key].setValue(default_value);
    }
    else
    {
        properties_[key].push_back(default_value);
    }
}

bool Element::addProperty(std::string key, skiros_common::any default_value)
{
    if(addProperty(skiros_common::Param(key, "", default_value.type(), skiros_common::symbolic, 1)))
    {
        properties_[key].setValue(default_value);
    }
    else
    {
        properties_[key].push_back(default_value);
    }
    return true;
}

std::string Element::toUrl() const {return type()+"-"+std::to_string(id());}

std::vector<skiros_msgs::WoStatement> Element::toMsgStatements(BaseOntologyInterface * wo, bool scene_element)
{
    std::vector<skiros_msgs::WoStatement> to_ret;
    WoStatement temp;
    std::string uri;
    if(scene_element)
        uri = toUrl();
    else
        uri = label();
    temp.subject.uri = uri;
    temp.subject.type = temp.subject.RESOURCE;
    temp.predicate.uri = std_uri::TYPE;
    temp.predicate.type = temp.subject.RESOURCE;
    temp.object.uri = this->type();
    temp.object.type = temp.subject.RESOURCE;
    to_ret.push_back(temp);
    //Register a new individual
    temp.object.uri = std_uri::OWL_INDIVIDUAL;
    to_ret.push_back(temp);
    //Save the properties stored by SpatialReasoners
    if(this->hasProperty(data::DiscreteReasoner))
    {
        std::vector<std::string> v = this->properties(data::DiscreteReasoner).getValues<std::string>();
        for(std::string r : v)
        {
            skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner(r);
            ReasonerDataMap data = reasoner->extractOwlData(*this);
            for(ReasonerDataMap::value_type pair : data)
            {
                temp.predicate.uri = pair.first;
                temp.object.uri = pair.second.first;
                temp.object.type = temp.subject.LITERAL;
                temp.object.literal_type = pair.second.second;
                to_ret.push_back(temp);
            }
        }
    }
    //Save all the remaining properties
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    for(skiros_common::ParamMap::value_type pair : this->properties())
    {
        if(wo->getType(pair.first).find("ObjectProperty")!=std::string::npos)
        {
            if(!scene_element)
                for(int i=0; i<pair.second.size();i++)
                {
                    std::string value = pair.second.getValueStr(i);
                    temp.predicate.uri = pair.first;
                    temp.object.uri = value;
                    temp.object.type = temp.subject.RESOURCE;
                    to_ret.push_back(temp);
                }
        }
        else
        {
            //FWARN("[Element::toMapStatements] Element '" << *this << "' has data property '" << pair.first << "' with size " << pair.second.size() << ". Only one will appear in the ontology.");
            auto values = pair.second.getValuesStr();
            temp.predicate.uri = pair.first;
            temp.object.type = temp.subject.LITERAL;
            temp.object.literal_type = param_types.getDataTypeStr(pair.second);
            for(auto value : values)
            {
                temp.object.uri = value;
                to_ret.push_back(temp);
            }
        }
    }
    return to_ret;
}

using SPair = std::pair<std::string, std::string>;
RelationsMMap Element::toMapStatements(BaseOntologyInterface * wo)
{
    RelationsMMap to_ret;
    Element copy = *this;
    to_ret.insert(RelationsPair(std_uri::TYPE, SPair(this->type(), "")));
    to_ret.insert(RelationsPair(std_uri::TYPE, SPair(std_uri::OWL_INDIVIDUAL, "")));
    //Save the properties stored by SpatialReasoners
    if(copy.hasProperty(data::DiscreteReasoner))
    {
        std::vector<std::string> v = copy.properties(data::DiscreteReasoner).getValues<std::string>();
        for(std::string r : v)
        {
            skiros_wm::ReasonerPtrType reasoner = skiros_wm::getDiscreteReasoner(r);
            ReasonerDataMap data = reasoner->extractOwlData(copy);
            for(ReasonerDataMap::value_type pair : data)
                to_ret.insert(pair);
        }
    }
    //Save all the remaining properties (no relations)
    skiros_config::ParamTypes param_types = skiros_config::ParamTypes::getInstance();
    for(skiros_common::ParamMap::value_type pair : copy.properties())
    {
        if(wo->getType(pair.first).find("ObjectProperty")==std::string::npos)
        {
            //FWARN("[Element::toMapStatements] Element '" << copy << "' has data property '" << pair.first << "' with size " << pair.second.size() << ". Only one will appear in the ontology.");
            auto values = pair.second.getValuesStr();
            for(auto value : values)
                to_ret.insert(RelationsPair(pair.first, SPair(value, param_types.getDataTypeStr(pair.second))));
        }
    }
    return to_ret;
}

void Element::removeProperty(std::string key)
{
    skiros_common::ParamMap::iterator it = this->properties_.find(key);
    if(it != this->properties_.end()) properties_.erase(it);
}

void Element::clear()
{
    this->properties_.clear();
    this->id_ = -1;
    this->type_ = "Undefined";
}

void Element::importProperties(const Element& rhs)
{
    for(skiros_common::ParamMap::value_type property : rhs.properties())
    {
        if(this->hasProperty(property.first)) this->properties(property.first) = property.second;
        else this->properties().insert(property);
    }
}

/*TODO: buggy function, to check
    void Element::operator+=(const Element& rhs)
    {
        //Skip ID and Lastupdate
        this->type() = rhs.type();
        this->label() = rhs.label();
        skiros_common::ParamMap::iterator it;
        //I copy all properties except the ones that are not specified
        for(it=rhs.properties().begin();it!=rhs.properties().end();it++)
        {
            if(this->properties().find(it->first)==this->properties().end()) properties().insert(skiros_common::ParamPair(it->first, it->second));
            else
            {
                if(it->second.isSpecified())
                    properties()[it->first] = it->second;
            }
        }
    }*/
//Note: e' = e is differend to e = e'!! The element on the right is the restrictive one. This function check if the element on the left has all the properties of the right one.
bool Element::operator==(const Element& rhs) const
{
    if(rhs.type() == "Unknown" || this->type() == "Unknown") return true;
    if(this->type() != rhs.type()) return false;
    else return true;
    /*      std::map<std::string, skiros_common::Param >::const_iterator it;
      std::map<std::string, skiros_common::Param >::const_iterator it2;
for(it=rhs.properties.begin();it!=rhs.properties.end();it++)
      {
          it2 = this->properties.find(it->second.key());
          if(it2==this->properties.end()) { return false;}
          if(it->second != it2->second) { return false;}
      }*/
    return true;
}

RelationsMap Element::getRelationsWithDetailWrt(skiros_wm::Element &other, std::string reasoner_name)
{
    if(reasoner_name!="")
        current_reasoner_ = reasoner_name;
    ReasonerPtrType reasoner = getDiscreteReasoner(current_reasoner_);
    return reasoner->computeRelations(*this, other);
}

std::set<std::string> Element::getRelationsWrt(skiros_wm::Element &other, std::string reasoner_name)
{
    RelationsMap map = getRelationsWithDetailWrt(other, reasoner_name);
    std::set<std::string> to_ret;
    for(auto e : map)
        to_ret.insert(e.first);
    return to_ret;
}

bool Element::storeData(boost::any any, std::string  set_code, std::string reasoner_name)
{
    if(reasoner_name!="")
        current_reasoner_ = reasoner_name;
    ReasonerPtrType reasoner = getDiscreteReasoner(current_reasoner_);
    return reasoner->storeData(*this, any, set_code);
}

boost::any Element::getData(std::string  get_code, std::string reasoner_name)
{
    if(reasoner_name!="")
        current_reasoner_ = reasoner_name;
    ReasonerPtrType reasoner = getDiscreteReasoner(current_reasoner_);
    return reasoner->getData(*this, get_code);

}

ReasonerDataMap Element::extractOwlData(std::string reasoner_name)
{
    if(reasoner_name!="")
        current_reasoner_ = reasoner_name;
    ReasonerPtrType reasoner = getDiscreteReasoner(current_reasoner_);
    return reasoner->extractOwlData(*this);
}

bool Element::associateReasoner(std::string reasoner_name)
{
    current_reasoner_ = reasoner_name;
    ReasonerPtrType reasoner = getDiscreteReasoner(current_reasoner_);
    if(!reasoner)
        return false;
    reasoner->addProperties(*this);
    return true;
}

void Element::removeReasoner(std::string reasoner_name)
{
    if(current_reasoner_==reasoner_name)
        current_reasoner_="";
    ReasonerPtrType reasoner = getDiscreteReasoner(reasoner_name);
    if(reasoner)
        reasoner->removeProperties(*this);
}

std::set<std::string> Element::getAssociatedReasoners()
{
    std::set<std::string> to_ret;
    try
    {
        if(this->hasProperty(data::DiscreteReasoner))
        {
            for(auto d : this->properties(data::DiscreteReasoner).getValues<std::string>())
                to_ret.insert(d);
        }
    }
    catch(...)
    {
        FERROR("[getAssociatedReasoners] Error while retrieving associated reasoners.");
    }
    return to_ret;
}

//------------- WORLD GRAPH -------------------
std::vector<ChildsPair> WorldGraph::getChildElements(int id, std::string relation)
{
    std::vector<ChildsPair> to_ret;
    std::pair<WorldArcPtrMultimap::iterator, WorldArcPtrMultimap::iterator> ret = active_rel_.equal_range(id);
    for (WorldArcPtrMultimap::iterator it=ret.first; it!=ret.second; ++it)
    {
        if(relation=="" || relation==it->second->predicate()) to_ret.push_back(ChildsPair(it->second->predicate(), nodes_.at(it->second->object_id())));
    }
    return to_ret;
}

void WorldGraph::updateElement(skiros_wm::Element & e, int parent_id, std::string predicate)
{
    nodes_.at(e.id()) = e;
    if(parent_id>=0)
    {
        removeRelation(*passive_rel_.at(e.id()));
        addRelation(RelationType(parent_id, predicate, e.id()));
    }
    e.lastUpdate() = ros::Time::now();
}

int WorldGraph::addElement(skiros_wm::Element & e, int parent_id, std::string predicate, bool is_global)
{
    addElement(e, is_global);
    addRelation(RelationType(parent_id, predicate, e.id()));
    return e.id();
}

void WorldGraph::removeElement(int id)
{
    std::pair<WorldArcPtrMultimap::iterator, WorldArcPtrMultimap::iterator> ret = active_rel_.equal_range(id);
    while (ret.first!=ret.second)
    {
        removeElement(ret.first->second->object_id());
        ret = active_rel_.equal_range(id);
    }
    nodes_.erase(id);
    if(global_element_.find(id)!=global_element_.end())
        global_element_.erase(id);
    auto parent_rel = passive_rel_.find(id);
    if(parent_rel==passive_rel_.end())
        return;
    removeRelation(*parent_rel->second);
}

void WorldGraph::clear()
{
    nodes_.clear();
    active_rel_.clear();
    passive_rel_.clear();
    id_counter_ = 1;
    root_id_ = -1;
}

ExportedGraphType WorldGraph::exportGraph()
{
    ExportedGraphType to_ret;
    for(WorldElementMap::iterator it = nodes_.begin();it!=nodes_.end();it++)
    {
        to_ret.first.push_back(it->second);
    }
    for(WorldArcPtrMap::iterator it = passive_rel_.begin();it!=passive_rel_.end();it++)
    {
        to_ret.second.push_back(*it->second);
    }
    return to_ret;
}

void WorldGraph::importGraph(std::vector<Element> ev, std::vector<RelationType> rv, bool is_global)
{
    if(ev.size()<=0) return;
    root_id_ = ev[0].id();
    for(int i=0;i<ev.size();i++)
        addElement(ev[i], is_global);
    for(int i=0;i<rv.size();i++)
        addRelation(rv[i]);
}

std::string WorldGraph::print(bool verbose)
{
    return printRecursive(getElement(getRootId()), "", verbose);
}

std::string WorldGraph::printRecursive(Element e, std::string indend, bool verbose)
{
    std::stringstream to_ret;
    std::string white_string;
    //Print element
    to_ret << e.printState(indend, verbose) << std::endl;
    //Recursively print all sub tree
    std::vector<ChildsPair> v = this->getChildElements(e.id());
    for(ChildsPair pair : v)
    {
        to_ret << indend+pair.first+"->";
        white_string.clear();
        for(int i=0;i<pair.first.size()+2;i++)white_string.push_back(' ');
        to_ret << printRecursive(pair.second, indend+white_string, verbose);
    }
    return to_ret.str();
}

void WorldGraph::addElement(skiros_wm::Element & e, bool is_global)
{
    if(!is_global || e.id()==-1)
        e.id() = getId();
    else if(nodes_.find(e.id())!=nodes_.end())
    {
        std::stringstream ss;
        ss << "[WorldGraph::addElement] An element with the same id already exist in the graph: " << e.id();
        throw std::invalid_argument(ss.str().c_str());
    }

    if(id_counter_ < e.id())
        id_counter_ = e.id();
    nodes_.insert(WorldElementPair(e.id(), e));
    if(is_global)
        global_element_.insert(e.id());
}

void WorldGraph::addRelation(RelationType rel, bool passive)
{
    if(rel.subject_id()==-1 || rel.object_id()==-1)
    {
        std::stringstream ss;
        ss << "[addRelation] Error while inserting relation: " << rel.subject_id() << "-" << rel.predicate() << "-" << rel.object_id();
        throw std::invalid_argument(ss.str().c_str());
    }
    boost::shared_ptr<RelationType> rel_ptr(new RelationType(rel.subject_id(), rel.predicate(), rel.object_id()));
    active_rel_.insert(WorldArcPtrPair(rel.subject_id(), rel_ptr));
    if(passive)
        passive_rel_[rel.object_id()] = rel_ptr;
}

void WorldGraph::removeRelation(RelationType rel)
{
    std::pair<WorldArcPtrMultimap::iterator, WorldArcPtrMultimap::iterator> ret = active_rel_.equal_range(rel.subject_id());
    for (WorldArcPtrMultimap::iterator it=ret.first;it!=ret.second;)
    {
        if(it->second->object_id() == rel.object_id())
            it = active_rel_.erase(it);
        else
            ++it;
    }
    passive_rel_.erase(passive_rel_.find(rel.object_id()));
}

}
