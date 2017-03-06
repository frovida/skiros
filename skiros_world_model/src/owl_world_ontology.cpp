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

#include "skiros_world_model/owl_world_ontology.h"
#include "skiros_common/logger_sys.h"
#include "skiros_common/utility.h"
#include <boost/foreach.hpp>

namespace skiros_wm
{
namespace owl
{

Node::Node(librdf_node *node)
{
    if(librdf_node_is_resource(node))
    {
        type = LIBRDF_NODE_TYPE_RESOURCE;
        data = (char*)librdf_uri_as_string(librdf_node_get_uri(node));
    }
    else if (librdf_node_is_literal(node))
    {
        type = LIBRDF_NODE_TYPE_LITERAL;
        data = (char*)librdf_node_get_literal_value(node);
        data_type_uri = (char*)librdf_uri_as_string(librdf_node_get_literal_value_datatype_uri(node));
    }
    else if (librdf_node_is_blank(node))
    {
        type = LIBRDF_NODE_TYPE_BLANK;
        data = (char*)librdf_node_get_blank_identifier(node);
    }
    else FERROR("[Node::constructor] Node type not recognized");
}

bool Node::operator==(const Node& rhs) const
{
    return (type==rhs.type) && (data==rhs.data) && (data_type_uri == rhs.data_type_uri);
}

std::ostream& operator<<(std::ostream& os, const Node& obj)
{
    switch(obj.type)
    {
    case LIBRDF_NODE_TYPE_RESOURCE:
        os << "<" << obj.data << ">";
        break;
    case LIBRDF_NODE_TYPE_LITERAL:
        os << "<" << obj.data << ">^^\"" << obj.data_type_uri << "\"";
        break;
    case LIBRDF_NODE_TYPE_BLANK:
        break;
    }
    return os;
}

Ontology::Ontology() : initialized_(false), model_(NULL), storage_(NULL), world_(NULL)
{
}

Ontology::~Ontology()
{
  if(model_)librdf_free_model(model_);
  if(storage_)librdf_free_storage(storage_);
  if(world_)librdf_free_world(world_);
}

//---------------------- Modification ----------------------

/*bool Ontology::changeNodeName(owl::Node old_node, owl::Node new_node)
{

}*/

bool Ontology::addStatement(owl::Node subject, owl::Node predicate, owl::Node object)
{
    librdf_node * rdf_subject = getRdfNode(subject);
    librdf_node * rdf_predicate = getRdfNode(predicate);
    librdf_node * rdf_object = getRdfNode(object);
    //std::cout << librdf_node_to_string(rdf_subject) << librdf_node_to_string(rdf_predicate) << librdf_node_to_string(rdf_object) << std::endl;
    if(rdf_subject && rdf_predicate && rdf_object)
    {
        setStatement(rdf_subject, rdf_predicate, rdf_object, true);
        return true;
    }
    else return false;
}

bool Ontology::removeContextStatements(owl::Node context)
{
    librdf_node * rdf_context = getRdfNode(context);
    if(!rdf_context) return false;
    //Remove active statements
    librdf_statement * partial_statement=librdf_new_statement(world_);
    librdf_statement_set_subject(partial_statement, rdf_context);
    librdf_stream * stream = getStatements(partial_statement);
    librdf_free_statement(partial_statement);
    if(stream)
    {
        //Remove all
        while(!librdf_stream_end(stream))
        {
         librdf_statement *statement=librdf_stream_get_object(stream);
         if(!statement) break;
         librdf_model_remove_statement(model_, statement);
         librdf_stream_next(stream);
        }
        librdf_free_stream(stream);
    }
    //Remove passive statements
    rdf_context = getRdfNode(context);
    partial_statement=librdf_new_statement(world_);
    librdf_statement_set_object(partial_statement, rdf_context);
    stream = getStatements(partial_statement);
    librdf_free_statement(partial_statement);
    if(stream!=NULL)
    {
        //Remove all
        while(!librdf_stream_end(stream))
        {
         librdf_statement *statement=librdf_stream_get_object(stream);
         if(!statement) break;
         librdf_model_remove_statement(model_, statement);
         librdf_stream_next(stream);
        }
        librdf_free_stream(stream);
    }
    //Finish
    return true;
}

bool Ontology::removeStatement(owl::Node subject, owl::Node predicate, owl::Node object)
{
    librdf_node * rdf_subject = getRdfNode(subject);
    librdf_node * rdf_predicate = getRdfNode(predicate);
    librdf_node * rdf_object = getRdfNode(object);
    if(!rdf_subject || !rdf_predicate || !rdf_object) return false;
    setStatement(rdf_subject, rdf_predicate, rdf_object, false);
    return true;
}

//---------------------- Queries ----------------------

std::list<std::string>  Ontology::getAllObjProp()
{
    std::list<std::string> to_ret;
    //Get all declared object relations
    librdf_iterator * it = getSources(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_OBJPROP));
    if(it)
    {
        std::string temp;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          temp = (char *)librdf_uri_as_string(librdf_node_get_uri(target));
          to_ret.push_back(temp);
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    return to_ret;
}

std::list<std::string>  Ontology::getAllDataProp()
{
    std::list<std::string> to_ret;
    //Get all declared object relations
    librdf_iterator * it = getSources(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_DATATPROP));
    if(it)
    {
        std::string temp;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          temp = (char *)librdf_uri_as_string(librdf_node_get_uri(target));
          to_ret.push_back(temp);
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    return to_ret;
}

std::list<std::string>  Ontology::getAllIndividuals()
{
    std::list<std::string> to_ret;
    librdf_query_results* results = query("SELECT ?x    { ?x   <http://www.w3.org/1999/02/22-rdf-syntax-ns#type> <http://www.w3.org/2002/07/owl#NamedIndividual> }");
    while(!librdf_query_results_finished(results))
    {
       const char **names=NULL;
       librdf_node* values[10];
       if(librdf_query_results_get_bindings(results, &names, values))break;
       if(names)
       {
           if(values[0])
           {
             librdf_uri * uri;
             if(librdf_node_is_blank(values[0])) throw std::runtime_error("[Ontology::getAllIndividuals] librdf_node_is_blank");
             if(librdf_node_is_literal(values[0])) throw std::runtime_error("[Ontology::getAllIndividuals] librdf_node_is_literal");
             if(librdf_node_is_resource(values[0]))
             {
                uri = librdf_node_get_uri(values[0]);
                std::string individual = (char *)librdf_uri_as_string(uri);
                to_ret.push_back(individual);
             }
             librdf_free_node(values[0]);
           }
       }
       librdf_query_results_next(results);
     }
     librdf_free_query_results(results);
    return to_ret;
}

std::list<std::string>  Ontology::getAllClasses()
{
    std::list<std::string> to_ret;
    //Get all classes and store them
    librdf_iterator * it = NULL;
    it = getSources(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_CLASS));
    if(it)
    {
        std::string temp;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          if(librdf_node_is_blank(target)) temp = (char*)librdf_node_get_blank_identifier(target);
          else if(librdf_node_is_resource(target)) temp = (char *)librdf_uri_as_string(librdf_node_get_uri(target));
          else if(librdf_node_is_literal(target)) throw std::invalid_argument("[getAllClasses] Unexpected node literal type");
          to_ret.push_back(temp);
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    return to_ret;
}

std::set<std::string>  Ontology::getSubClassesS(std::string class_name)
{
    std::set<std::string> set;
    std::list<std::string> list = getSubClassesL(class_name);
    for(std::string str : list)
    {
        set.insert(str);
    }
    return set;
}

std::list<std::string>  Ontology::getSubClassesL(std::string class_name)
{
    librdf_node * rdf_subject = getRdfNode(Node(LIBRDF_NODE_TYPE_RESOURCE, class_name));
    if(!rdf_subject) return std::list<std::string>();
    return getSubClasses(rdf_subject);
}

std::set<std::string>  Ontology::getSubPropertiesS(std::string prop_name)
{
    std::set<std::string> set;
    std::list<std::string> list = getSubPropertiesL(prop_name);
    BOOST_FOREACH(std::string str, list)
    {
        set.insert(str);
    }
    return set;
}

std::list<std::string> Ontology::getSubPropertiesL(std::string prop_name)
{
    librdf_node * rdf_subject = getRdfNode(Node(LIBRDF_NODE_TYPE_RESOURCE, prop_name));
    std::list<std::string> list;
    if(!rdf_subject) return list;
    return getSubProperties(rdf_subject);
}

bool Ontology::isSubProperty(std::string child, std::string parent)
{
    std::set<std::string> set = getSubPropertiesS(parent);
    return set.find(getUri(child))!=set.end();
}

bool Ontology::isType(owl::Node subject, owl::Node type)
{
    librdf_node * rdf_type = getRdfNode(type);
    librdf_node  * rdf_target = getTarget(subject, owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE));
    //FINFO((const char*)librdf_uri_as_string(librdf_node_get_uri(rdf_target)) << " equal to " << (const char*) librdf_uri_as_string(librdf_node_get_uri(rdf_type)) << " " << librdf_node_equals(rdf_target, rdf_type));
    bool to_ret = librdf_node_equals(rdf_target, rdf_type);
    librdf_free_node(rdf_target);
    librdf_free_node(rdf_type);
    return to_ret;
}

std::list<std::string>  Ontology::getBranchIndividuals(owl::Node node)
{
    librdf_node * rdf_subject = getRdfNode(node);
    if(!rdf_subject) return std::list<std::string>();
    std::list<std::string> to_ret = getBranchIndividuals(rdf_subject);
    librdf_free_node(rdf_subject);
    return to_ret;
}

librdf_iterator * Ontology::getSources(owl::Node predicate, owl::Node object)
{
    librdf_node * rdf_object = getRdfNode(object);
    librdf_node * rdf_predicate = getRdfNode(predicate);
    if(rdf_object && rdf_predicate)
    {
        librdf_iterator * to_ret = librdf_model_get_sources(model_, rdf_predicate, rdf_object);
        librdf_free_node(rdf_object);librdf_free_node(rdf_predicate);
        return to_ret;
    }
    else return NULL;
}

librdf_node * Ontology::getTarget(owl::Node subject, owl::Node predicate)
{
    librdf_node * rdf_subject = getRdfNode(subject);
    librdf_node * rdf_predicate = getRdfNode(predicate);
    if(rdf_subject && rdf_predicate)
    {
        librdf_node * to_ret = librdf_model_get_target(model_, rdf_subject, rdf_predicate);
        librdf_free_node(rdf_subject);librdf_free_node(rdf_predicate);
        return to_ret;
    }
    else return NULL;
}

ResultList Ontology::getClassToConstraintsMap()
{
    librdf_query_results * results =
            query("SELECT ?class ?constraint WHERE {?constraint rdf:type <http://www.w3.org/2002/07/owl#Restriction> . ?class <http://www.w3.org/2000/01/rdf-schema#subClassOf> ?constraint }");
    return results2list(results, 2);
}

ResultList Ontology::getConstraintsDescription()
{
    librdf_query_results * results =
            query("SELECT ?constraint ?y ?z WHERE {?constraint rdf:type <http://www.w3.org/2002/07/owl#Restriction> . ?constraint ?y ?z. }");
    return results2list(results, 3);
}

/*std::vector<std::string> querySubClasses(owl::Node node)
{
    if(node.type != LIBRDF_NODE_TYPE_RESOURCE) return std::vector<std::string>();
    librdf_node * rdf_node = getRdfNode(node);
    if(!rdf_node) return std::vector<std::string>();
    librdf_uri * uri = librdf_node_get_uri(rdf_node);
    char * str = (char *)librdf_uri_as_string(uri);
    std::stringstream ss;
    //TODO: create a function to format automatically
    ss << "SELECT ?x WHERE {"
       << " ?x " << " <" << std_uri::RDFS_SUBCLASS << "> " << "<" << librdf_uri_as_string(uri) << "> " << " ."
       << " ?x <" << std_uri::TYPE << "> <" << std_uri::OWL_CLASS << ">}";
    FINFO("[Model::queryType]" << ss.str());
    librdf_query_results* results = query(ss.str().c_str());
    librdf_node * to_ret;
    const char **names=NULL;
    librdf_node* values[10];
    if(!librdf_query_results_get_bindings(results, &names, values))
    {
        if(names)
        {
            if(values[0])
            {
                to_ret = values[0];
            }
        }
    }
    if(to_ret) str = (char *)librdf_uri_to_string(librdf_node_get_uri(to_ret));
    else str = NULL;
    librdf_free_query_results(results);
    return str;
}*/
//FREE THE POINTER
ResultList Ontology::results2list(librdf_query_results * results, int num_of_var)
{
    if(!results) return ResultList();
    ResultList result_list;
    std::vector<librdf_node*> result;
    result.resize(num_of_var);
    librdf_node* values[num_of_var];
    while(!librdf_query_results_finished(results))
    {
        for(int i=0;i<num_of_var;i++)
            values[i]=NULL;
        const char **names=NULL;
        if(librdf_query_results_get_bindings(results, &names, values))
            break;

        if(names)
        {
         for(int i=0; i<num_of_var; i++) result[i] = values[i];
        }
        result_list.push_back(result);
        librdf_query_results_next(results);
    }
    librdf_free_query_results(results);
    return result_list;
}
//FREE THE POINTER
std::string Ontology::results2string(librdf_query_results * results, bool cut_prefix)
{
    if(!results) return std::string();
    std::stringstream ss;
    std::vector<librdf_node*> result;
    while(!librdf_query_results_finished(results))
    {
       const char **names=NULL;
       librdf_node* values[10];
       for(int i=0; i<=10; i++)
           values[i] = NULL;
       if(librdf_query_results_get_bindings(results, &names, values))break;

       if(names)
       {
         for(int i=0; values[i]!=NULL; i++)
         {
              std::string str;
              if(librdf_node_is_blank(values[i])) str = (char*)librdf_node_get_blank_identifier(values[i]);
              else if(librdf_node_is_literal(values[i])) str =  (char *)librdf_node_get_literal_value(values[i]);
              else if(librdf_node_is_resource(values[i])) str = (char *)librdf_uri_as_string(librdf_node_get_uri(values[i]));
              if(cut_prefix)
                str = str.substr(str.find("#")+1);
              else
              {
                  std::string prefix = str.substr(0, str.find("#"));
                  if(hasPrefix(prefix))
                  {
                      std::string short_prefix = getShortPrefix(prefix);
                      str.replace(str.find(prefix), prefix.length()+1, short_prefix+":");
                  }
              }
              ss << str << " ";
         }
       }
       ss << "\n";
       librdf_query_results_next(results);
    }
    librdf_free_query_results(results);
    return ss.str();
}

ResultList Ontology::getContextStatementsOld(owl::Node context)
{
    librdf_node * rdf_node = getRdfNode(context);
    std::stringstream ss;
    if(!rdf_node)
    {
        ss << "[owl::Ontology] Error while allocating rdf node.";
        throw std::runtime_error(ss.str().c_str());
    }
    librdf_uri * uri = librdf_node_get_uri(rdf_node);
    ss << "SELECT ?predicate ?object WHERE {<" << librdf_uri_as_string(uri) << "> ?predicate ?object .}";
    librdf_query_results * results = query(ss.str().c_str());
    librdf_free_node(rdf_node);
    return results2list(results, 2);
}

std::vector<Statement> Ontology::getContextStatements(owl::Node context, int filter)
{
    std::vector<Statement> to_ret;
    librdf_node * rdf_context = getRdfNode(context);
    if(!rdf_context) return to_ret;
    //Active statements
    if(filter==0 || filter == 1)
    {
        librdf_statement * partial_statement=librdf_new_statement(world_);
        librdf_statement_set_subject(partial_statement, rdf_context);
        librdf_stream * stream = getStatements(partial_statement);
        librdf_free_statement(partial_statement);
        if(stream!=NULL)
        {
            while(!librdf_stream_end(stream))
            {
                librdf_statement *statement=librdf_stream_get_object(stream);
                if(!statement) break;
                to_ret.push_back(Statement(statement));
                librdf_stream_next(stream);
            }
            librdf_free_stream(stream);
        }
    }
    //Passive statements
    if(filter==0 || filter == 2)
    {
        rdf_context = getRdfNode(context);
        librdf_statement * partial_statement=librdf_new_statement(world_);
        librdf_statement_set_object(partial_statement, rdf_context);
        librdf_stream * stream = getStatements(partial_statement);
        librdf_free_statement(partial_statement);
        if(stream!=NULL)
        {
            while(!librdf_stream_end(stream))
            {
                librdf_statement *statement=librdf_stream_get_object(stream);
                if(!statement) break;
                to_ret.push_back(Statement(statement));
                librdf_stream_next(stream);
            }
            librdf_free_stream(stream);
        }
    }
    return to_ret;
}

std::string Ontology::getUpperClass(owl::Node node)
{
    if(node.type != LIBRDF_NODE_TYPE_RESOURCE) return NULL;
    librdf_node * rdf_node = getRdfNode(node);
    if(!rdf_node) return NULL;
    librdf_uri * uri = librdf_node_get_uri(rdf_node);
    char * str = (char *)librdf_uri_as_string(uri);
    std::stringstream ss;
    //TODO: create a function to format automatically
    ss << "SELECT ?x WHERE {<" << librdf_uri_as_string(uri) << "> <" << std_uri::RDFS_SUBCLASS << "> ?x . ?x <" << std_uri::TYPE << "> <" << std_uri::OWL_CLASS << ">}";
    librdf_query_results* results = query(ss.str().c_str());
    librdf_node * to_ret;
    const char **names=NULL;
    librdf_node* values[10];
    if(!librdf_query_results_get_bindings(results, &names, values))
    {
        if(names)
        {
            if(values[0])
            {
                to_ret = values[0];
            }
        }
    }
    if(to_ret) str = (char *)librdf_uri_to_string(librdf_node_get_uri(to_ret));
    else str = NULL;
    librdf_free_query_results(results);
    librdf_free_node(rdf_node);
    return std::string(str);
}

std::string Ontology::getIndividualType(owl::Node node)
{
    if(node.type != LIBRDF_NODE_TYPE_RESOURCE)
    {
        FERROR("[owl::ontology::queryIndividualType] Error, only resource nodes in input.");
        return "";
    }
    librdf_node * rdf_node = getRdfNode(node);
    if(rdf_node == NULL)return "";
    if(!this->isNodeInModel(rdf_node)) return "";
    std::string str;
    std::stringstream ss;
    //TODO: create a function to format automatically
    //TODO: make this better...
    ss << "SELECT ?x WHERE {<" << librdf_uri_as_string(librdf_node_get_uri(rdf_node)) << "> <" << std_uri::TYPE << "> ?x . ?x <" << std_uri::TYPE << "> <" << std_uri::OWL_CLASS << ">}";
    //FINFO("[Model::queryType]" << ss.str());
    librdf_query_results* results = query(ss.str().c_str());
    librdf_node * to_ret;
    const char **names=NULL;
    librdf_node* values[10];
    if(!librdf_query_results_get_bindings(results, &names, values))
    {
        if(names && values[0])
        {
            to_ret = values[0];
        }
    }
    if(to_ret) str = (char *)librdf_uri_to_string(librdf_node_get_uri(to_ret));
    else str = "";
    librdf_free_query_results(results);
    librdf_free_node(rdf_node);
    return str;
}

librdf_query_results * Ontology::getEquivalentIndividuals(std::string uri)
{
    uri = getUri(uri);
    std::stringstream ss;
    ss << "SELECT ?x WHERE { {?x <http://www.w3.org/2002/07/owl#sameAs> <" << uri << ">.  } UNION { <" << uri << "> <http://www.w3.org/2002/07/owl#sameAs> ?x. } }";
    librdf_query_results * results = query(ss.str().c_str());
    return results;
}

librdf_stream * Ontology::getStatements(owl::Node sub, owl::Node pred, owl::Node obj)
{
    //INIT
    librdf_statement * partial_statement=librdf_new_statement(world_);
    if(sub.data!="")
        librdf_statement_set_subject(partial_statement, getRdfNode(sub));
    if(pred.data!="")
        librdf_statement_set_predicate(partial_statement, getRdfNode(pred));
    if(obj.data!="")
        librdf_statement_set_object(partial_statement, getRdfNode(obj));

    //MAIN
    librdf_stream * stream = librdf_model_find_statements(model_, partial_statement);
    librdf_free_statement(partial_statement);
    return stream;
}

librdf_query_results *  Ontology::query(const char * query_string)
{
    std::string temp;
    BOOST_FOREACH(PrefixMap::value_type pair, prefix_map_)
    {
        temp += "PREFIX " + pair.second + ": <" + pair.first + "#> \n";
    }
    temp += query_string;
    librdf_query * query = librdf_new_query(world_, "sparql", NULL, (const unsigned char*)temp.c_str(), NULL);
    if(!query)
    {
        FERROR("[OwlOntology]: librdf_new_query failed to return");
        return NULL;
    }
    librdf_query_results * results = librdf_query_execute(query, model_);
    librdf_free_query(query);
    return results;
}


bool Ontology::containStatement(owl::Node sub, owl::Node pred, owl::Node obj)
{
    std::stringstream ss;
    ss << "SELECT * WHERE {" << sub << " " << pred << " " << obj << ".}";
    //FINFO(ss.str());
    librdf_query_results * res = query(ss.str().c_str());
    bool to_ret = false;
    if(res)
    {
        if(!librdf_query_results_finished(res))
            to_ret = true;
        librdf_free_query_results(res);
    }
    return to_ret;
}

//-------------------- Load and visualization ----------------------

void Ontology::init(std::string default_uri, std::string storage_name, PrefixMap prefixes)
{
    //------- Init redland with raptor -------
    //Note: because redland is MODULAR it has to load all wanted modules in this step
    world_ = librdf_new_world();
    librdf_world_open(world_);
    raptor_w_ptr_ = librdf_world_get_raptor(world_);
    std::string temp("new='yes',hash-type='bdb',dir='");
    temp = temp + getWorkspacePath() + "db/'";
    storage_=librdf_new_storage(world_, "hashes", (const char *) storage_name.c_str(), (const char *) temp.c_str());
    if(!storage_) throw std::runtime_error("[OwlOntology]: Failed to create new storage");
    model_ = librdf_new_model(world_, storage_, NULL);
    if(!model_) throw std::runtime_error("[OwlOntology]: Failed to create model");
    default_uri_ = default_uri;
    initialized_ = true;
    for(auto pair : prefixes)
        addPrefix(pair);
}

void Ontology::loadMainOntology(std::string filename, std::string storage_name)
{
    if(!initialized_)
        init("", storage_name, PrefixMap());
    //Open document --------------
    std::string path = "file:"+filename;
    //------- Load an ontology -------
    librdf_parser* parser;
    char *parser_name=NULL;
    parser=librdf_new_parser(world_, parser_name, NULL, NULL);
    if(!parser) throw std::runtime_error("[owl::Ontology]: Failed to create new parser");
    //Assign an uri to a file
    librdf_uri *uri;
    uri=librdf_new_uri(world_, (const unsigned char*)path.c_str());
    if(!uri) throw std::runtime_error("[owl::Ontology]: Failed to create URI");
    FDEBUG("[loadOntology]: Parsing URI " << librdf_uri_as_string(uri));
    /* PARSE the URI as RDF/XML*/
    if(librdf_parser_parse_into_model(parser, uri, uri, model_)) throw std::runtime_error("[owl::Ontology]: Failed to parse RDF into model");
    for(int i=1;i<librdf_parser_get_namespaces_seen_count(parser); i++)
    {
      std::string temp((const char*)librdf_uri_as_string(librdf_parser_get_namespaces_seen_uri(parser, i)));
      temp.erase(--temp.end(), temp.end());
      addPrefix(std::pair<std::string, std::string>(temp,librdf_parser_get_namespaces_seen_prefix(parser, i)));
      FINFO("[loadMainOntology]: I add: " << librdf_parser_get_namespaces_seen_prefix(parser, i) << " - " << temp );
    }
    librdf_node * ontology = librdf_model_get_source(model_, LIBRDF_MS_type(world_), librdf_new_node_from_uri_string(world_, (const unsigned char*)std_uri::OWL_ONTOLOGY.c_str()));
    if(ontology)
    {
        default_uri_ = (char *)librdf_uri_as_string(librdf_node_get_uri(ontology));
        librdf_free_node(ontology);
        auto last_slash = default_uri_.find_last_of('/');
        std::string short_prefix = default_uri_.substr(last_slash+1, default_uri_.length()-1);
        this->addPrefix(std::pair<std::string, std::string>(default_uri_, short_prefix));
        FINFO("[loadMainOntology]: I add: " << default_uri_);
    }
    librdf_free_parser(parser);
    librdf_free_uri(uri);
    initialized_ = true;
}

void Ontology::importOntology(owl::Ontology & other_ontology)
{
     librdf_stream * stream = NULL;
     stream = librdf_model_as_stream(other_ontology.getModel());
     if(stream)
     {
        librdf_model_add_statements(model_, stream);
        librdf_free_stream(stream);
     }
}

void Ontology::loadSubOntology(std::string filename)
{
    if(!initialized_) throw std::runtime_error("[owl::importOntology]: The ontology is not initialized, load a main ontology first.");
    std::string path = "file:"+filename;
    //------- Load an ontology -------
    librdf_parser* parser;
    parser=librdf_new_parser(world_, NULL, NULL, NULL);
    if(!parser) throw std::runtime_error("[owl::importOntology]: Failed to create new parser");
    //Assign an uri to a file
    librdf_uri *uri;
    uri=librdf_new_uri(world_, (const unsigned char*)path.c_str());
    if(!uri) throw std::runtime_error("[owl::importOntology]: Failed to create URI");
    librdf_stream * stream = librdf_parser_parse_as_stream(parser, uri, NULL);
    if(stream==NULL) throw std::runtime_error("[owl::importOntology]: Failed to create stream");
    /*for(int i=1;i<librdf_parser_get_namespaces_seen_count(parser); i++)
    {
      std::string temp((const char*)librdf_uri_as_string(librdf_parser_get_namespaces_seen_uri(parser, i)));
      temp.erase(--temp.end(), temp.end());
      addPrefix(std::pair<std::string, std::string>(temp,librdf_parser_get_namespaces_seen_prefix(parser, i)));
      FINFO("[loadSubOntology]: I add: " << librdf_parser_get_namespaces_seen_prefix(parser, i) << " - " << temp );
    }*/
    librdf_iterator*  it = librdf_model_get_sources(model_, LIBRDF_MS_type(world_), librdf_new_node_from_uri_string(world_, (const unsigned char*)std_uri::OWL_ONTOLOGY.c_str()));
    while(!librdf_iterator_end(it))
    {
        librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
        std::string uri  = (char *)librdf_uri_as_string(librdf_node_get_uri(target));
        auto last_slash = uri.find_last_of('/');
        std::string short_prefix = uri.substr(last_slash+1, uri.length()-1);
        this->addPrefix(std::pair<std::string, std::string>(uri, short_prefix));
        FINFO("[loadSubOntology]: I add: " << uri );
        librdf_iterator_next(it);
    }
    librdf_model_add_statements(model_, stream);
    librdf_free_stream(stream);
    librdf_free_parser(parser);
}

void Ontology::printOntology()
{
    FILE * file = tmpfile();
    std::string save_path = getWorkspacePath() + "ontology_log.txt";
    FILE * f_to_print = fopen(save_path.c_str(), "w");
    if(file==NULL) return;
    raptor_iostream* iostr = raptor_new_iostream_to_file_handle(raptor_w_ptr_, file);
    librdf_model_write(model_, iostr);
    //rewind(file);
    //format(file, stdout);
    rewind(file);
    format(file, f_to_print);
    fclose(f_to_print);
    fclose(file);
    raptor_free_iostream(iostr);
    std::cout << "RDF store output dumped in: " << save_path << std::endl ;
    std::cout << "Default uri is: " << default_uri_ << std::endl;
}

void Ontology::saveOntology(std::string filename)
{
    //filename = getWorkspacePath() + filename;
    FILE * f = std::fopen(filename.c_str() , "w");
    librdf_serializer* serializer=librdf_new_serializer(world_, "rdfxml", NULL, NULL);
    librdf_uri* base_uri=librdf_new_uri(world_, (const unsigned char*) default_uri_.c_str());
    librdf_serializer_serialize_model_to_file_handle(serializer, f, base_uri, model_);
    librdf_free_serializer(serializer);
    librdf_free_uri(base_uri);
    std::fclose(f);
}

std::string Ontology::getWorkspacePath()
{
  if(wrkspc_path_.empty()) wrkspc_path_ = skiros_common::utility::getSkirosSaveDirectory()+"owl/";
  return wrkspc_path_;
}

librdf_node * Ontology::getRdfNode(owl::Node node)
{
    if(node.data.find("#")==std::string::npos && node.type==LIBRDF_NODE_TYPE_RESOURCE) node.data = getUri(node.data);
    if(node.data_type_uri!="" && node.data_type_uri.find("#")==std::string::npos) node.data_type_uri = getUri(node.data_type_uri);
    switch(node.type)
    {
    case LIBRDF_NODE_TYPE_RESOURCE:
        return librdf_new_node_from_uri_string(world_, (const unsigned char*)node.data.c_str());
        break;
    case LIBRDF_NODE_TYPE_LITERAL:
        if(node.data_type_uri=="")return librdf_new_node_from_literal(world_, (const unsigned char*)node.data.c_str(), NULL, 0);
        else return librdf_new_node_from_typed_literal(world_,
                                                       (const unsigned char*) node.data.c_str(),
                                                       NULL,
                                                       librdf_new_uri(world_, (const unsigned char*)node.data_type_uri.c_str()));
        break;
    case LIBRDF_NODE_TYPE_BLANK:
        return librdf_new_node_from_blank_identifier(world_, (const unsigned char*)node.data.c_str());
        break;
    default:
        throw std::invalid_argument("[owl::Ontology]: Node type not found");
        break;
    }
}

std::string Ontology::getUri(std::string resource_name)
{
    //If the input is already an uri, returns
    if(resource_name.find("#")!=std::string::npos) return resource_name;
    //If the input has no prefix, add the default uri
    if(resource_name.find(":")==std::string::npos && resource_name!="") resource_name = this->getDefaultUri()+"#"+resource_name;
    else
    {
        //If the input has prefix, this is sobstituted with the correspondent uri
        for(PrefixMap::iterator it=prefix_map_.begin();it!=prefix_map_.end();it++)
        {
            if(resource_name.find(it->second)!=std::string::npos)
            {
                resource_name.replace(resource_name.find(it->second), it->second.length()+1, it->first+"#");
                return resource_name;
            }
        }
        std::stringstream ss;
        ss << "[Ontology::getUri] Cannot replace prefix for: " << resource_name << ". Available prefixes are: ";
        for(auto pair : prefix_map_)
            ss << pair.second << ", ";
        FWARN(ss.str());
    }
    return resource_name;
}

//--------------------- Prefix ----------------------

void Ontology::addPrefix(std::pair<std::string, std::string> prefix)
{
    if(prefix_map_.find(prefix.first)==prefix_map_.end())
    {
        prefix_map_.insert(prefix);
        //For the world model I store the opposite
        prefix_map_inverse_.insert(std::pair<std::string, std::string>(prefix.second, prefix.first));
    }
}

bool Ontology::hasPrefix(std::string prefix)
{
    return prefix_map_.find(prefix) != prefix_map_.end();
}

std::string Ontology::getShortPrefix(std::string prefix)
{
    return prefix_map_[prefix];
}

//--------------------- Private ----------------------

void Ontology::format(FILE* in, FILE * out)
{
  char str[300];
  if(fgets(str, 300, in)!=NULL);
  while(!feof(in))
  {
    std::string temp(str);
    for(PrefixMap::iterator it=prefix_map_.begin();it!=prefix_map_.end();it++)
    {
      while(temp.find(it->first) != std::string::npos)
      {
        temp.replace(temp.find(it->first), it->first.length(), it->second);
      }
    }
    fprintf(out, "%s", temp.c_str());
    if(fgets(str, 300, in));
  }
}

bool Ontology::isNodeInModel(librdf_node * node)
{
    //Note: i check only the subject. Not 100% sure can be enough to state a presence.
    librdf_statement * statement=librdf_new_statement(world_);
    librdf_statement_set_subject(statement,
                                 node);
    librdf_stream * stream = librdf_model_find_statements(model_, statement);
    bool to_ret = true;
    if(!stream) to_ret = false;
    else
    {
        if(librdf_stream_end(stream)) to_ret = false;
        else
        {
            statement=librdf_stream_get_object(stream);
            if(!statement) to_ret = false;
        }
    }
    librdf_free_statement(statement);
    librdf_free_stream(stream);
    return to_ret;
}

void Ontology::setStatement(librdf_node * subject, librdf_node * predicate, librdf_node * object, bool add)
{
  librdf_statement *statement;
  //------- Add a statement -------
  statement=librdf_new_statement_from_nodes(world_, subject, predicate, object);
  if(!add) librdf_model_remove_statement(model_, statement);
  else librdf_model_add_statement(model_, statement);
  /* Free what we just used to add to the model - now it should be stored */
  librdf_free_statement(statement);
}

std::list<std::string> Ontology::getSub(librdf_node * node, librdf_node * subOf)
{
    std::list<std::string> to_ret;
    //Goes recursively into all subClasses
    librdf_iterator * it = NULL;
    it = librdf_model_get_sources(model_, subOf, node);
    if(it)
    {
        std::list<std::string> temp1;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          temp1 = getSub(target, subOf);
          to_ret.insert(to_ret.end(), temp1.begin(), temp1.end());
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    std::string temp = (char *)librdf_uri_as_string(librdf_node_get_uri(node));
    to_ret.push_back(temp);
    return to_ret;
}

std::list<std::string>  Ontology::getBranchIndividuals(librdf_node * node)
{
    std::list<std::string> to_ret;
    //Get all individuals and store them
    librdf_iterator * it = NULL;
    it = librdf_model_get_sources(model_, LIBRDF_MS_type(world_), node);
    if(it)
    {
        std::string temp;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          temp = (char *)librdf_uri_as_string(librdf_node_get_uri(target));
          to_ret.push_back(temp);
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    //Goes recursively into all subClasses
    it = librdf_model_get_sources(model_, LIBRDF_S_subClassOf(world_), node);
    if(it)
    {
        std::list<std::string> temp1;
        while(!librdf_iterator_end(it))
        {
          librdf_node *target=(librdf_node*)librdf_iterator_get_object(it);
          temp1 = getBranchIndividuals(target);
          to_ret.insert(to_ret.end(), temp1.begin(), temp1.end());
          librdf_iterator_next(it);
        }
        librdf_free_iterator(it);
    }
    else return to_ret;
}

}
}
