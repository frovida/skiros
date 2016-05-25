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

#ifndef OWL_WORLD_ONTOLOGY_H
#define OWL_WORLD_ONTOLOGY_H

#include <string>
#include <map>
#include <set>
#include <list>
#include <vector>
#include <exception>      // std::exception
#include <ostream>
#include <redland.h>
#include "skiros_world_model/std_uris.h"

namespace skiros_wm
{
namespace owl
{
    //! \brief An RDF triple store node. This is a wrapper for an librdf_node
    class Node
    {
    public:
        Node(librdf_node_type typeIn, std::string dataIn, std::string dataTypeUri=""): type(typeIn), data(dataIn), data_type_uri(dataTypeUri) {}
        Node(librdf_node * node);

        bool operator==(const Node& rhs) const;

        friend std::ostream& operator<<(std::ostream& os, const Node& obj);
        librdf_node_type type;
        std::string data;
        std::string data_type_uri;
    };

    //! \brief A RDF triple store statement. This is a wrapper for a librdf_statement
    struct Statement
    {
        Statement(Node subject_in, Node predicate_in, Node object_in) : subject(subject_in), predicate(predicate_in), object(object_in) {}
        Statement(librdf_statement * statement) :   subject(Node(librdf_statement_get_subject(statement))),
            predicate(Node(librdf_statement_get_predicate(statement))),
            object(Node(librdf_statement_get_object(statement))) {}

        friend std::ostream& operator<<(std::ostream& os, const Statement& obj)
        {
            os << obj.subject << " " << obj.predicate << " " << obj.object;
            return os;
        }

        Node subject;
        Node predicate;
        Node object;
    };

    typedef std::map<std::string, std::string> PrefixMap;
    typedef std::list<std::vector<librdf_node*> > ResultList;
    /*!
     * \brief  This class offers API to load an OWL ontology in a RDF triple store and interact with it. The store is by default loaded in a in-memory Berkeley database optimized with hashed search.
     *
     * The class is implemented using the open-source library redland. Many convertions are realized to make the not intuitive C redland interface more Cpp friendly.
     *
     */
    class Ontology
    {
    public:
        Ontology();
        ~Ontology();
        /*!
         * \brief Initialize the database
         */
        void init(std::string default_uri, std::string storage_name);
        //---------- Convertion -----------
        inline Statement getCppStatement(librdf_statement * stmnt)
        {return Statement(Node(librdf_statement_get_subject(stmnt)), Node(librdf_statement_get_predicate(stmnt)), Node(librdf_statement_get_object(stmnt)));}
        //---------- Load and visualization -----------
        /*!
         * \brief loadOntology from file. The ontology URI will be used as base URI (default_uri_). Automatically calls init() if not called already
         * \param filename the file name (located in skiros/owl folder)
         * \param storage_name the name to assign to the database
         */
        void loadMainOntology(std::string filename, std::string storage_name);

        //TODO: load ontology from URLs
        /*!
         * \brief loadSubOntology from file load an ontology subordinated to the main one
         * \param filename the complete path to the file
         */
        void loadSubOntology(std::string filename);

        /*!
         * \brief save the whole ontology to a file
         * \param filename the complete path to the file
         */
        void saveOntology(std::string filename);
        /*!
         * \brief importOntology,import concepts from another ontology
         * \param other_ontology the ontology to import
         */
        void importOntology(Ontology &other_ontology);
        //! \brief print the ontology to stdout
        void printOntology();

        //---------- Modification -----------
        //! \brief Add a statement
        bool addStatement(owl::Node subject, owl::Node predicate, owl::Node object);
        //! \brief Remove all statements (passive and active) related to an object
        bool removeContextStatements(owl::Node context);
        //! \brief Remove a statement
        bool removeStatement(owl::Node subject, owl::Node predicate, owl::Node object);

        //---------- Get -----------
        inline bool isDeclaredObjProp(std::string uri)
        {
            return containStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, getUri(uri)), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_OBJPROP));
        }
        inline bool isDeclaredDataProp(std::string uri)
        {
            return containStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, getUri(uri)), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_DATATPROP));
        }
        inline bool isDeclaredIndividual(std::string uri)
        {
            return containStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, getUri(uri)), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_INDIVIDUAL));
        }
        inline bool isDeclaredClass(std::string uri)
        {
            return containStatement(owl::Node(LIBRDF_NODE_TYPE_RESOURCE, getUri(uri)), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::TYPE), owl::Node(LIBRDF_NODE_TYPE_RESOURCE, std_uri::OWL_CLASS));
        }
        //! \brief Return all defined object properties with extended URI
        std::list<std::string>  getAllObjProp();
        //! \brief Return all defined data type properties with extended URI
        std::list<std::string>  getAllDataProp();
        //! \brief Return all defined classes with extended URI
        std::list<std::string>  getAllClasses();
        //! \brief Return all defined individuals with extended URI
        std::list<std::string>  getAllIndividuals();
        //! \brief Return a list of equivalent individuals (checking statement owl:sameAs)
        librdf_query_results *getEquivalentIndividuals(std::string uri);
        /*!
         * \brief Return a set of all sub-classes
         * \param prop_name, the root property. If no prefix is used the default_uri_ prefix is assigned
         * \return a set of all classes in the subtree (not ordered)), with extended URI
         */
        std::set<std::string>  getSubClassesS(std::string class_name);
        /*!
         * \brief Return a list of all sub-classes
         * \param prop_name the root property. If no prefix is used the default_uri_ prefix is assigned
         * \return a list of all classes in the subtree (not ordered), with extended URI
         */
        std::list<std::string>  getSubClassesL(std::string class_name);
        /*!
         * \brief Return a set of all sub-properties
         * \param prop_name the root property. If no prefix is used the default_uri_ prefix is assigned
         * \return a set of all properties in the subtree (not ordered)), with extended URI
         */
        std::set<std::string> getSubPropertiesS(std::string prop_name);
        /*!
         * \brief Return a list of all sub-properties
         * \param prop_name the root property. If no prefix is used the default_uri_ prefix is assigned
         * \return a list of all properties in the subtree (not ordered)), with extended URI
         */
        std::list<std::string> getSubPropertiesL(std::string prop_name);
        //! \brief Return all individuals of the branch
        std::list<std::string>  getBranchIndividuals(owl::Node node);
        //! \brief Return all sources of the partial statement
        librdf_iterator * getSources(owl::Node predicate, owl::Node object);
        //! \brief Return one target of the partial statement
        librdf_node * getTarget(owl::Node subject, owl::Node predicate);
        //! \brief Return all active statements related to an object
        ResultList getContextStatementsOld(owl::Node context);
        /*!
         * \brief Return all statements related to an object
         * \param context the object of interest
         * \param filter 0: all statements, 1: active statements, 2: passive statements
         * \return a vector of statements
         */
        std::vector<Statement> getContextStatements(owl::Node context, int filter=0);

        std::string getUpperClass(owl::Node node);

        std::string getIndividualType(owl::Node node);

        //! \brief Returns a class-constraint map
        ResultList getClassToConstraintsMap();
        //! \brief Returns all defined constraints in ontology
        ResultList getConstraintsDescription();

        /*!
         * \brief The input can't be left partially undefined leaving the Node.data = "". At least 1 node should be specified
         * \param sub, the statement subject
         * \param pred, the statement predicate
         * \param obj, the statement object
         * \return a stream of all statements partially matching with the input
         */
        librdf_stream * getStatements(owl::Node sub, owl::Node pred, owl::Node obj);

        /*!
         * \brief
         * \return True if the statement is found in the model, false otherwise
         */
        bool containStatement(owl::Node sub, owl::Node pred, owl::Node obj);

        //---------- Queries -----------
        /*!
         * \brief
         * \param child the child URI (if no prefix the default_uri_ prefix is assigned)
         * \param parent the parent URI (if no prefix the default_uri_ prefix is assigned)
         * \return true if child is a subproperty of parent, false otherwise
         */
        bool isSubProperty(std::string child, std::string parent);
        /*!
         * \brief isType check if a node is of a certain type
         * \param subject the subject node
         * \param type the type
         * \return true if the subject is of the type, false otherwise
         */
        bool isType(owl::Node subject, owl::Node type);


        //! \brief SPARQL query to string
        std::string queryAsString(const char * query_string, bool cut_prefix)
        {
            return results2string(query(query_string), cut_prefix);
        }
        //! \brief Convert librdf results to list of nodes
        ResultList results2list(librdf_query_results * result, int num_of_var);
        //! \brief Convert librdf results to a string
        std::string results2string(librdf_query_results * result, bool cut_prefix);

        //---------- Prefix -----------
        //! \brief Add a prefix (a short uri alias) to the list
        void addPrefix(std::pair<std::string, std::string> prefix);
        //! \brief Check if a prefix has been declared
        bool hasPrefix(std::string prefix);
        /*!
         * \brief converts a prefix to its shorter version
         * \param long prefix an extended prefix
         * \return short prefix a short prefix
         */
        std::string getShortPrefix(std::string prefix);
        /*!
         * \brief converts a short name to a complete URI
         * \param resource_name a resource name with a short prefix. If no prefix the default_uri_ prefix is assigned
         * \return an extended URI string
         */
        std::string getUri(std::string resource_name);

        //---------- Members access and info -----------
        librdf_world* getWorldPtr() {return world_;}
        inline PrefixMap & getPrefixMap() { return prefix_map_inverse_;}
        inline std::string getDefaultUri(){ return default_uri_;}
        inline bool isInitialized(){return initialized_;}
        inline librdf_model* getModel(){return model_;}
        //! \brief Get the skiros workspace folder path
        std::string getWorkspacePath();
    private:
        //! \brief Convertion from owl::Node to librdf_node
        librdf_node * getRdfNode(owl::Node node);
        inline librdf_stream * getStatements(librdf_statement * statement){return librdf_model_find_statements(model_, statement);}
        bool isNodeInModel(librdf_node * node);
        std::list<std::string>  getSub(librdf_node * node, librdf_node * subOf);
        inline std::list<std::string>  getSubClasses(librdf_node * node)
        {
            std::list<std::string> to_ret = getSub(node, LIBRDF_S_subClassOf(world_));
            librdf_free_node(node);
            return to_ret;
        }
        inline std::list<std::string>  getSubProperties(librdf_node * node)
        {
            std::list<std::string> to_ret = getSub(node, LIBRDF_S_subPropertyOf(world_));
            librdf_free_node(node);
            return to_ret;
        }
        std::list<std::string>  getBranchIndividuals(librdf_node * node);
        //! Set/Unset a statement
        void setStatement(librdf_node * subject, librdf_node * predicate, librdf_node * object, bool add);
        //! Format the printOntology() output using prefixes
        void format(FILE* in, FILE * out);
        //! SPARQL query
        librdf_query_results *  query(const char * query_string);

        //Owl local prefixes
        PrefixMap prefix_map_;
        PrefixMap prefix_map_inverse_;
        std::string default_uri_;

        //Redland variables
        librdf_world* world_;
        librdf_storage* storage_;
        librdf_model* model_;
        raptor_world* raptor_w_ptr_;

        //Other variables
        bool initialized_;
        std::string wrkspc_path_;
    };
}
}


#endif // OWL_WORLD_ONTOLOGY_H
