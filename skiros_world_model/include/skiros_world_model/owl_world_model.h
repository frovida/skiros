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

#ifndef OWL_WORLD_MODEL_H
#define OWL_WORLD_MODEL_H

#include <exception>      // std::exception
#include "skiros_world_model/world_element.h"
#include "skiros_msgs/WmTypeLikelihood.h"
#include "skiros_msgs/WmObjLikelihood.h"
#include "boost/thread.hpp"

namespace skiros_wm
{
namespace owl
{

//Forward declarations
class Ontology;

typedef std::map<int, skiros_wm::Element> DatabaseMap;
typedef std::pair<int, skiros_wm::Element> DatabasePair;
/*!
     * \brief A simple database used internally in the WorldModel class
     */
class Database
{
public:
    Database() : counter_(1){initialized_=false;}

    void add(skiros_wm::Element & e, bool keep_id=false)
    {
        if(!keep_id)
            e.id() = getId();
        map_.insert(DatabasePair(e.id(), e));
    }
    void addRoot(skiros_wm::Element & e)
    {
        if(!initialized_)
        {
            e.id() = 0;
            map_.insert(DatabasePair(e.id(), e));
            initialized_ = true;
        }
    }
    void remove(int id)
    {
        map_.erase(id);
        counter_ = 1;
    }

    void clear()
    {
        counter_ = 1;
        initialized_ = false;
        map_.clear();
    }

    inline bool isRegisteredId(int id)
    {
        return map_.find(id)!=map_.end();
    }

    void loadSpecial(skiros_wm::Element & e) //(used only when loading saved database)
    {
        initialized_ = true;
        map_.insert(DatabasePair(e.id(), e));
    }

    DatabaseMap & map(){return map_;}
    //ID handling
    int getId()
    {
        while(map_.find(counter_) != map_.end()) counter_++;
        return counter_;
    }
private:
    bool initialized_;
    int counter_;
    DatabaseMap map_;
};


typedef std::multimap<std::string, std::string> UrisMap;
typedef std::pair<std::string, std::string> UrisPair;
typedef std::multimap<std::string, std::pair<std::string, std::string> > LiteralsMap;
typedef std::pair<std::string, std::pair<std::string, std::string> > LiteralsPair;
    /*!
     * \brief A wrapper of an ontology individual
     */
    class ElementFactory
    {
    public:
        ElementFactory(Ontology & ont): ontology_(ont) {}
        ~ElementFactory(){}

        bool load(std::string type);

        bool isIndividual();

        std::string getClassType();

        std::string getLoadedType() {return loaded_type_;}
        LiteralsMap getLiteralsMap(){return literals_map_;}

    private:
        //std::list<std::string> parent_list;
        UrisMap attributes_map_;
        LiteralsMap literals_map_;
        std::string loaded_type_;
        Ontology & ontology_;
    };

    /* Possible constraints:
     * -onProperty -> (can be on relation or data)
     * -onClass
     * -cardinality(min,max,qualified)
     * -valueFrom (some, only) y */
    /*!
     * \brief A wrapper of an ontology constraint
     */
    class Constraint
    {
    public:
        Constraint(){}
        Constraint(Ontology * ont, Database * db): ontology_(ont), db_(db)
        {}

        void load(std::vector<std::pair<std::string,std::string> > statements_list )
        {
            for(int i=0;i<statements_list.size();i++)
            {
                s_list_.insert(statements_list[i]);
            }
        }

        inline bool hasSpec(std::string specification)
        {
            //specification = WorldModel::uri2lightString(specification);
            return s_list_.find(specification)!=s_list_.end();
        }

        inline std::string getSpec(std::string specification)
        {
            //specification = WorldModel::uri2lightString(specification);
            return s_list_[specification];
        }

        bool evaluate()
        {

        }

    private:
        std::map<std::string, std::string> s_list_;
        Ontology * ontology_;
        Database * db_;
    };

    typedef std::vector<skiros_msgs::WmTypeLikelihood> ClassifyVector;
    typedef std::vector<skiros_msgs::WmObjLikelihood> IdentifyVector;
    typedef std::list<skiros_msgs::WmObjLikelihood> IdentifyList;
    typedef std::map<std::string, skiros_wm::Element> DefaultIndMap;
    typedef std::pair<std::string, skiros_wm::Element> DefaultIndPair;
    //!< Maps a class to its constraints
    typedef std::map<std::string, std::vector<std::string> > ClassToConstraintMap;
    typedef std::pair<std::string, std::vector<std::string> > ClassToConstraintPair;
    //!< Maps a constraint name to an handling class
    typedef std::map<std::string, Constraint > ConstraintMap;
    typedef std::pair<std::string, Constraint > ConstraintPair;
    /*!
     * \brief The core of the world model node, provide methods to load and handle a world model instance.
     *
     * The world model is always chained to an Ontology. The ontology defines which elements and relations
     * is possible to find in the instance. The ontology also defines constraints and default individuals.
     * The world model class is made to be shared between multiple processes.
     *
     */
    class WorldModel
    {
    public:
        WorldModel(Ontology & ont);
        void reset();


        //------------- Identification methods ---------------
        IdentifyVector identify(skiros_wm::Element e, int root_id);
        ClassifyVector classify(skiros_wm::Element e, float threshold);

        //------------- Parsing methods ---------------
        bool parseElementModify(std::string action, Element &e, RelationsVector &relations);
        bool parseRelationModify(std::string action, Element subj, std::string predicate, Element obj);
        /*!
         * \brief Check the reasoners specified for the object and add the data
         * \param e a reference to the element which data will be updated
         */
        void checkReasoners(Element & e);

        //------------- Element's methods ---------------
        skiros_wm::Element getElement(int id);
        void removeElements(std::vector<int> ids);
        bool removeElement(int id);
        bool removeElement(skiros_wm::Element e);
        int addElement(Element & e, RelationsVector relations);
        std::vector<skiros_wm::Element> resolveElement(skiros_wm::Element e);
        bool updateElement(skiros_wm::Element e, RelationsVector v=RelationsVector());
        /*!
         * \brief
         * \param type, the name as defined in the ontology
         * \param set_default_values, when true add default properties in the return element
         * \return element istance with un-initialized id (-1)
         */
        skiros_wm::Element getDefaultElement(std::string type);

        std::list<ChildsPair> getChilds(skiros_wm::Element parent, std::string relation_filter="", std::string type_filter="");
        ExportedGraphType getBranch(skiros_wm::Element parent, std::string relation_filter, std::string type_filter="");

        //------------- Relation methods ---------------
        RelationsVector findStatements(int sub_id, std::string predicate, int obj_id);
        bool removeStatement(int sub_id, std::string predicate, int obj_id);
        bool addStatement(int sub_id, std::string predicate, int obj_id);

        bool isDefaultIndividual(std::string name);
        bool isDeclaredRelation(std::string name);
        bool isDeclaredClass(std::string name);

        //--------------- IO methods ---------------

        std::string getSceneTree(bool verbose = false, std::string relations="spatiallyRelated");
        bool loadScene(std::string filename);
        bool saveScene(std::string filename);

        //------------- Utils ---------------
        std::string element2uri(skiros_wm::Element e);
        std::string uri2lightString(std::string uri);
        std::string checkPrefix(std::string uri);
        inline std::string getSceneName() const {return scene_name_;}
        int uri2id(std::string uri);

        void loadMainOntology(std::string filename, std::string scene_name = "factory");

        void loadSubOntology(std::string filename);
        void importOntology(owl::Ontology & other_ontology);

        //---------- Members access and info -----------
        std::string getWorkspacePath() const;
        std::set<std::string> getDeclaredRelations();
        std::set<std::string> getDeclaredClasses();
        std::set<std::string> getDeclaredIndividuals();
        std::set<std::string> getDeclaredData();
        bool hasChanged()
        {
            boost::mutex::scoped_lock lock(change_mux_);
            if(new_changes_)
            {
                new_changes_ = !new_changes_;
                return true;
            }
            else
                return false;
        }
        boost::shared_ptr<boost::mutex::scoped_lock> startChange()
        {
            boost::shared_ptr<boost::mutex::scoped_lock> lock(new boost::mutex::scoped_lock(change_mux_));
            new_changes_ = true;
            return lock;
        }
    private:
        //------------- Private methods ---------------
        IdentifyList identifyRecursive(Element e, Element root);
        void addElemenToOntology(Element e, std::string class_type_uri);
        LiteralsMap getLiteralsMapLight(LiteralsMap in);
        std::string printTree(int rood_id, std::string relations, std::string indend = "", bool verbose = false);
        void setWorld(std::string scene_name);
        void updateStatements();
        skiros_wm::Element createDefaultClass(std::string owl_class);
        skiros_wm::Element createDefaultIndividual(std::string owl_individual);
        //! Create a new root for the element database
        void createScene(std::string name);
        //------------- Private variables ---------------
        boost::mutex change_mux_;
        bool new_changes_ = true;
        std::string scene_name_;
        std::string save_path_;
        DefaultIndMap default_individuals_;
        ClassToConstraintMap class_constraint_map_;
        ConstraintMap constraints_;
        Database db_;
        Ontology & ontology_;
    };
}
}


#endif // OWL_WORLD_MODEL_H
