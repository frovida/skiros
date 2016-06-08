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

#ifndef WORLD_ELEMENT_H
#define WORLD_ELEMENT_H

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include "skiros_common/param.h"
#include <skiros_config/declared_uri.h>
#include "skiros_world_model/ontology_interface.h"


namespace skiros_msgs
{
    ROS_DECLARE_MESSAGE(WoStatement);
}

namespace skiros_wm
{
    typedef std::multimap<std::string, std::pair<std::string, std::string> > RelationsMMap;
    typedef std::map<std::string, std::pair<std::string, std::string> > RelationsMap;
    typedef std::pair<std::string, std::pair<std::string, std::string> > RelationsPair;
    /*! \brief The primitive world model element, characterized by an ID, a class type, an individual label and a set of properties
	 *
     * An element is the semantic description for any kind of object in the world. The class type field allows to categorize it in a
     * taxonomy, the label (string) and the ID (int) are the unique identifiers for, respectevely, the ontology and the database.
     * The set of properties is a list of eterogeneous datatypes that contain any relevant information associated to the object.
     * The information gets complete when the object is connected with other objects by mean of semantic relations, building a semantic graph.
     *
	 */
    class Element
	{
	public:
		//Initialize the element like an undefined
        Element(): type_("Undefined"), id_(-1) {}
		//Initialize the element with a type
		Element(std::string type): type_(type), id_(-1){}
        //Initialize the element with a type
        Element(skiros_config::owl::concept::ConceptType type): type_(skiros_config::owl::concept::Str[type]), id_(-1){}

		~Element(){}

		//-------- Access --------------
        inline int const id() const { return id_;	}
		inline int & id() { return id_;	}
        inline std::string const type() const { return type_; }
		inline std::string & type() { return type_; }
        inline ros::Time lastUpdate() const { return last_update_;  }
        inline ros::Time & lastUpdate()  { return last_update_;  }
        inline std::string const label() const { return label_;  }
        inline std::string & label()  { return label_;  }
        inline skiros_common::ParamMap const properties() const { return properties_;  }
		inline skiros_common::ParamMap & properties()  { return properties_;  }
		inline skiros_common::Param properties(std::string key) const {  return this->properties(key); }
        inline skiros_common::Param properties(skiros_config::owl::data::DataType key) const {  return this->properties(skiros_config::owl::data::Str[key]); }
		skiros_common::Param & properties(std::string key);
        inline skiros_common::Param & properties(skiros_config::owl::data::DataType key) {  return this->properties(skiros_config::owl::data::Str[key]); }
        inline bool hasProperty(std::string key) const {return (this->properties_.find(key)!=this->properties_.end());}
        inline bool hasProperty(skiros_config::owl::data::DataType key) const {  return hasProperty(skiros_config::owl::data::Str[key]); }

		//-------- Methods --------------
        std::string printState(std::string indend = "", bool verbose = true) const;
        /*!
         * \brief get the element URI
         * \return the URI in format type-id
         */
        std::string toUrl() const;
        /*!
         * \brief convert the element into a set of statements
         * \param wo interface to ontology
         * \return a vector of statements msgs
         */
        std::vector<skiros_msgs::WoStatement> toMsgStatements(BaseOntologyInterface * wo, bool scene_element);
        RelationsMMap toMapStatements(BaseOntologyInterface * wo);

		bool addProperty(skiros_common::Param p);
        bool addProperty(skiros_common::Param p, skiros_common::any default_value);
        bool addPropertyString(skiros_config::owl::data::DataType key, std::string default_value) { return addPropertyString(skiros_config::owl::data::Str[key], default_value); }
        bool addPropertyString(std::string key, std::string default_value);
        template <typename T>
        bool addProperty(skiros_config::owl::data::DataType key, T default_value) { return addProperty(skiros_config::owl::data::Str[key], skiros_common::any(default_value)); }
        template <typename T>
        bool addProperty(std::string key, T default_value){ return addProperty(key, skiros_common::any(default_value)); }
        bool addProperty(std::string key, skiros_common::any default_value);

        void removeProperty(skiros_common::Param p){this->removeProperty(p.key());}
        void removeProperty(skiros_config::owl::data::DataType key){removeProperty(skiros_config::owl::data::Str[key]);}
        void removeProperty(std::string key);
        void importProperties(const Element& rhs);
        inline void reset(std::string type){this->clear(); this->type() = type;}
		void clear();
		//Note: e' = e is differend to e = e'!! The element on the right is the restrictive one.
		//This function check if the element on the left has all the properties of the right one.
		bool operator==(const Element& rhs) const;

        friend std::ostream &operator<<(std::ostream& o, const Element & e)
        {
            o << e.toUrl();
            return o;
        }

        //void operator+=(const Element& rhs);

        //-------- Discrete reasoner's methods --------------
        /*!
        * \brief Return semantic relations between two objects with associated literal value
        * \param reasoner_name the reasoner to use to infer relations
        * \param other an element to compare to
        * \return a map of semantic relations between this and the other element
        */
        RelationsMap getRelationsWithDetailWrt(skiros_wm::Element &other, std::string reasoner_name="");
        /*!
        * \brief Return semantic relations between two objects
        * \param reasoner_name the reasoner to use to infer relations
        * \param other an element to compare to
        * \return a set of semantic relations between this and the other element
        */
        std::set<std::string> getRelationsWrt(skiros_wm::Element &other, std::string reasoner_name="");
        /*!
        * \brief Convert user data to reasoner data and store it into the element
        * \param reasoner_name the reasoner to use
        * \param any the data to be stored
        * \param set_code a code defining which data want to store. For the available codes refer to the documentation of the specific reasoner
        * \return
        */
        bool storeData(boost::any any, std::string  set_code="", std::string reasoner_name="");
        template<class T>
        //! \brief Templated version of storeData
        bool storeData(T any, std::string set_code="", std::string reasoner_name=""){ return storeData(boost::any(any), set_code, reasoner_name); }
        template<class T>
        //! \brief Templated version of storeData
        bool storeData(T any, skiros_config::owl::data::DataType set_code, std::string reasoner_name=""){ return storeData(boost::any(any), skiros_config::owl::data::Str[set_code], reasoner_name); }
        /*!
        * \brief Convert user data to reasoner data and store it into given element
        * \param reasoner_name the reasoner to use
        * \param get_code a code defining which data want to get. For the available codes refer to the documentation of the specific reasoner
        * \return
        */
        boost::any getData(std::string  get_code="", std::string reasoner_name="");
        template<class T>
        //! \brief Templated version of getData
        T getData(std::string get_code="", std::string reasoner_name=""){ return boost::any_cast<T>(getData(get_code, reasoner_name)); }
        template<class T>
        //! \brief Templated version of getData
        T getData(skiros_config::owl::data::DataType get_code, std::string reasoner_name="") { return getData<T>(skiros_config::owl::data::Str[get_code], reasoner_name); }
        typedef std::multimap<std::string, std::pair<std::string, std::string> > ReasonerDataMap;
        /*!
        * \brief extractOwlData, remove from the element all params that are not convertible to OWL
        * \param reasoner_name the reasoner to use. All properties related to the reasoner are returned in OWL format are removed from the object
        * \return the extracted parameters in a OWL compatible multimap
        */
        ReasonerDataMap extractOwlData(std::string reasoner_name="");
        /*!
         * \brief Add default reasoner properties to the element
         * \param reasoner_name the name of the reasoner to associate
         * \return True on success, false if reasoner can't be loaded
         */
        bool associateReasoner(std::string reasoner_name);
        /*!
         * \brief Clear the object from all properties related to a reasoner
         * \param reasoner_name the name of the reasoner to be cleared
         */
        void removeReasoner(std::string reasoner_name);
        //! \brief Return the reasoners associated to the element
        std::set<std::string> getAssociatedReasoners();
	private:
        //--------------------- Symbolic properties ---------------
		//!< Unique Identifier of the element, given when inserted in the world model
		int id_;
        //!< Defines the element category
		std::string type_;
        //! A string identifier (optional)
        std::string label_;
		//!< The time stamp of the last update
        ros::Time last_update_;
		//--------------------- Properties ---------------
		//!< A list of properties related to the object (color, pose, size, etc..)
        skiros_common::ParamMap properties_;
        //--------------------- Reasoner ---------------
        std::string current_reasoner_;
	};

    /*!
     * \brief The RelationType class wraps a relation between two elements in the database
     */
    class RelationType
    {
    public:
        RelationType(){}
        RelationType(int subject_id, std::string predicate, int object_id)
        {
            reset(subject_id,predicate, object_id);
        }

        RelationType reset(int subject_id, std::string predicate, int object_id)
        {
            subject_id_ = subject_id;
            predicate_ = predicate;
            object_id_ = object_id;
            return *this;
        }

        inline int & subject_id(){return subject_id_;}
        inline std::string & predicate(){return predicate_;}
        inline int & object_id(){return object_id_;}
    private:
        int subject_id_;
        std::string predicate_;
        int object_id_;
    };

    typedef std::vector<RelationType> RelationsVector;
    typedef std::list<RelationType> RelationsList;

    typedef std::map<int, Element> WorldElementMap;
    typedef std::pair<int, Element> WorldElementPair;
    typedef std::multimap<int, boost::shared_ptr<RelationType> > WorldArcPtrMultimap;
    typedef std::map<int, boost::shared_ptr<RelationType> > WorldArcPtrMap;
    typedef std::pair<int, boost::shared_ptr<RelationType> > WorldArcPtrPair;
    typedef std::pair<std::vector<Element>, std::vector<RelationType> > ExportedGraphType;
    typedef std::pair<std::string, skiros_wm::Element> ChildsPair;
    //TODO: by now the relation predicates are not taken into consideration (would need the support from ontology)
    /*!
     * \brief A light class that wraps a graph of elements and relations.
     *
     * This class is used for the transport of world model subsets.
     * Doesn't provide any consistancy check with respect to the ontology, so must be handled carefully.
     * Support only a tree structure (every node has 1 parent and n children), no other relations can be defined
     * It can be merged into the shared world model, as long as there is at least 1 shared element or a relation referring to an element in the world model (to know where to attach the subtree)
     */
    class WorldGraph
    {
    public:
        WorldGraph() : id_counter_(1), root_id_(-1) {}
        ~WorldGraph() {}

        /*!
         * \brief isGlobalElement
         * \param id, element id
         * \return true if the element is present in the shared world model, false if it is local
         */
        bool isGlobalElement(int id) {return global_element_.find(id)!=global_element_.end();}
        void setGlobalElement(int id) {if(global_element_.find(id)==global_element_.end())global_element_.insert(id);}
        void unsetGlobalElement(int id) {if(global_element_.find(id)!=global_element_.end())global_element_.erase(id);}

        int getRootId(){return root_id_;}
        void setRootId(int id){root_id_=id;}

        Element getParentElement(int id)
        {
            if(id==0)
                return Element();
            else return nodes_.at(passive_rel_.at(id)->subject_id());
        }

        int getParentId(int id)
        {
            if(id==0)
                return -1;
            else return passive_rel_.at(id)->subject_id();
        }

        RelationType getParentRelation(int id)
        {
            if(id==0)
                return RelationType(-1,"",id);
            else return *passive_rel_.at(id);
        }

        Element & getElement(int id){return nodes_.at(id);}

        std::vector<ChildsPair> getChildElements(int id, std::string relation="");

        void updateElement(skiros_wm::Element & e, int parent_id=-1, std::string predicate="");

        int addElement(skiros_wm::Element & e, int parent_id, std::string predicate, bool is_global=false);

        void removeElement(int id);

        void clear();

        ExportedGraphType exportGraph();

        void importGraph(std::vector<Element> ev, std::vector<RelationType> rv, bool is_global);

        std::string print(bool verbose = false);

    private:

        std::string printRecursive(Element e, std::string indend="", bool verbose=false);

        void addElement(skiros_wm::Element & e, bool is_global=false);

        bool isRegisteredId(int id){return nodes_.find(id)!=nodes_.end();}

        //! ID handling
        int getId()
        {
            while(isRegisteredId(id_counter_)) id_counter_++;
            return id_counter_;
        }

        void addRelation(RelationType rel, bool passive=true);

        void removeRelation(RelationType rel);

        //Variable
        int root_id_;
        int id_counter_;
        WorldElementMap nodes_;
        WorldArcPtrMultimap active_rel_;
        WorldArcPtrMap passive_rel_;
        //!< The set of elements that are present also in the world model
        std::set<int> global_element_;
    };
}

namespace ros
{
namespace serialization
{
  template<>
  struct Serializer<skiros_wm::Element>
  {
    template<typename Stream>
    inline static void write(Stream& stream, const skiros_wm::Element& e)
    {
      stream.next(e.id());
      stream.next(e.type());
      stream.next(e.label());
      stream.next(e.lastUpdate());
      stream.next(e.properties());
    }

    template<typename Stream>
    inline static void read(Stream& stream, skiros_wm::Element& e)
    {
      stream.next(e.id());
      stream.next(e.type());
      stream.next(e.label());
      stream.next(e.lastUpdate());
      stream.next(e.properties());
    }

    inline static uint32_t serializedLength(const skiros_wm::Element& e)
    {
      uint32_t size = 0;
      size += serializationLength(e.id());
      size += serializationLength(e.type());
      size += serializationLength(e.label());
      size += serializationLength(e.lastUpdate());
      size += serializationLength(e.properties());
      return size;
    }
  };
}
}
#endif //WORLD_ELEMENT_H
