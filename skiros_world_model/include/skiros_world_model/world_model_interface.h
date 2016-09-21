#ifndef OWL_WORLD_MODEL_INTERFACE_H
#define OWL_WORLD_MODEL_INTERFACE_H

#include <ros/ros.h>
#include "skiros_world_model/world_element.h"
#include <tf/transform_listener.h>
#include "skiros_msgs/WmMonitor.h"
#include <skiros_config/declared_uri.h>
#include "skiros_world_model/ontology_interface.h"

namespace skiros_wm
{

typedef std::vector<std::pair<int, double> > IdLkhoodListType;
typedef std::vector<std::pair<std::string, double> > ClassLkhoodListType;

/*!
 * \brief Provide methods to query and edit the ontology via ROS
 */
class OntologyInterface : public BaseOntologyInterface
{
protected:
    void connectionFailed(std::string service_name);
    void addStdPrefix(std::string &uri);
    bool connected_;
    ros::NodeHandle nh_;
    ros::ServiceClient query_ontology_;
    ros::ServiceClient modify_ontology_;
public:
    OntologyInterface(ros::NodeHandle nh);
    //----------- Utils ------------------
    /*!
     * \brief Check that the connection with the World Model server is active
     * \return True if the connection is active
     */
    bool isConnected();
    /*!
     * \brief Wait that the connection with the World Model server is active
     * \param timeout Wait for the corresponded amount of time
     * \return True if the connection is active, false otherwise
     */
    bool waitConnection(ros::Duration timeout);
    //----------- Basic query ------------------
    /*!
     * \brief Direct query to ontology
     * \param query_string a query string in SPARQL format (https://en.wikipedia.org/wiki/SPARQL)
     * \param cut_prefix if true remove the prefix from the results URIs
     * \return a string with the results separated by an endline
     */
    std::string queryOntology(std::string query_string, bool cut_prefix=true);
    //----------- Modify methods ------------------
    /*!
     * \brief store permanently an element as an individual in the ontology
     * \param e the element to store, where: the label define the individual URI, the type define the class, the ID is ignored, all other properties are stored
     * \return positive value on success, negative on failure
     */
    int addIndividual(skiros_wm::Element e);
    /*!
     * \brief remove permanently an individual from the ontology
     * \param e the element to remove
     * \return positive value on success, negative on failure
     */
    int removeIndividual(skiros_wm::Element e);
    //----------- Advanced methods ------------------
    /*!
     * \brief
     * \param parent_class the URI of type
     * \return a set with the subtypes of the parent class
     */
    std::set<std::string> getSubClasses(std::string parent_class);
    /*!
     * \brief
     * \param sub_class the URI of type
     * \return a set with the parent types of the class
     */
    std::set<std::string> getSuperClasses(std::string sub_class);
    /*!
     * \brief
     * \param individual the URI of an individual
     * \param reverse if true the individual is used as subject (instead of object) of the statement in the query
     * \return a set with equivalent individuals URIs
     */
    std::set<std::string> getEquivalents(std::string individual, bool reverse=false);
    /*!
     * \brief
     * \param uri an URI of a resource
     * \return the resource type
     */
    std::string getType(std::string uri){ return queryOntology("SELECT ?x where {"+addPrefix(uri)+" rdf:type ?x}"); }
    /*!
     * \brief convert the result of a query into a std::set. Valid only with single value per line
     * \param query_result result of a query
     * \return a set containing the results
     */
    std::set<std::string> makeSet(std::string query_result);
};

/*! \brief Extend OntologyInterface with methods to interact with the world model scene
 *
 *
 */
class WorldModelInterface : public OntologyInterface
{
public:
    /*!
     * \brief
     * \param nh a ros nodehandle
     * \param monitor_changes If true, the interface will subscribe to the world model monitor (see function hasChanged)
     */
    WorldModelInterface(ros::NodeHandle nh, bool monitor_changes=false);

    //--------- Relation methods  -------------
    /*!
     * \brief set\unset a relation between two elements (almost unused)
     * \param subject_id
     * \param predicate
     * \param object_id
     * \param set, true to set the relation, false to unset the relation
     * \return
     */
    int setRelation(int subject_id, std::string predicate, int object_id, bool set = true);
    /*!
     * \brief query relations between elements
     * \param subject the id of an element existing in the world instance or -1
     * \param predicate the relation between subject and object. Leave "" to get all relations
     * \param object the id of an element existing in the world instance  or -1
     * \return the relations matching the input
     */
    RelationsVector queryRelation(int subject, std::string predicate, int object);
    //--------- Element methods  -------------
    /*!
     * \brief
     * \param id the number identifing the element in the database
     * \return  the element. If not found, an empty element is returned (with ID = -1 and type="Unknown")
     */
    skiros_wm::Element getElement(int id);
    /*!
     * \brief
     * \param uri element URI in format "Type-Id".
     * \return the element. If not found, an empty element is returned (with ID = -1 and type="Unknown")
     */
    skiros_wm::Element getElementFromUri(std::string uri);

    /*!
     * \brief return all elements in the db having the same type as the input.
     * \param e, element. The important field is the "Type".
     * \return a vector of elements
     */
	std::vector<skiros_wm::Element> resolveElement(skiros_wm::Element e);
    //! \brief Remove the element. Return true if succeed
    bool removeElement(int id);
    /*!
     * \brief add the element to the world instance and assign an ID
     * \param e the element to add
     * \param parent_id the id of the parent element
     * \param predicate the type of relation between the input element and the parent
     * \return the ID assigned to the element or -1
     */
    int addElement(skiros_wm::Element & e, int parent_id, std::string predicate)
    {
        RelationsVector relations;
        relations.push_back(RelationType(parent_id, predicate, -1));
        return addElement(e, relations);
    }
    int addElement(skiros_wm::Element & e, int parent_id, skiros_config::owl::relation::RelationType predicate)
    {
        return addElement(e, parent_id, skiros_config::owl::relation::Str[predicate]);
    }
    /*!
     * \brief add the element to the world instance and assign an ID
     * \param e the element to add
     * \param relations list of relations
     * \return the ID assigned to the element or -1
     */
    int addElement(skiros_wm::Element & e, RelationsVector relations);
    /*!
     * \brief update an element description. All properties not specified are NOT overwritten.
     * \param e element to update, with ID > 0
     * \param parent_id the id of the parent element
     * \param predicate the type of relation between the input element and the parent
     * \return the id of the element or -1
     */
    int updateElement(skiros_wm::Element e, int parent_id=-1, std::string predicate="");
    int updateElement(skiros_wm::Element & e, int parent_id, skiros_config::owl::relation::RelationType predicate)
    {
        return updateElement(e, parent_id, skiros_config::owl::relation::Str[predicate]);
    }
    /*!
     * \brief get an element intitialized with the properties defined in the ontology
     * \param individual the URI of the individual in the ontology (the prefix is optional)
     * \return an element intialized with the properties of the individual
     */
    skiros_wm::Element getDefaultElement(std::string individual);
    /*!
     * \brief Check if a string is a valid instance URI (so that can be used as input in the function getElementFromUri)
     * \param uri the string to be checked
     * \return True if the string is a valid URI, false otherwise
     */
    bool isElementUri(std::string uri);
    /*!
     * \brief Add the object and recursively all the individual related to the object with hasA relation
     * \param object
     * \param parent_id
     * \param relation
     * \return the object without the hasA property
     */
    int addBranch(Element &object, int parent_id, std::string relation);
    //--------- Identification methods  -------------
    /*!
     * \brief
     * \param e element to classify
     * \param threshold between 0 and 1. Filters the matches below the filter
     * \return a list of pairs in the format: type-likelihood
     */
    ClassLkhoodListType classify(skiros_wm::Element e, float threshold = 0);
    /*!
     * \brief
     * \param e element to identify
     * \param parent_id limit the comparison to the branch of the parent
     * \param threshold between 0 and 1. Filters the matches below the filter
     * \return a list of pairs in the format: id-likelihood
     */
    IdLkhoodListType identify(skiros_wm::Element e, int parent_id, float threshold = 0);

    //--------- Advanced methods  -------------
    /*!
     * \brief
     * \param V1) parent element V2) parent id
     * \param relation
     * \return parent element or element with id -1 if not found
     */
    skiros_wm::Element getParentElement(skiros_wm::Element e, std::string relation=""){return getParentElement(e.id(), relation);}
    skiros_wm::Element getParentElement(skiros_wm::Element e, skiros_config::owl::relation::RelationType relation)
    {return getParentElement(e.id(), skiros_config::owl::relation::Str[relation]);}
    skiros_wm::Element getParentElement(int id, std::string relation="");
    //TODO: getChildElements and getBRanchElements must be done with SPARQL queries. This means also support functions like getSubClasses, etc. TODO!!
    /*!
     * \brief
     * \param V1) parent element V2) parent id
     * \param relation filter on the relation
     * \param type filter on the child's type\label
     * \return vector of child elements filtered by "relation" and "type"
     */
    std::vector<skiros_wm::Element> getChildElements(skiros_wm::Element e, std::string relation="", std::string type="")
    {return getChildElements(e.id(), relation, type);}
    std::vector<skiros_wm::Element> getChildElements(skiros_wm::Element e,
                                                     skiros_config::owl::relation::RelationType relation)
    {return getChildElements(e.id(), skiros_config::owl::relation::Str[relation]);}
    std::vector<skiros_wm::Element> getChildElements(skiros_wm::Element e,
                                                     skiros_config::owl::relation::RelationType relation,
                                                     skiros_config::owl::concept::ConceptType type)
    {return getChildElements(e.id(), skiros_config::owl::relation::Str[relation], skiros_config::owl::concept::Str[type]);}
    std::vector<skiros_wm::Element> getChildElements(int id, std::string relation="", std::string type="");
    /*!
     * \brief This is the recursive version of getChildElements
     * \param V1) root element V2) root id
     * \param relation filter on the relation
     * \param type filter on the type\label
     * \return vector of matches
     */
    inline std::vector<skiros_wm::Element> getBranchElements(skiros_wm::Element e, std::string relation="", std::string type=""){return getBranchElements(e.id(), relation, type);}
    std::vector<skiros_wm::Element> getBranchElements(int id, std::string relation="", std::string type="");
    /*!
     * \brief remove an entire branch of the world model
     * \param id is the root node id
     */
    inline void removeBranch(skiros_wm::Element e){removeBranch(e.id());}
    void removeBranch(int id);

    /*!
     * \brief get a branch of the world model and stores it in a WorldGraph class
     * \param graph is the WorldGraph where the branch gets stored
     * \param root_id the id of the branch's root
     * \param relation filter on the relation
     * \param type filter on the type\label
     */
    void getBranch(WorldGraph & graph, int root_id, std::string relation="", std::string type="");

    /*!
     * \brief Push a WorldGraph into the world model.
     * \param graph, a reference the graph to be pushed in the world model. The flag "isGlobalElement" get set to true for all elements on success.
     * \param root_id, if specified, defines where to attach the branch, if not the connections are made automatically (if there is at least 1 element in common)
     */
    void pushBranch(WorldGraph & graph, int root_id=-1);

    //--------- Callbacks & utility  -------------
    /*!
     * \brief Check if the world model has been modified since the last call of this function.
     *
     * This function can be used only if the interface has been initialized with monitor_changes=true
     *
     * \return True if the world model has been modified, false otherwise
     */
    bool hasChanged();

    /*!
     * \brief lock the world model server for critical modifications
     */
    void lock();
    /*!
     * \brief unlock the world model server
     */
    void unlock();
protected:
    //--------- Private methods  -------------
    //Check changes
    void wmMonitorCB(const skiros_msgs::WmMonitor& msg);
    bool new_changes_;
    ros::Subscriber wm_monitor_sub_;
    bool wm_locked_=false;

    //--------- Private variables  -------------
    int robot_registered_id_;
    ros::ServiceClient query_model_;
	ros::ServiceClient set_relation_;
	ros::ServiceClient element_get_;
	ros::ServiceClient element_modify_;
	ros::ServiceClient element_identify_;
    ros::ServiceClient element_classify_;
    ros::ServiceClient lock_unlock_;
};

/*! \brief Extend WorldModelInterface with methods for skills
 *
 *  This interface has extra methods to retrieve information about the robot description and interact with the ROS TF system
 */
class WorldModelInterfaceS : public WorldModelInterface
{
public:
    WorldModelInterfaceS(ros::NodeHandle nh) : WorldModelInterface(nh){}
    //--------- Robot registration methods  -------------
    //! \brief Return the robot element
    skiros_wm::Element getRobot();
    //! \brief Return the robot location element (the parent of the robot)
    skiros_wm::Element getRobotLocation();
    //! \brief Return the list of robot's devices (the childs of the robot)
    inline std::vector<skiros_wm::Element> getRobotHardware()
    {
        if(robot_registered_id_>0) return getBranchElements(robot_registered_id_, "hasA");
        else return std::vector<skiros_wm::Element>();
    }
    /*!
     * \brief Register the robot on the world model (done by the skill manager at boot)
     * \param location_id, the location of the robot
     * \param robot, the robot element
     * \param devices, the list of devices
     * \return
     */
    int registerRobot(skiros_wm::Element location_id, skiros_wm::Element robot);
    //! \brief Erase from the world model all the elements created with the registerRobot function
    void unregisterRobot();
public:
    //------ Tf on SkiROS
    inline std::string getFirstParentFrameId(skiros_wm::Element e){return getFirstFrameId(getParentElement(e));}
    std::string getFirstFrameId(skiros_wm::Element e);
    //------ Tf -------
    //! \brief Updating the FrameID property with tf prefix. If the prefix is already present, is not added again.
    void addTfPrefix(Element &e);
    //! \brief Activate the TF listener
    void startTfListener();
    //! \brief Stop the TF listener
    void stopTfListener();
    /*!
     * \brief
     * \return A const reference to the tf::TransformListener
     */
    inline const tf::TransformListener & getTfListener(){return *tf_listener_;}
    //! \brief Same as tf::TransformListener::waitForTransform
    bool waitForTransform(std::string target_frame, std::string  source_frame, ros::Time t, ros::Duration d = ros::Duration(3.0));
    //! \brief Same as tf::TransformListener::lookupTransform
    void lookupTransform(std::string target_frame, std::string source_frame, ros::Time t, tf::StampedTransform & transform);
    //! \brief Wraps waitForTransform and lookupTransform in one call
    bool waitAndLookupTransform(std::string target_frame, std::string  source_frame, ros::Time t, tf::StampedTransform & transform, ros::Duration d = ros::Duration(3.0));
    //! \brief Wait a newly added transform for the first published transformation
    bool waitAndLookupFirstTransform(std::string target_frame, std::string  source_frame, tf::StampedTransform & transform, ros::Duration d = ros::Duration(4.0), ros::Duration interval= ros::Duration(1.0));
    //! \brief Same as tf::TransformListener::transformPose
    void transformPose(const std::string& target_frame, const tf::Stamped<tf::Pose>& stamped_in, tf::Stamped<tf::Pose>& stamped_out);
protected:
    //!< A tf listener. Allow all modules to interact with TF with a unique listener.
    boost::shared_ptr<tf::TransformListener> tf_listener_;
};

}
#endif //OWL_WORLD_MODEL_INTERFACE_H
