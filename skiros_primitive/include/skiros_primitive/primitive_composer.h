#ifndef PRIMITIVE_COMPOSER_H
#define PRIMITIVE_COMPOSER_H

#include <ros/ros.h>
#include <skiros_primitive/definitions.h>
#include <skiros_msgs/PrimitiveEvent.h>
#include <skiros_msgs/PrimSchedulerSet.h>
#include <skiros_msgs/PrimSchedulerStartStop.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/condition_variable.hpp>
#include <skiros_world_model/module_core.h>

namespace skiros_primitive
{
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Primitive is a mirror class of real primitive nodelets, and is used
 by skills developers to manipulate connections and behaviours */

class Primitive
{
    //TODO: in the future...
};

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b  */

class PrimitiveComposer
{
public:
    /** \brief Initialize the PrimitiveComposer associated to a precise nodelet manager */
    PrimitiveComposer(std::string primitives_namespace, ros::NodeHandle & nh);

    ~PrimitiveComposer();
    /** \brief Unload a primitive */
    bool unloadPrimitive (const std::string name);

    /** \brief Load a primitive */
    bool loadPrimitive (std::string name,std::string type,std::vector<std::string> args, ros::M_string remappings);
    /*!
     * \brief setSchedulingRhythm Must be called before defining any primitive scheduling. Set the cycle time and the number of time slots in the scheduler
     * \param cycle_time loop period
     * \param time_division time slots contained in the loop period
     */
    void setSchedulingRhythm(double cycle_time, int time_division);
    /*!
     * \brief addSchedule add a new scheduling
     * \param primitive_name primitive to trigger
     * \param time_slot triggering time
     * \param set true=set false=unset. Default: true
     */
    void addSchedule(std::string primitive_name, int time_slot, bool set=true);

    //TODO: calling init should load all primitives, set scheduler, then wait until all nodelets are in idle state (wait tot time, then return error with not init nodelets)
    /** \brief init must be called before start scheduling or triggering. It initialize the scheduler and check that all primitives are ready to run */
    void init();

    /*!
     * \brief triggerPrimitive trigger a primitive. Note: this triggering method has a delay of 1 second to assure the connection with the primitive has been enstablished
     * \param primitive_name name of the primitive to trigger
     */
    void triggerPrimitive(std::string primitive_name);

    void configPrimitive(std::string primitive_name, skiros_common::ParamMap config);

    /** \brief Start the scheduler */
    void startScheduling();

    /** \brief Stop the scheduler */
    void stopScheduling();

    /*!
     * \brief waitEvent wait for an event
     * \param event_type wait for a specific event type. Leave blank to wait for any kind of event
     * \return a string in format "primitive_name:event_type"
     */
    std::string waitEvent(std::string event_type="", ros::Duration timeout=ros::Duration(1.0));

    /*!
     * \brief getEvent get the first event
     * \param pop remove the event from the list
     * \param wait if true wait until an event arrives
     * \return the first event in list in format "primitive_name:event_type", or an empty string
     */
    std::string getEvent(bool pop=true, bool wait=false);

    /** \brief Terminate all loaded primitives */
    void clearPrimitives();

private:
    void eventMonitorCB(const skiros_msgs::PrimitiveEvent& msg);
    void stateMonitorCB(const skiros_msgs::PrimitiveEvent& msg);
    void waitAllInit();
    //--- Event
    std::list<std::string> event_;
    boost::mutex event_mux_;
    boost::condition_variable event_cond_;
    //--- Scheduling
    std::vector<std::vector<std::string> > schedule_list_;
    double cycle_time_;
    int time_division_;

    //--- Primitive monitor
    typedef std::map<std::string, skiros::state::StateType> M_primitives;
    typedef std::pair<std::string, skiros::state::StateType> P_primitives;
    M_primitives primitives_;
    boost::mutex primitives_mux_;
    boost::condition_variable primitives_cond_;

    //--- Ros publisher and subscribers
    std::string primitives_namespace_;
    std::string nodelet_manager_;
    ros::NodeHandle & nh_;
    ros::Publisher scheduler_start_stop_;
    ros::Publisher scheduler_set_;
    ros::Subscriber monitor_state_;
    ros::Subscriber monitor_event_;

};
}

#endif // PRIMITIVE_COMPOSER_H
