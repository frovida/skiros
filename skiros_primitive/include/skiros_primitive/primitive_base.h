#ifndef BASEPRIMITIVE_H
#define BASEPRIMITIVE_H
#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <skiros_msgs/PrimitiveEvent.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <std_msgs/Bool.h>
#include <skiros_world_model/module_core.h>
#include <skiros_primitive/definitions.h>
//--- Header to be moved to cpp -----
#include <skiros_msgs/ParamMapSerialized.h>

namespace skiros_primitive
{

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b BasePrimitive represents the base privimitive class.
 * All primitive nodelets should inherit from this class. */

class BasePrimitive : public nodelet::Nodelet, public skiros::ModuleCore
{
public:
    virtual ~BasePrimitive(){}

    /** \brief Primitive initialization routine. */
    virtual void onInit();

protected:
    BasePrimitive() : ModuleCore() {}

    //-----------------------------------------------------------------------------
    //-------------- Protected functions for use by the subclass.  ----------------
    //-----------------------------------------------------------------------------
    //--------- State and events --------------
    void setStateAndPublish(skiros::state::StateType new_state, bool publish=true);

    inline skiros::state::StateType getState(){return state_;}

    void publishEvent(std::string event);
    //--------- I\O --------------
    template <class T, class M>
    void addInput(const std::string& topic, void(T::*fp)(M), T* obj, int queue_size=1)
    {
        in_[topic] = getMTPrivateNodeHandle().subscribe(topic, queue_size, fp, obj);
    }
    inline ros::Subscriber & input(std::string topic){ return in_[topic]; }

    template <class T>
    void addOutput(std::string topic, int queue_size=1)
    {
        out_[topic] = getMTNodeHandle().advertise<T>(topic, queue_size);
    }
    inline ros::Publisher & output(std::string topic){ return out_[topic]; }

    void setConfigCB(skiros_msgs::ParamMapSerialized msg);
private:
    //-------------------------------------------------
    //-------------- Private data fields --------------
    //-------------------------------------------------
    //! \brief Timer activated function
    void statePubCB(const ros::TimerEvent& e)
    {
        publishState();
    }

    //! \brief Called automatically when changing state
    void publishState();

    /** \brief Primitive triggered routine. */
    void triggerCB(std_msgs::Bool msg);

    //--------- Configuration port --------------
    ros::Subscriber config_sub_;
    ros::Publisher config_pub_;

    //--------- Scheduling ports --------------
    ros::Subscriber trigger_sub_;

    //--------- I\O ports --------------
    //TODO: maybe better ptr?
    typedef std::map<std::string, ros::Publisher> M_out;
    typedef std::pair<std::string, ros::Publisher> T_out;
    typedef std::map<std::string, ros::Subscriber> M_in;
    typedef std::pair<std::string, ros::Subscriber> T_in;
    M_out out_;
    M_in in_;
    //--------- State and events --------------
    ros::Publisher state_pub_;
    ros::Publisher event_pub_;
    ros::Timer publish_state_timer_;
    //--------- ROS config --------------
    int max_queue_size_;
};

};

#endif // BASEPRIMITIVE_H
