//---------- Mandatory primitive's include ----------
#include <pluginlib/class_list_macros.h>//Plugin export library
#include <string>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Bool.h>
#include <skiros_primitive/definitions.h>
#include <skiros_msgs/PrimitiveEvent.h>
#include <skiros_msgs/PrimSchedulerSet.h>
#include <skiros_msgs/PrimSchedulerStartStop.h>
#include "skiros_common/logger_sys.h"   //Skiros logging system
#include <boost/foreach.hpp>
#include <skiros_world_model/module_core.h>
//---------- Mandatory skill's include end ----------

using namespace skiros;

namespace skiros_primitive
{

/* BasePrimitive represents the base privimitive Nodelet class.
 * All primitives nodelets should inherit from this class. */
class PrimitiveScheduler : public nodelet::Nodelet
{
public:
    PrimitiveScheduler() : state_(state::uninitialized)
    {

    }

    ~PrimitiveScheduler()
    {
    }
    //TODO: try to define services outside the class. Use of topics is not reliable
    /** \brief Nodelet initialization routine. */
    void onInit()
    {
        cycle_period_ = ros::Duration(1.0);
        max_queue_size_ = 10;
        time_division_ = 1;
        current_time_slot_ = 0;
        trig_msg_.reset(new std_msgs::Bool());
        trig_msg_->data = true;
        private_nh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle ())); //This publish on the namespace of the nodelet
        public_nh_.reset(new ros::NodeHandle(getMTNodeHandle())); //This publish on the namespace of the nodelet manager
        scheduling_timer_ = private_nh_->createTimer(cycle_period_, &PrimitiveScheduler::schedulingCB, this, false, false);
        //------------------- Advertisers --------------------
        state_pub_ = public_nh_->advertise<skiros_msgs::PrimitiveEvent> (STATUS_TOPIC, max_queue_size_);
        //------------------- Subscribers --------------------
        //event_sub_ = public_nh_->subscribe("monitor/event", 10, &PrimitiveScheduler::eventCB, this);
        start_stop_ = public_nh_->subscribe(SCHED_START_TOPIC, 1, &PrimitiveScheduler::startStopCB, this);
        set_scheduling_ = public_nh_->subscribe(SCHED_SET_TOPIC, 1, &PrimitiveScheduler::setSchedulingCB, this);
        sleep(1); //Wait for the connection with the monitor to be setted up
    }


private:
    /*void eventCB(const skiros_msgs::PrimitiveEvent& msg)
    {

    }*/

    void schedulingCB(const ros::TimerEvent& e)
    {
        std::vector<std::string> temp = time_slots_[current_time_slot_];
        BOOST_FOREACH(std::string primitive_name, temp)
        {
            topic_map_[primitive_name].publish(trig_msg_);
        }
        if(++current_time_slot_>=time_division_) current_time_slot_ = 0;
    }

    void startStopCB(const skiros_msgs::PrimSchedulerStartStop& msg)
    {
        if(msg.start_stop==msg.START)
        {
            if(state_==state::idle) scheduling_timer_.start();
            else FERROR("Scheduler can't start, state is: " << state::StateStr[state_]);
        }
        else
        {
            current_time_slot_ = 0;
            scheduling_timer_.stop();
        }
    }

    void setSchedulingCB(const skiros_msgs::PrimSchedulerSet& msg)
    {
        cycle_period_.fromSec((double)msg.cycle_time_sec);
        time_division_ = msg.time_division;
        ros::Duration temp;
        temp.fromSec((double)cycle_period_.toSec()/(double)time_division_);
        FINFO("Time slot period is: " << temp);
        scheduling_timer_ = private_nh_->createTimer(temp, &PrimitiveScheduler::schedulingCB, this, false, false);
        time_slots_.resize(time_division_);

        if(msg.primitive_list.size()!=msg.scheduling_time_list.size())
            throw std::runtime_error("Primitive list and scheduling time list have not same lenght.");

        for(int i=0;i<msg.primitive_list.size();i++)
        {
            if(msg.scheduling_time_list[i]>=0 && msg.scheduling_time_list[i]<time_division_)
            {
                    time_slots_.at(msg.scheduling_time_list[i]).push_back(msg.primitive_list[i]);
                    topic_map_[msg.primitive_list[i]] = public_nh_->advertise<std_msgs::Bool>(msg.primitive_list[i]+"/"+TRIGGER_TOPIC, 1);
            }
            else
            {
                FERROR("Scheduling time (" << msg.scheduling_time_list[i] << ") for primitive '"
                       << msg.primitive_list[i] << "' is outside boundaries. Max: " << time_division_);
                return;
            }
        }
        FINFO("Scheduling has been set.");
        setState(state::idle);
    }

    void setState(state::StateType new_state, bool publish=true)
    {
        state_ = new_state;
        if(publish) publishState();
    }

    //Called automatically when changing state
    void publishState()
    {
        skiros_msgs::PrimitiveEventPtr ptr(new skiros_msgs::PrimitiveEvent);
        ptr->header.stamp = ros::Time::now();
        ptr->primitive_name = this->getName();
        ptr->status = state_;
        state_pub_.publish(ptr);
    }

    std_msgs::BoolPtr trig_msg_;
    ros::Duration cycle_period_;
    ros::Timer scheduling_timer_;
    int time_division_;
    int current_time_slot_;
    std::vector<std::vector<std::string> > time_slots_;
    typedef std::map<std::string, ros::Publisher> M_out;
    typedef std::pair<std::string, ros::Publisher> T_out;
    M_out topic_map_;
    ros::Subscriber start_stop_;
    ros::Subscriber set_scheduling_;
    //--------- State and events --------------
    state::StateType state_;
    ros::Publisher state_pub_;
    ros::Publisher event_pub_;
    ros::Subscriber event_sub_;
    //--------- ROS node handlers and config --------------
    boost::shared_ptr<ros::NodeHandle> private_nh_;
    boost::shared_ptr<ros::NodeHandle> public_nh_;
    int max_queue_size_;
}; // class

} // namespace

//Export
PLUGINLIB_EXPORT_CLASS(skiros_primitive::PrimitiveScheduler, nodelet::Nodelet)
