#include <skiros_primitive/primitive_composer.h>
#include <skiros_primitive/primitive_base.h>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "nodelet/loader.h"
#include "nodelet/NodeletList.h"
#include "nodelet/NodeletLoad.h"
#include "nodelet/NodeletUnload.h"
#include <skiros_common/logger_sys.h>
#include <skiros_msgs/ParamMapSerialized.h>
#include <skiros_common/utility.h>

using namespace skiros;

namespace skiros_primitive
{
    const char SCHEDULER[] = "scheduler";
    PrimitiveComposer::PrimitiveComposer(std::string primitives_namespace, ros::NodeHandle & nh):
    nh_(nh), primitives_namespace_(primitives_namespace)
    {
        nodelet_manager_ = primitives_namespace + "/manager";

        //Search nodelets_namespace/manager in the rosnodes list
        ros::V_string v;
        ros::master::getNodes(v);
        bool found = false;
        BOOST_FOREACH(std::string node, v)
        {
            if(node.find(nodelet_manager_) != std::string::npos)
            {
                found = true;
                break;
            }
        }
        if(!found)
        {
            //TODO: Here I create a new nodelet manager
            //BUT: I need the resource manager to start a new manager
            //SO NOW: throw error
            throw std::runtime_error("[PrimitiveComposer] The primitive nodeled manager seems down. ");
        }

        //Subscribe to monitor topics (primitives_namespace + "/monitor" +  "/state"/ "/event")
        monitor_state_   = nh_.subscribe(primitives_namespace+"/"+STATUS_TOPIC, 10, &PrimitiveComposer::stateMonitorCB, this);
        monitor_event_   = nh_.subscribe(primitives_namespace+"/"+EVENT_TOPIC, 10, &PrimitiveComposer::eventMonitorCB, this);
        scheduler_start_stop_ = nh_.advertise<skiros_msgs::PrimSchedulerStartStop>(primitives_namespace+"/"+SCHED_START_TOPIC, 1);
        scheduler_set_   = nh_.advertise<skiros_msgs::PrimSchedulerSet>(primitives_namespace+"/"+SCHED_SET_TOPIC, 1);
        loadPrimitive(SCHEDULER, "skiros_primitive::PrimitiveScheduler", std::vector<std::string>(), ros::M_string());
    }

    PrimitiveComposer::~PrimitiveComposer()
    {
        clearPrimitives();
        unloadPrimitive(SCHEDULER);
    }

    bool
      PrimitiveComposer::unloadPrimitive (const std::string name)
    {
      ROS_INFO_STREAM ("Unloading nodelet " << name << " from manager " << nodelet_manager_);

      std::string service_name = nodelet_manager_ + "/unload_nodelet";
      // Check if the service exists and is available
      if (!ros::service::exists (service_name, true))
      {
        // Probably the manager has shut down already, which is fine
        ROS_WARN("Couldn't find service %s, perhaps the manager is already shut down",
                 service_name.c_str());
        return (false);
      }

      ros::ServiceClient client = nh_.serviceClient<nodelet::NodeletUnload> (service_name);
      if(!client.exists()) throw std::runtime_error("[unloadPrimitive] The primitive nodeled manager seems down. ");

      // Call the service
      nodelet::NodeletUnload srv;
      srv.request.name = name;
      if (!client.call (srv))
      {
        // Maybe service shut down in the meantime, which isn't an error
        if (ros::service::exists(service_name, false))
          ROS_ERROR("Service call failed!");
        return false;
      }
      //Remove the primitive from internal list
      primitives_.erase(name);
      return true;
    }

    bool
      PrimitiveComposer::loadPrimitive (std::string name,
                     std::string type,
                     std::vector<std::string> args,
                     ros::M_string remappings)
    {
      std::vector<std::string> sources (remappings.size ()), targets (remappings.size ());
      ROS_INFO_STREAM ("Loading nodelet " << name << " of type " << type << " to manager " << nodelet_manager_ << " with the following remappings:");
      int i = 0;
      for (ros::M_string::iterator it = remappings.begin(); it != remappings.end (); ++it, ++i)
      {
        sources[i] = (*it).first;
        targets[i] = (*it).second;
        ROS_INFO_STREAM (sources[i] << " -> " << targets[i]);
      }

      std::string service_name = nodelet_manager_ + "/load_nodelet";

      // Wait until the service is advertised
      ROS_DEBUG ("Checking for service %s to be available...", service_name.c_str ());
      ros::ServiceClient client = nh_.serviceClient<nodelet::NodeletLoad> (service_name);
      if(!client.exists()) throw std::runtime_error("[loadPrimitive] The primitive nodeled manager seems down. ");

      //TODO: check that the required nodelet is actually a "skill_primitive" type

      // Call the service
      nodelet::NodeletLoad srv;
      srv.request.name = name;
      srv.request.type = type;
      srv.request.remap_source_args = sources;
      srv.request.remap_target_args = targets;
      srv.request.my_argv = args;
      srv.request.bond_id = ""; // No bond (means I don't have a feedback if a nodelet shutdown)
      if (!client.call (srv))
      {
        throw std::runtime_error("[loadPrimitive]Service call failed.");
      }
      //Store the primitive state
      primitives_.insert(P_primitives(name, state::uninitialized));
      return true;
    }

    void PrimitiveComposer::setSchedulingRhythm(double cycle_time, int time_division)
    {
        time_division_ = time_division;
        cycle_time_ = cycle_time;
        schedule_list_.clear();
        schedule_list_.resize(time_division);
    }

    void PrimitiveComposer::addSchedule(std::string primitive_name, int time_slot, bool set)
    {
        if(time_division_>0)
        {
            if(time_slot>=time_division_ || time_slot<0)
            {
                FERROR("[addSchedule] Time slot " << time_slot << " is out of boundary: " << time_division_);
                return;
            }
            try
            {
                primitives_.at(primitive_name);
                schedule_list_[time_slot].push_back(primitive_name);
            }
            catch(...)
            {
                FERROR("[addSchedule] Primitive " << primitive_name << " cannot be scheduled because hasn't been loaded.");
            }
        }
        else FERROR("[addSchedule] Scheduling period is not initialized. Did you call setSchedulingPeriod?");
    }

    //TODO: calling init should load all primitives, set scheduler, then wait until all nodelets are in idle state (wait tot time, then return error with not init nodelets)
    void PrimitiveComposer::init()
    {
        skiros_msgs::PrimSchedulerSet msg;
        msg.cycle_time_sec = cycle_time_;
        msg.time_division = time_division_;
        for(int i=0;i<schedule_list_.size();i++)
        {
            BOOST_FOREACH(std::string p, schedule_list_[i])
            {
                msg.primitive_list.push_back(p);
                msg.scheduling_time_list.push_back(i);
            }
        }
        scheduler_set_.publish(msg);
        FINFO("Waiting primitives initialization..");
        waitAllInit();
        FINFO("All primitives initialized.");
    }

    //Note: this triggering style has a delay of 1 second to assure the connection with the primitive has been enstablished
    void PrimitiveComposer::triggerPrimitive(std::string primitive_name)
    {
        ros::Publisher temp_pub = nh_.advertise<std_msgs::Bool>(primitive_name+"/"+TRIGGER_TOPIC, 1);
        std_msgs::Bool msg;
        ros::Duration t(0.1);
        while(temp_pub.getNumSubscribers()<=0)
            t.sleep();
        temp_pub.publish(msg);
    }

    void PrimitiveComposer::configPrimitive(std::string primitive_name, skiros_common::ParamMap config)
    {
        ros::Publisher temp_pub = nh_.advertise<skiros_msgs::ParamMapSerialized>(primitives_namespace_+"/"+primitive_name+"/"+SET_CONFIG_TOPIC, 1);
        skiros_msgs::ParamMapSerialized msg = skiros_common::utility::serializeParamMap(config);
        ros::Time start_time = ros::Time::now();
        ros::Duration t(0.1);
        //Wait for connection
        while(temp_pub.getNumSubscribers()<=0)
            t.sleep();
        temp_pub.publish(msg);
        std::string good_event = primitive_name+":"+CONFIG_EVENT;
        std::string bad_event = primitive_name+":"+BAD_CONFIG_EVENT;
        int i = 0;
        while(getEvent()!=good_event)
        {
            //if(getEvent()==bad_event) throw std::invalid_argument("[PrimitiveComposer::configPrimitive] The primitive configuration wasn't sufficient");
            t.sleep();
            i++;
            //Every 15 iterations i publish again the msg
            if(i>15)
            {
                i=0;
                temp_pub.publish(msg);
            }
        }
        FDEBUG("Configured in: " << (ros::Time::now() - start_time).toSec() << " second(s).");
    }

    void PrimitiveComposer::waitAllInit()
    {
        bool ok = false;
        while(!ok)
        {
            boost::mutex::scoped_lock lock(primitives_mux_);
            ok = true;
            BOOST_FOREACH(M_primitives::value_type &p ,primitives_)
            {
                if(p.second!=state::idle) {FINFO("Primitive " << p.first << " is in state " << state::StateStr[p.second]);ok = false;break;}
            }
            lock.unlock();
            sleep(1);
        }
    }

    void PrimitiveComposer::startScheduling()
    {
        skiros_msgs::PrimSchedulerStartStop msg;
        msg.start_stop = skiros_msgs::PrimSchedulerStartStop::START;
        scheduler_start_stop_.publish(msg);
    }

    void PrimitiveComposer::stopScheduling()
    {
        skiros_msgs::PrimSchedulerStartStop msg;
        msg.start_stop = skiros_msgs::PrimSchedulerStartStop::STOP;
        scheduler_start_stop_.publish(msg);
    }

    void PrimitiveComposer::clearPrimitives()
    {
        //Here I should kill all loaded primitives except the scheduler
        BOOST_FOREACH(M_primitives::value_type &p ,primitives_)
        {
            if(p.first!=SCHEDULER) unloadPrimitive(p.first);
        }
    }

    //TODO: to restructure completely, not to use now
    std::string PrimitiveComposer::waitEvent(std::string event_type, ros::Duration timeout)
    {
        boost::mutex::scoped_lock lock(event_mux_);
        while(event_.size()<=0) event_cond_.wait(lock);
        std::string to_ret = event_.front();
        event_.pop_front();
        return to_ret;
    }

    std::string PrimitiveComposer::getEvent(bool pop, bool wait)
    {
        boost::mutex::scoped_lock lock(event_mux_);
        while(event_.size()<=0)
        {
            if(wait) event_cond_.wait(lock);
            else return std::string();
        }
        std::string to_ret = event_.front();
        if(pop) event_.pop_front();
        return to_ret;
    }

    void PrimitiveComposer::eventMonitorCB(const skiros_msgs::PrimitiveEvent& msg)
    {
        boost::mutex::scoped_lock lock(event_mux_);
        std::string temp(msg.primitive_name);
        temp = temp + ":" + msg.event_type;
        if(event_.size()>20)event_.pop_front();
        event_.push_back(temp);
        event_cond_.notify_all();
    }

    void PrimitiveComposer::stateMonitorCB(const skiros_msgs::PrimitiveEvent& msg)
    {
        boost::mutex::scoped_lock lock(primitives_mux_);
        //ROS_INFO_STREAM("[" << msg.header.seq << "]" << "[" << msg.header.stamp << "]" << msg.primitive_name << ": " <<  state::StateStr[msg.state]);
        primitives_[msg.primitive_name] = (state::StateType) msg.status;
        primitives_cond_.notify_all();
    }
}
