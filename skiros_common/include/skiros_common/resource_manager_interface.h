#ifndef RESOURCEMGRINT_HPP
#define RESOURCEMGRINT_HPP

#include "ros/ros.h"
#include <vector>
#include "skiros_msgs/RmDeviceHandle.h"
#include "skiros_common/logger_sys.h"

namespace skiros_common
{
    class ResourceManagerInterface
    {

    public:

        ResourceManagerInterface(ros::NodeHandle & nh, std::string rm_name) : nh_(nh), rm_name_(rm_name)
        {
            rm_name = rm_name + "_rm/device_handler";
            device_handler_ = nh_.serviceClient<skiros_msgs::RmDeviceHandle>(rm_name.c_str());
            connected_ = false;
        }

        ~ResourceManagerInterface();

        bool loadDriver(std::string driver_type, std::string driver_address, std::string driver_pkg, std::string driver_name, std::string driver_param)
        {
            skiros_msgs::RmDeviceHandle msg;
            msg.request.action = msg.request.START;
            msg.request.id_name = driver_address;
            msg.request.package = driver_pkg;
            msg.request.executable = driver_name;
            msg.request.type = driver_type;
            msg.request.parameters = driver_param;
            if(device_handler_.call(msg))
            {
                return msg.response.success;
            }
            else
            {
                connected_ = false;
                return false;
            }
        }

        bool unloadDriver(std::string device_address)
        {
            skiros_msgs::RmDeviceHandle msg;
            msg.request.action = msg.request.STOP;
            msg.request.id_name = device_address;
            if(device_handler_.call(msg))
            {
                return msg.response.success;
            }
            else
            {
                connected_ = false;
                return false;
            }

        }

        bool unloadAllDrivers()
        {
            skiros_msgs::RmDeviceHandle msg;
            msg.request.action = msg.request.STOP_ALL;
            if(device_handler_.call(msg))
            {
                return msg.response.success;
            }
            else
            {
                connected_ = false;
                return false;
            }
        }

        bool isConnected()
        {
            skiros_msgs::RmDeviceHandle msg;
            msg.request.action = msg.request.PING;
            if(device_handler_.call(msg))
            {
                connected_ = true;
                return true;
            }
            else
            {
                connected_ = false;
                return false;
            }
        }

        bool waitConnection(bool block = false)
        {
            if(connected_ == true) return true;
            ros::Time start_time = ros::Time::now();
            ros::Duration timeout(5.0);
            FINFO("[ResourceManagerInterface]Waiting connection...");
            skiros_msgs::RmDeviceHandle msg;
            msg.request.action = msg.request.PING;
            while(ros::Time::now() - start_time < timeout || block)
            {
                if(device_handler_.call(msg))
                {
                    connected_ = true;
                    break;
                }
                sleep(0.5);
            }
            if(connected_) {FINFO("[ResourceManagerInterface]Resource manager " << rm_name_ << " connected");return true;}
            else {FWARN("[ResourceManagerInterface]Connection timed out.");return false;}

        }

    private:
        ros::NodeHandle & nh_;
        std::string rm_name_;
        bool connected_;
        ros::ServiceClient device_handler_;
    };
}

#endif //RESOURCEMGRINT_HPP
