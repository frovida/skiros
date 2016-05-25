#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skiros_msgs/GripperAction.h>
#include <math.h>
#include <skiros_resource/exceptions.h>
#include <skiros_resource/base_gripper_device.h>

namespace skiros_resource
{

/** \brief  Generate the gripper action server. This class use "proxies" to interact with the hardware,
 * to make the server independent from hardware. A pointer to a proxy must be provided when instantiating. */

//TODO: Define quickly the most generic action message, which should remain fixed.

class GripperActionServer
{
protected:

  ros::NodeHandle node_h_; //TODO make const
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<skiros_msgs::GripperAction> action_srv_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  skiros_msgs::GripperFeedback feedback_;
  skiros_msgs::GripperResult result_;
  boost::shared_ptr<BaseGripperDevice> gripper_proxy_;
  bool fingers_stopped_;

public:

  GripperActionServer(std::string name, boost::shared_ptr<BaseGripperDevice> gripper_proxy, ros::NodeHandle node_h) :
    node_h_(node_h),
    action_srv_(node_h, name, boost::bind(&GripperActionServer::runCallback, this, _1), false),
    action_name_(name),
    gripper_proxy_(gripper_proxy)
  {
  }

  ~GripperActionServer(void){}

  bool start();
  void preemptCB();

  void timerCallback(const ros::TimerEvent& e);

  void runCallback(const skiros_msgs::GripperGoalConstPtr &goal);
  bool checkIfGoalAchieved(skiros_msgs::GripperGoal goal, bool *success);
private:
  ros::Timer timer_;
  boost::mutex feedback_mux_;
};
}
