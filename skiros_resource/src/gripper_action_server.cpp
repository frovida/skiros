#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <skiros_msgs/GripperAction.h>
#include <math.h>
#include <skiros_resource/exceptions.h>
#include <skiros_resource/base_gripper_device.h>
#include <boost/thread.hpp>
#include <skiros_resource/gripper_action_server.h>
#include<boost/foreach.hpp>


namespace skiros_resource
{
  bool GripperActionServer::start()
  {
      //TODO: preempt doesn't work, but if i don't preempt the goals go in queue
     // action_srv_.registerPreemptCallback(boost::bind(&GripperActionServer::preemptCB, this));
      //THe proxy initialization should be handled from the device manager if we decide to have multiple action srvs
      try
      {
        gripper_proxy_->init(node_h_);
      }
      catch(skiros_device::EndEffectorException e)
      {
          ROS_ERROR_STREAM("[GripperActionServer::start] Gripper " << action_name_ << " failed to initialize.");
          return false;
      }
       action_srv_.start();
       //timer_ = node_h_.createTimer(ros::Duration(0.5),&GripperActionServer::timerCallback, this);
       ROS_INFO_STREAM("[GripperActionServer::start] Gripper " << action_name_ << " ready to move.");
       return true;
  }

  void GripperActionServer::preemptCB()
  {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      gripper_proxy_->stop();
      // set the action state to preempted
      action_srv_.setPreempted();
  }

  void GripperActionServer::timerCallback(const ros::TimerEvent& e)
  {
      boost::mutex::scoped_lock lock(feedback_mux_);
      skiros_msgs::GripperFeedback old_f = feedback_;
      gripper_proxy_->getFeedback(feedback_.fingersFB);
      feedback_.graspMode = gripper_proxy_->getCurrentModality();
      if(feedback_.graspMode!=old_f.graspMode)
      {
          action_srv_.publishFeedback(feedback_);
      }
  }

  void GripperActionServer::runCallback(const skiros_msgs::GripperGoalConstPtr &goal)
  {
	  /*TODO: implement a feedback to allow the understated ctrl
	   *
	   * TODO: implement a force based ctrl (yes, but in the action client):
	   * -move fingers with the speed (required from user) but with very low force, so they stop when they just feel the object
		-once all the fingers get the same (low) force application, complete the movement applying the user required force
	   *
	   *TODO: in the future will be possible to allow different ctrls. Maybe with a service to configure which one to use.
	   *
	   *Note: in case we implement the ctrl in the action server, then it would become a primitive.
	   *Note: tolerance is for now not in use
	   */
    // helper variables
	bool success = true;
    fingers_stopped_ = false;

    try
    {
        if(goal->graspMode>=0)gripper_proxy_->changeModality(goal->graspMode);
    }
    catch(skiros_device::EndEffectorException& e)
    {
       	action_srv_.setAborted();
       	return;
    }
    skiros_msgs::GripperGoal goal_copy = *goal;

    gripper_proxy_->getFeedback(feedback_.fingersFB);
    for(int i=0;i<feedback_.fingersFB.size();i++)
    {
        for(int u=0;i<2;i++)
        {
            if(goal_copy.fingersG[i].pos[u]<0)
                goal_copy.fingersG[i].pos[u] = feedback_.fingersFB[i].pos[u];
        }
    }

    //Start action execution
    try
    {
        gripper_proxy_->fingersToPos(goal_copy.fingersNum, goal_copy.fingersG);
    }
    catch(skiros_device::EndEffectorException& e)
    {
       	action_srv_.setAborted();
       	return;
    }

    ros::Rate r(10);
    bool loopAgain = true;
    //Loop until the goal isn't reached
    boost::mutex::scoped_lock lock(feedback_mux_);
    lock.unlock();
    while(loopAgain && ros::ok())
    {
        //ROS_INFO("%i", feedback_.getpos);
      lock.lock();
      gripper_proxy_->getFeedback(feedback_.fingersFB);
      feedback_.graspMode = gripper_proxy_->getCurrentModality();
      lock.unlock();

      // publish the feedback
      action_srv_.publishFeedback(feedback_);

      r.sleep();

      if(action_srv_.isPreemptRequested())
      {
          ROS_INFO("%s: Preempted", action_name_.c_str());
          gripper_proxy_->stop();
          // set the action state to preempted
          action_srv_.setPreempted();
          return;
      }
      loopAgain = !this->checkIfGoalAchieved(goal_copy, &success);
    }

    if(success)
    {
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      action_srv_.setSucceeded(result_);
    }
    else
    {
       	action_srv_.setAborted();
    }
  }

  bool GripperActionServer::checkIfGoalAchieved(skiros_msgs::GripperGoal goal, bool *success)
  {
	  	static ros::Time timeout;

	  	//First I check if fingers are moving..
	    for(int i=0; i < feedback_.fingersFB.size();i++)
	    {
	      //If a finger is still moving...
	      if(!feedback_.fingersFB.at(i).state[0] || !feedback_.fingersFB.at(i).state[1])
	      {
	    	  //I loop again
	    	  fingers_stopped_ = false;
	    	  return false;
	      }
	    }


		//If not, I check if the fingers are at the goal position

        int goalPos[2];
        goalPos[0] = goal.fingersG.at(0).pos[0];
        goalPos[1] = goal.fingersG.at(0).pos[1];
        bool at_goal[2];
        at_goal[0] = at_goal[1] = at_goal[2] = true;
		*success = true;

		//If is the firstTime I find all the fingers stopped, I reset the timer
		if(!fingers_stopped_)
		{
			fingers_stopped_ = true;
			timeout = ros::Time::now();
		}
        bool end_iteration = false;
        //Action end: (i) when goal is reached or (ii) after a timeout
        if(ros::Time::now() - timeout  > ros::Duration(1.0))
            end_iteration = true;

        if(end_iteration)
            ROS_INFO("The action ended with: ");

	    for(int i=0; i < feedback_.fingersFB.size();i++)
        {
            //ROS_INFO_STREAM(feedback_.fingersFB.at(i).pos[0] << " " << feedback_.fingersFB.at(i).appliedFrc[0]);
            if(goal.fingersNum == feedback_.fingersFB.size()) //In multifinger ctrl I have a goal for every finger
            {
                goalPos[0] = goal.fingersG.at(i).pos[0];
                goalPos[1] = goal.fingersG.at(i).pos[1];
            }
            //TODO: add the scissor axis check: abs(feedback_.fingersFB.at(i).pos[1]-goalPos[1]) > goal.tolerance
            if(abs(feedback_.fingersFB.at(i).pos[0]-goalPos[0]) <= goal.tolerance || (goal.fingersNum != feedback_.fingersFB.size() && i>0))
            {
                if(end_iteration)
                    ROS_INFO("F%i at goal.",  i);
                at_goal[i] = true;
            }
            else
            {
                at_goal[i] = false;
                if(end_iteration)
                    ROS_INFO("F%i Goal: %i %i, Actual: %i %i, Result: %i %i",  i, goalPos[0], goalPos[1], (int)feedback_.fingersFB.at(i).pos[0], (int)feedback_.fingersFB.at(i).pos[1], (int)feedback_.fingersFB.at(i).pos[0]-goalPos[0], (int)feedback_.fingersFB.at(i).pos[1]-goalPos[1]);
            }
        }
        if(at_goal[0] && at_goal[1] && at_goal[2])
        {
            *success = true;
            return true;
        }
        if(end_iteration)
        {
            *success = false;
            return true;
        }
	    //If a finger is still not in position I loop again
	    return false;
  }

}
