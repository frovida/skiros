#ifndef GRIPPERACTIONCLIENT_H
#define GRIPPERACTIONCLIENT_H

#include <ros/ros.h>
#include <skiros_msgs/GripperAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

namespace skiros_primitive
{

namespace rb3_modes
{
    static const int  BASIC 	= 0;
    static const int  PINCH 	= 1;
    static const int  WIDE 		= 2;
    static const int  SCISSOR 	= 3;
    static const int  ADVANCED 	= 4;
}

/** \brief  Generate the gripper action client.
 * */

class GripperClientClass{
public:
    GripperClientClass(std::string address): curr_modality_(0),
        //Set up the client. It's publishing to topic "", and is set to auto-spin
        gripper_ac_(address.c_str(), true),
        //Stores the name
        action_name_(address)
    {
        //Get connection to a server
        ROS_INFO("%s Waiting For Server...", action_name_.c_str());
        //Wait for the connection to be valid
        gripper_ac_.waitForServer();
        ROS_INFO("%s Got a Server...", action_name_.c_str());
        lastActionSucces_ = true;
    } // action_name_(name) - end
	/*void init(boost::shared_ptr<BaseGripperPrimitive> primitive)
	{

	}*/

    // Called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
            const skiros_msgs::GripperResultConstPtr& result)
    {
        //ROS_INFO("Finished in state [%s]", state.toString().c_str());
        lastActionSucces_ = true;
        isActionOnGoing_ = false;
    }


    // Called once when the goal becomes active
    void activeCb()
    {
      //  ROS_INFO("Goal just went active...");
    }

    // Called every time feedback is received for the goal
    void feedbackCb(const skiros_msgs::GripperFeedbackConstPtr& feedback)
    {
    	/*        primitive->sendFB();
        for(int i=0;i<feedback->fingersFB.size();i++)
        {
            ROS_INFO("Finger %i =  state: %i %i pos:%i %i Frc: %f %f", i,
            		feedback->fingersFB.at(i).state[0],feedback->fingersFB.at(i).state[1],
            		feedback->fingersFB.at(i).pos[0], feedback->fingersFB.at(i).pos[1],
            		feedback->fingersFB.at(i).appliedFrc[0],feedback->fingersFB.at(i).appliedFrc[0]);
        }*/
        fingers_ = feedback->fingersFB;
        curr_modality_ = feedback->graspMode;
    }

    void cancelGoal()
    {
        if(isActionOnGoing_)
        {
            isActionOnGoing_ = false;
            lastActionSucces_ = false;
        	gripper_ac_.cancelGoal();
        }
    }

    /*
     * g - goal c -constraints y - stopcondition
     * Goal: object in the gripper
     * Constraints: force limit, ..
     * Stop conditions: timeout, .. */

    void grasp(int grasp_mode = -1, int position = 255, int force_limit = 255, int velocity = 255, int tolerance = 0, int num_of_fingers=3)
    {
        skiros_msgs::GripperGoal goal;
        skiros_msgs::Finger f;
        goal.graspMode = grasp_mode;
        goal.tolerance = tolerance;
        goal.fingersNum = num_of_fingers;
        f.pos[0] = position;
        f.spd[0] = velocity;
        f.frc[0] = force_limit;
        for(int i=0;i<num_of_fingers;i++)
        {
            goal.fingersG.push_back(f);
        }
        //Once again, have to used boost::bind because you are inside a class
        gripper_ac_.sendGoal(goal, boost::bind(&GripperClientClass::doneCb, this, _1, _2),
                             boost::bind(&GripperClientClass::activeCb, this),
                             boost::bind(&GripperClientClass::feedbackCb, this, _1));
        isActionOnGoing_ = true;
        lastActionSucces_ = false;
    } // void send - end

    void release(int position = 0, int force_limit = 255, int velocity = 255, int tolerance = 6)
    {
        skiros_msgs::GripperGoal goal;
        skiros_msgs::Finger f;
        goal.graspMode = -1;
        goal.tolerance = tolerance;
        f.pos[0] = position;
        f.spd[0] = velocity;
        f.frc[0] = force_limit;
        goal.fingersG.push_back(f);
        gripper_ac_.sendGoal(goal, boost::bind(&GripperClientClass::doneCb, this, _1, _2),
                             boost::bind(&GripperClientClass::activeCb, this),
                             boost::bind(&GripperClientClass::feedbackCb, this, _1));
        isActionOnGoing_ = true;
        lastActionSucces_ = false;
    }

    void waitFinish()
    {
        while(isActionOnGoing_ && ros::ok())
            ros::Duration(0.1).sleep();
    }

    void move(int fingersNum, int graspMode, std::vector<int> inputPos, int spd, int frc, int tolerance = 0)
    {
        skiros_msgs::GripperGoal goal;
        skiros_msgs::Finger f;
        goal.graspMode = graspMode;
        goal.tolerance = tolerance;
        goal.fingersNum = fingersNum;
        if(fingersNum == 1)
        {
			f.pos[0] = inputPos.at(0);
			f.spd[0] = spd;
			f.frc[0] = frc;
			goal.fingersG.push_back(f);
        }
        else
        {
			f.pos[0] = inputPos.at(0);
			f.spd[0] = spd;
			f.frc[0] = frc;
			goal.fingersG.push_back(f);
			f.pos[0] = inputPos.at(1);
			f.spd[0] = spd;
			f.frc[0] = frc;
			if(graspMode==4)
			{
				f.pos[1] = inputPos.at(3);
				f.spd[1] = spd;
				f.frc[1] = frc;
			}
			goal.fingersG.push_back(f);
			f.pos[0] = inputPos.at(2);
			f.spd[0] = spd;
			f.frc[0] = frc;
			if(graspMode==4)
			{
				f.pos[1] = inputPos.at(4);
				f.spd[1] = spd;
				f.frc[1] = frc;
			}
			goal.fingersG.push_back(f);
        }
        //Once again, have to used boost::bind because you are inside a class
        gripper_ac_.sendGoal(goal, boost::bind(&GripperClientClass::doneCb, this, _1, _2),
                             boost::bind(&GripperClientClass::activeCb, this),
                             boost::bind(&GripperClientClass::feedbackCb, this, _1));
        isActionOnGoing_ = true;
    }

    inline bool isActionSuccessfull(){return lastActionSucces_;}

    inline bool isOpen(){return this->isAtPosition(0, 6);}

    inline bool isClosed(){return this->isAtPosition(255, 6);}

    bool isAtPosition(int inputPos, int tolerance = 0) //TODO: support for specific finger check
    {
        for(int i=0;i<fingers_.size();i++)
        {
            if(curr_modality_ != rb3_modes::SCISSOR)
            {
                //ROS_INFO_STREAM(std::abs((double)fingers_[i].pos[0]-inputPos));
                if(std::abs((double)fingers_[i].pos[0]-inputPos)>tolerance)
                    return false;
            }
            else
            {
                //ROS_INFO_STREAM(std::abs((double)fingers_[i].pos[1]-inputPos));
                if(i!=0) //The thumb doesn't have a scissor axis. TODO: Check that this works..
                    if(std::abs((double)fingers_[i].pos[1]-inputPos)>tolerance)
                        return false;
            }
        }
        return true;
    }

    int getCurrentModality(bool update=true)
    {
        if(update)
        {
            grasp(-1,-1);
            waitFinish();
        }
        return curr_modality_;
    }

private:
    actionlib::SimpleActionClient<skiros_msgs::GripperAction> gripper_ac_;
    std::vector<skiros_msgs::FingerFB> fingers_;
    int curr_modality_;
    std::string action_name_;
    bool isActionOnGoing_;
    bool lastActionSucces_;
}; // class GripperClientClass - end

}
#endif // GRIPPERACTIONCLIENT_H
