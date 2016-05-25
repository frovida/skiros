#include <vector>
#include <sys/stat.h>
#include <skiros_primitive/gripper_action_client.h>
#include <skiros_primitive/GripperTestConfig.h>
#include <dynamic_reconfigure/server.h>

boost::shared_ptr<skiros_primitive::GripperClientClass> gripper_ac_ptr;

void callback(skiros_primitive::GripperTestConfig &config, uint32_t level) {
    if(config.isAtPosition)
        ROS_INFO_STREAM(gripper_ac_ptr->isAtPosition(255, 6)? "True":"False");
    if(config.grasp)
        gripper_ac_ptr->grasp();
    if(config.release)
        gripper_ac_ptr->release();
    if(config.stop)
        gripper_ac_ptr->cancelGoal();
}


using namespace std;
using namespace skiros_primitive;

/////////////////////////////////////////////////////////////////////////////
// MAIN
/////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv){

	// Initialise the ros node
	ros::init(argc, argv, "test_primitive_node"); //ros::this_node::getName()
	/*
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
     */
	ros::NodeHandle node_h;
    if(argc < 2)
    {
        std::cout << "Usage: test_gripper_action <action server name>" << std::endl;
        return -1;
    }
    std::string server_name(argv[1]);
    //Initialize the gripper action client
    gripper_ac_ptr.reset(new skiros_primitive::GripperClientClass(server_name));

    // Use multiple threads to handle callbacks
	ros::AsyncSpinner spinner(3);
    spinner.start();

    dynamic_reconfigure::Server<skiros_primitive::GripperTestConfig> server;
    dynamic_reconfigure::Server<skiros_primitive::GripperTestConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);



    ros::spin();

	spinner.stop();

	return 0;
} // main
