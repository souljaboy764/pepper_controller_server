#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <controller_interface/controller.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <pepper_controller_server/JointTarget.h>

namespace pepper_controller_server
{

//
// Position Controller
/*
	Position Controller class derives from the controller_interface::ControllerBase class from ros_control.
*/
class PositionStiffnessController : public controller_interface::ControllerBase
{
public:

	// initRequest
	/*
		Function to initialize the controller that is launched when the controller is loaded
			- robot_hw: the robot hardware interface
			- root_nh
			- controller_nh: NodeHandle in the namespace of the controller
			- claimed_resources: actuated joints for this controller
			returns true if initializon was successful, false otherwise

	*/
	bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
									 ros::NodeHandle& controller_nh, ClaimedResources& claimed_resources);

	// update
	/*
		Function that performs the control loop at every control cycle once it is started and before it is stopped.
			- time: the current time
			- period: the time passed since thje last time call
	*/
	void update(const ros::Time& time, const ros::Duration& period);
	
	// starting
	/*
		Function to start the controller once it has been loaded. It is call before the first call to update.
			- time: the current time
	*/
	void starting(const ros::Time& time);

	// stopping
	/*
		Function to stop the controller once it has been loaded. It is call after the last call to update
			- time: the current time
	*/
	void stopping(const ros::Time& time);

	// subscribing/service
	/*
		Function for the backend to the service call to set the joint angle and stiffness targets
			- req: the service request with the targets
			- resp: the service respose with the result of the success of the operation
	*/
	bool JointTrajCb(pepper_controller_server::JointTarget::Request &req, pepper_controller_server::JointTarget::Response &resp);

protected:

	 // init
	/*
		Function to initialize controller that is called from the initRequest function
	*/
	bool init(hardware_interface::PositionJointInterface* position_iface,
						hardware_interface::EffortJointInterface* effort_iface,
						hardware_interface::JointStateInterface* joint_state_iface,
						ros::NodeHandle& root_nh, ros::NodeHandle& control_nh);

	std::map<std::string, hardware_interface::JointHandle> actuated_joints_;  /*!< Map with the actuated joints and his parameters + hardware interface to read and write commands*/
	std::map<std::string, hardware_interface::JointHandle> actuated_joints_efforts_;  /*!< Map with the actuated joints and his parameters + hardware interface to read and write commands*/

	std::vector<std::string> joint_names_; // Vector with the joint names of all the joints
	std::vector<float> joint_targets_; // Vector with the joint targets of all the joints
	float stiffness_target_; // Stiffness target for Pepper
	ros::ServiceServer joint_targets_srv_; // Server for the Joint Target Service 
};
}

// Exports the ComplianceTutorial as a plugin of ros_control
PLUGINLIB_EXPORT_CLASS(pepper_controller_server::PositionStiffnessController, controller_interface::ControllerBase)