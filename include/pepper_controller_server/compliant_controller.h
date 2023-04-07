#include <pepper_controller_server/position_stiffness_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace pepper_controller_server
{

//! 
// Compliant Handshakeing Controller
/*!
	Compliant Handshakeing Controller class derives from the controller_interface::ControllerBase class from ros_control.
*/
class CompliantController : public PositionStiffnessController
{
public:
	// update
	/*
		Function that performs the control loop at every control cycle once it is started and before it is stopped.
			- time: the current time
			- period: the time passed since thje last time call
	*/
	void update(const ros::Time& time, const ros::Duration& period);

private:

	double q, qdot, qdot_t, qddot, y, eps, qddot_d, qdot_d, q_d, dt;
};
}

// Exports the ComplianceTutorial as a plugin of ros_control
PLUGINLIB_EXPORT_CLASS(pepper_controller_server::CompliantController, controller_interface::ControllerBase)