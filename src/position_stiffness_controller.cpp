#include <pepper_controller_server/position_stiffness_controller.h>

#include <numeric>
#include <vector>
#include <iostream>

namespace pepper_controller_server
{
bool PositionStiffnessController::initRequest(hardware_interface::RobotHW* robot_hw,
									ros::NodeHandle& root_nh,
									ros::NodeHandle& controller_nh,
									ClaimedResources& claimed_resources)
{
	// Check if construction finished cleanly
	if (state_ != ControllerState::CONSTRUCTED)
	{
		ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
		return false;
	}

	// Get a pointer to the joint effort control interface
	hardware_interface::PositionJointInterface* position_iface =
			robot_hw->get<hardware_interface::PositionJointInterface>();

	if (!position_iface)
	{
		ROS_ERROR("This controller requires a hardware interface of type PositionJointInterface."
							" Make sure this is registered in the hardware_interface::RobotHW class.");
		return false;
	}

	hardware_interface::EffortJointInterface* effort_iface =
			robot_hw->get<hardware_interface::EffortJointInterface>();

	if (!effort_iface)
	{
		ROS_ERROR("This controller requires a hardware interface of type EffortJointInterface."
							" Make sure this is registered in the hardware_interface::RobotHW class.");
		return false;
	}

	// Get a pointer to the joint position control interface
	hardware_interface::JointStateInterface* joint_state_iface =
			robot_hw->get<hardware_interface::JointStateInterface>();
	if (!joint_state_iface)
	{
		ROS_ERROR("This controller requires a hardware interface of type JointStateInterface."
							" Make sure this is registered in the hardware_interface::RobotHW class.");
		return false;
	}

	// Clear resources associated at both interfaces
	position_iface->clearClaims();
	joint_state_iface->clearClaims();

	if (!init(position_iface, effort_iface, joint_state_iface, root_nh, controller_nh))
	{
		ROS_ERROR("Failed to initialize the controller");
		return false;
	}

	// Saves the resources claimed by this controller
	claimed_resources.push_back(hardware_interface::InterfaceResources(
			hardware_interface::internal::demangledTypeName<hardware_interface::PositionJointInterface>(),
			position_iface->getClaims()));
	position_iface->clearClaims();

	// Changes state to INITIALIZED
	state_ = ControllerState::INITIALIZED;
	ROS_INFO_STREAM("Loaded position controller");
	return true;
}

bool PositionStiffnessController::init(hardware_interface::PositionJointInterface* position_iface,
								hardware_interface::EffortJointInterface* effort_iface,
								hardware_interface::JointStateInterface* joint_state_iface,
								ros::NodeHandle& root_nh, ros::NodeHandle& control_nh)
{
	ROS_INFO_STREAM("Loading position controller");
	control_nh.getParam("joints", joint_names_);

	for (size_t i = 0; i < joint_names_.size(); i++)
	{
		try
		{
			// Try to get an effort interface handle to command the joint in position
			hardware_interface::JointHandle joint_handle = position_iface->getHandle(joint_names_[i]);
			// Creates an actuated joint and insert in the map of actuated joints
			actuated_joints_.insert(std::make_pair(joint_names_[i], joint_handle));

			hardware_interface::JointHandle joint_effort_handle = effort_iface->getHandle(joint_names_[i]);
			actuated_joints_efforts_.insert(std::make_pair(joint_names_[i], joint_effort_handle));
		}
		catch (...)
		{
			ROS_ERROR_STREAM("Could not find joint " << joint_names_[i] << " with Position interface");
			return false;
		}
	}

	if(joint_targets_.size()==0)
		for (size_t i = 0; i < joint_names_.size(); i++)
			joint_targets_.push_back(actuated_joints_[joint_names_[i]].getPosition());
	
	stiffness_target_ = actuated_joints_efforts_[joint_names_[0]].getEffort();

	joint_targets_srv_ = control_nh.advertiseService("goal", &PositionStiffnessController::JointTrajCb, this);

	return true;
}

bool PositionStiffnessController::JointTrajCb(pepper_controller_server::JointTarget::Request &req, 
									pepper_controller_server::JointTarget::Response &resp)
{
	if(req.target.points.size()!=1 || req.target.joint_names.size()!=joint_names_.size() || req.target.points[0].positions.size()!=joint_names_.size())
		resp.result = false;
	else
	{
		for (size_t i = 0; i < joint_names_.size(); i++)
			joint_targets_[i] = req.target.points[0].positions[i];
		stiffness_target_ = static_cast<float>(req.target.points[0].effort[0]);
		resp.result = true;
	}
	return resp.result;
}

void PositionStiffnessController::update(const ros::Time& time, const ros::Duration& period)
{
	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		actuated_joints_[joint_names_[i]].setCommand(joint_targets_[i]);
		actuated_joints_efforts_[joint_names_[i]].setCommand(stiffness_target_);
	}
}


void PositionStiffnessController::starting(const ros::Time& time)
{
}

void PositionStiffnessController::stopping(const ros::Time& time)
{
}
}
