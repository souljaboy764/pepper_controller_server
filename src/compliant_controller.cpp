#include <pepper_controller_server/compliant_controller.h>

#include <numeric>
#include <vector>
#include <iostream>

namespace pepper_controller_server
{

void CompliantController::update(const ros::Time& time, const ros::Duration& period)
{
	int K = 5000, D = 10, M = 1;
	dt = period.toSec();
	for (size_t i = 0; i < joint_names_.size(); ++i)
	{
		q = actuated_joints_[joint_names_[i]].getPosition();
		qdot_t = actuated_joints_[joint_names_[i]].getVelocity();
		qddot = (qdot_t - qdot)/dt;
		qdot = qdot_t;
		eps = joint_targets_[i]-q;
		y = -M*qddot + -D*qdot + K*eps;
		q_d = q + (qdot + y*dt)*dt;
		if(i==0)
		{
			std::cout<<"q= "<<q<<std::endl;
			std::cout<<"qdot= "<<qdot<<std::endl;
			std::cout<<"qddot= "<<qddot<<std::endl;
			std::cout<<"target= "<<joint_targets_[i]<<std::endl;
			std::cout<<"eps= "<< eps<<std::endl;
			std::cout<<"y= "<<y<<std::endl;
			std::cout<<"dt= "<<dt<<std::endl;
			std::cout<<"qdot_r = qdot + y*dt: "<<qdot + y*dt<<std::endl;
			std::cout<<"qr = q + qdot_r*dt: "<<q_d<<std::endl;
			std::cout<<std::endl;
		}
		actuated_joints_[joint_names_[i]].setCommand(q_d);
		actuated_joints_efforts_[joint_names_[i]].setCommand(stiffness_target_);
		// std::cout<<qdot<<" ";
	}
	std::cout<<std::endl;
}

}
