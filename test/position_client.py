import numpy as np

import rospy
from pepper_controller_server.srv import JointTarget
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('talker')
# robot_traj_publisher = rospy.Publisher("/pepper_dcm/RightArm_controller/command", Float64MultiArray, queue_size=1)
# stiffness_pub = rospy.Publisher('/pepper_dcm/set_stiffness', Float32, queue_size=10)
# rospy.wait_for_service('/joint_targets')
send_target = rospy.ServiceProxy('/pepper_dcm/RightArm_controller/goal', JointTarget)


rate = rospy.Rate(200)
rate.sleep()
default_joint_angles = [np.pi/2, -0.109, np.pi/4, 0.009, np.pi/4]
raised_angles = np.array([0.5, -0.109, np.pi/4, 0.75, np.pi/4]).astype(np.float64).tolist()
joint_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
num_joints = len(joint_names)


joint_trajectory = JointTrajectory()
joint_trajectory.header.frame_id = "base_link"
joint_trajectory.joint_names = joint_names
joint_trajectory.points.append(JointTrajectoryPoint())
joint_trajectory.points[0].time_from_start = rospy.Duration.from_sec(0.5)

joint_trajectory.header.seq = 0
joint_trajectory.points[0].positions = default_joint_angles
joint_trajectory.points[0].effort = (0.2*np.ones(num_joints)).tolist()
rate.sleep()
print('Looping')
if True:
# for i in range(10):
# while not rospy.is_shutdown():
	joint_trajectory.header.seq += 1
	joint_trajectory.header.stamp = rospy.Time.now()
	print(send_target(joint_trajectory))
	# robot_traj_publisher.publish(Float64MultiArray(data=raised_angles))
	rate.sleep()
	print(joint_trajectory.header.seq)
