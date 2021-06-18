'''
Functions to move sonar in gazebo programatically

Contact: Robert DeBortoli (debortor@oregonstate.edu)
'''
import rospy
import pdb

from gazebo_msgs.srv import GetModelState,SetModelState
from gazebo_msgs.msg import ModelState 
import tf
import numpy as np

# [2.09,1.83,3.59,0,0.507,1.57]

def set_robot_pose(pose):
	'''
	Pose is just a list with 6 elements:
	(x, y, z, roll, pitch, yaw)
	'''
	state_msg = ModelState()
	state_msg.model_name = 'tritech_gemini_720i'
	state_msg.pose.position.x = pose[0]
	state_msg.pose.position.y = pose[1]
	state_msg.pose.position.z = pose[2]
	quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
	state_msg.pose.orientation.x = quaternion[0]
	state_msg.pose.orientation.y = quaternion[1]
	state_msg.pose.orientation.z = quaternion[2]
	state_msg.pose.orientation.w = quaternion[3]

	rospy.wait_for_service('/gazebo/set_model_state')
	set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	resp = set_state( state_msg )
	# print(resp)

def interpolate_between_poses(current_pose, new_pose, num_poses):
	'''
	'''
	# num_intermdiate_poses = 
	x = np.linspace(current_pose[0], new_pose[0], num_poses)
	y = np.linspace(current_pose[1], new_pose[1], num_poses)
	z = np.linspace(current_pose[2], new_pose[2], num_poses)
	roll  = np.linspace(current_pose[3], new_pose[3], num_poses)
	pitch = np.linspace(current_pose[4], new_pose[4], num_poses)
	yaw   = np.linspace(current_pose[5], new_pose[5], num_poses)
	return zip(x, y, z, roll, pitch, yaw)

def move_robot_to_pose(current_pose, new_pose, robot_velocity, movements_per_second):
	'''
	Move robot to pose in a smooth fashion
	'''
	trans_diff = np.array([new_pose[0] - current_pose[0],
						  new_pose[1] - current_pose[1],
						  new_pose[2] - current_pose[2]])
	trans_diff = np.sqrt(np.sum(trans_diff**2))
	# make sure to include rotation diff, o.w. we won't get pure
	# rotation being recorded
	rot_weighting = 1. # for weighting rot vs. trans
	rot_diff  = np.array([new_pose[3] - current_pose[3],
						  new_pose[4] - current_pose[4],
						  new_pose[5] - current_pose[5]])
	rot_diff  = np.sqrt(np.sum(rot_diff**2))

	pose_diff = trans_diff + rot_weighting * rot_diff

	num_intermdiate_poses = int((1./robot_velocity) * pose_diff * movements_per_second)
	intermediate_pose_list = interpolate_between_poses(current_pose, new_pose, num_intermdiate_poses)
	for intermediate_pose in intermediate_pose_list:
		set_robot_pose(intermediate_pose)
		rospy.sleep(1/movements_per_second)



if __name__ == '__main__':
	rospy.init_node('set_sonar_pose')

	robot_velocity = 0.1 # m/s
	movements_per_second = 100 # Hz

	pose_list = [[2.09,1.83,3.59,0,0.507,1.57],
				 [2.09,1.83,3.8,0,0.507,1.9],
				 [2.09,1.83,3.3,0,0.507,1.2],
				 [2,2,3.59,0,0.507,1.57],
				 [2.09,2.3,3.2,0,0.507,1.57],
				 [2.09,2.3,3.2,0,0.507,1.9],
				 [2.09,2.3,3.2,0,0.507,1.2]]

	for i in range(1,len(pose_list)):
		current_pose = pose_list[i-1]
		current_pose[4] += 0.174533 # to account for tilt of tritech down 10 degrees
		print('confirm this tilting is necessary')
		move_robot_to_pose(current_pose, pose_list[i], robot_velocity, movements_per_second)


