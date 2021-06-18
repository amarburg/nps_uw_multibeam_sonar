'''
Helper functions for recording sonar data and the accompanying
3D ground truth info
Some code inspired by: 
https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

Contact: Robert DeBortoli (debortor@oregonstate.edu)
'''

import numpy as np
import pdb
import ros_numpy
import rospy
import cv2
from sklearn.impute import KNNImputer

def raw2cvimg(msg):
	'''
	'''
	num_range_bins = len(msg.ranges)
	num_beams = len(msg.azimuth_angles)
	int_img =  np.fromstring(msg.intensities, dtype=np.uint8)
	return int_img.reshape(num_range_bins, num_beams)

def headertime2str(header):
	'''
	'''
	return str(header.stamp.secs)+'_'+str(header.stamp.nsecs)

def pcmsg2pc(pc_msg):
	'''
	Converts ros pc message to numpy pc
	Pc is in sonar local frame
	'''
	dd = ros_numpy.numpify(pc_msg)
	pc = np.zeros((dd.shape[0],3))
	pc[:,0] = dd['x']
	pc[:,1] = dd['y']
	pc[:,2] = dd['z']
	return pc

def cartesian2polar(pc):
	'''
	'''
	r = np.sqrt(pc[:,0]**2 + pc[:,1]**2 + pc[:,2]**2)
	# note the axis got switched between gazebo and here, z is forward
	bearing = np.arctan2(pc[:,0],pc[:,2]) 
	elev = np.arcsin(pc[:,1]/r)
	return r, bearing, elev

def pc2elevmap(pc_msg, num_elev_classes, sonar_params):
	'''
	Convert 3D pc to elev. map which can be used by ElevateNet
	'''
	max_range, min_range, range_res, num_beams, \
		num_rows, horizontal_fov , elev_aperature = sonar_params

	pc = pcmsg2pc(pc_msg)

	r, bearing, elev = cartesian2polar(pc)

	# remove any points not within the sonar frustrum
	pc = pc[(r < max_range) & (r > min_range) & \
			(bearing > horizontal_fov/-2.) & (bearing < horizontal_fov/2.) & \
			(elev > elev_aperature/-2.) & (elev < elev_aperature/2.)]

	# todo: make faster
	r, bearing, elev = cartesian2polar(pc)
	
	# generate image
	elev_classes = ((elev + (elev_aperature/2.)) / (elev_aperature/num_elev_classes)).astype('int')
	img_cols = ((bearing + horizontal_fov/2.) * (num_beams/horizontal_fov)).astype('int')
	img_rows = ((r-min_range)*(num_rows/(max_range-min_range))).astype('int')
	# print("rangeres:", ((max_range-min_range)/num_rows))
	# img_rows = ((r)*(1/range_res)).astype('int')
	img_coords = zip(img_rows, img_cols, elev_classes)
	
	elev_map = np.zeros((num_rows, num_beams))
	for pixel in img_coords:
		if (elev_map[pixel[0], pixel[1]] > pixel[2]):
			# use the maximum elev angle
			continue
		elev_map[pixel[0], pixel[1]] = pixel[2]

	return elev_map

def write_elev_map(elev_map, sonar_img, experiment_dir, time, num_elev_classes, also_visualize=True):
	'''
	also_visualize scales the pixels so they're visible and we can do a sanity check
	'''
	cv2.imwrite(experiment_dir + str(time) + '_raw_elev_orig.png', elev_map)
	elev_map_orig = elev_map
	if elev_map.max() > 0:
		cv2.imwrite(experiment_dir + str(time) + '_scaled_elev_orig.png', elev_map * int(255/elev_map.max()))
	else:
		cv2.imwrite(experiment_dir + str(time) + '_scaled_elev_orig.png', elev_map)

	# smooth so we're not missing pixels all over the place in the elev_map
	elev_map =  cv2.dilate(elev_map, np.ones((5,5)))
	cv2.imwrite(experiment_dir + str(time) + '_raw_elev.png', elev_map)
	if elev_map.max() > 0:
		cv2.imwrite(experiment_dir + str(time) + '_scaled_elev.png', elev_map * int(255/elev_map.max()))
	else:
		cv2.imwrite(experiment_dir + str(time) + '_scaled_elev.png', elev_map)

	overlay_img = np.zeros((elev_map.shape[0], elev_map.shape[1], 3))
	
	# blue is elevation map
	overlay_img[:,:,0] =  (elev_map_orig * int(255/elev_map.max()))*0.75
	# green is sonar image
	overlay_img[:,:,1] =  sonar_img * 0.75
	cv2.imwrite(experiment_dir + str(time) + '_overlay.png', overlay_img)
	# pdb.set_trace()
	
# def fan_image_callback(msg):
# 	cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
# 	time = headertime2str(msg.header)
# 	cv2.imwrite(experiment_dir + str(time) + '.png', cv2_img)