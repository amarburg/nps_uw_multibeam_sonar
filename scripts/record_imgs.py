'''
Rospy node to record sonar images
Much of the code inspired by: 
https://answers.ros.org/question/283724/saving-images-with-image_saver-with-timestamp/

Contact: Robert DeBortoli (debortor@oregonstate.edu)
'''


# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
from acoustic_msgs.msg import SonarImage

from cv_bridge import CvBridge, CvBridgeError
import cv2
from datetime import datetime
import os
import numpy as np
import time

experiment_dir = '/home/bob/Downloads/' + datetime.now().strftime('%Y-%m-%d-%H-%M-%S--%f')[:-3] + '/'
os.mkdir(experiment_dir)
raw_dir = experiment_dir+'raw_imgs/'
os.mkdir(raw_dir)
fan_dir = experiment_dir+'fan_imgs/'
os.mkdir(fan_dir)
fan_image_topic = "/tritech_gemini_720i/sonar_image"
raw_image_topic = "/tritech_gemini_720i/sonar_image_raw"

bridge = CvBridge()

def fan_image_callback(msg):
		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
		time = msg.header.stamp
		cv2.imwrite(experiment_dir + str(time) + '.png', cv2_img)

def raw2cvimg(msg):
	'''
	'''
	num_range_bins = len(msg.ranges)
	num_beams = len(msg.azimuth_angles)
	print('num range/ num beams:', num_range_bins, num_beams)
	img_list = []
	pixel_count = 0
	for r in range(num_range_bins):	
		img_list.append([])
		for b in range(num_beams):
			img_list[-1].append(ord(msg.intensities[pixel_count]))  #ord(msg.intensities[r*num_range_bins + b]))
			pixel_count += 1
	img = np.array(img_list)
	return img

def raw_image_callback(msg):
		# cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
		print("---------------")
		print(len(msg.ranges))
		print(len(msg.intensities))
		raw_img = raw2cvimg(msg)
		time = msg.header.stamp
		cv2.imwrite(raw_dir + str(time) + '_raw.png', raw_img)

def main():
		rospy.init_node('sonar_image_writer')    
		# rospy.Subscriber(fan_image_topic, Image, fan_image_callback) 
		rospy.Subscriber(raw_image_topic, SonarImage, raw_image_callback)    
		rospy.spin()

if __name__ == '__main__':
		main()