'''
Rospy node to record sonar images and 3D ground truth info

Contact: Robert DeBortoli (debortor@oregonstate.edu)
'''
import rospy
from sensor_msgs.msg import Image, PointCloud2
from acoustic_msgs.msg import SonarImage
import message_filters

from cv_bridge import CvBridge, CvBridgeError
import cv2
from datetime import datetime
import os
import numpy as np
from utils import raw2cvimg, headertime2str, pc2elevmap, write_elev_map

experiment_dir = '/media/bob/BackupHDD/bob/Winter2021/Research/sonar_reconstruction/test_data/' +\
								  datetime.now().strftime('%Y-%m-%d-%H-%M-%S--%f')[:-3] + '/'
os.mkdir(experiment_dir)
raw_image_topic = "/tritech_gemini_720i/sonar_image_raw"
pc_topic = "/tritech_gemini_720i/point_cloud"
NUM_ELEV_CLASSES = 10


max_range = rospy.get_param("/sonar_img/max_range")
min_range = rospy.get_param("/sonar_img/min_range")
range_res = rospy.get_param("/sonar_img/range_res")
num_beams = rospy.get_param("/sonar_img/num_beams")
num_rows = rospy.get_param("/sonar_img/num_rows")
horizontal_fov = rospy.get_param("/sonar_img/horizontal_fov")
elev_aperature = rospy.get_param("/sonar_img/elev_fov")

sonar_params = max_range, min_range, range_res, num_beams, \
				num_rows, horizontal_fov , elev_aperature

bridge = CvBridge()

def sonar_callback(image_msg, pc_msg):
	'''
	Synced image and pc, record sonar image, 3D elevation data
	Todo: add front-view ground truth info later?
	'''
	raw_img = raw2cvimg(image_msg)
	time = headertime2str(image_msg.header)
	cv2.imwrite(experiment_dir + str(time) + '_raw_sonar.png', raw_img)

	elev_map = pc2elevmap(pc_msg, NUM_ELEV_CLASSES, sonar_params)
	write_elev_map(elev_map, raw_img, experiment_dir, time, NUM_ELEV_CLASSES, also_visualize=True)



def main():
		rospy.init_node('sonar_image_writer')      

		image_sub = message_filters.Subscriber(raw_image_topic, SonarImage)
		pc_sub = message_filters.Subscriber(pc_topic, PointCloud2)

		ts = message_filters.TimeSynchronizer([image_sub, pc_sub], 10)
		ts.registerCallback(sonar_callback)

		rospy.spin()

if __name__ == '__main__':
		main()