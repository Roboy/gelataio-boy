#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from color_filter import ColorFilter

# This script listens /zed/zed_node/depth/depth_registered
# and calculates the average distance of the closest objects.
# Then, publishes average distance to shy_roboy/nearest_distance.

class ImageConverter:
	def __init__(self):
		# Initialize depth image listener and average distance publisher.
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback_depth)
		self.rgb_sub = rospy.Subscriber("/zed/zed_node/rgb_raw/image_raw_color", Image, self.callback_rgb)
		self.pc_sub = rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
		self.depth_image = []
		self.cv_image_rgb = []
		self.pointcloud = PointCloud2()
		self.callback_received = False

	def callback_depth(self, data):
		try:
			# Read depth image.
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
			self.callback_received = True
		except CvBridgeError as e:
			print(e)

	def callback_rgb(self, data):
		try:
			# Read rgb image.
			self.cv_image_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.callback_received = True
		except CvBridgeError as e:
			print(e)

	def callback_pc(self, data):
		
		self.pointcloud = data


	def get_depth(self):
		height, width = self.depth_image.shape

		h_new = height * 0.3
		w_new = width * 0.3

		# Show images.
		cv2.imshow('depth image', self.depth_image)
		cv2.waitKey(1)


	def get_rgb(self):
		
		# Show images.
		cv2.imshow('depth image', self.cv_image_rgb)
		cv2.waitKey(1)

    def get_rgb(self):
        
        # Show images.
        cv2.imshow('depth image', self.cv_image_rgb)
        cv2.waitKey(1)

def main():
<<<<<<< HEAD
    ic = ImageConverter()
	filt = ColorFilter("C")
    rospy.init_node('image_converter', anonymous=True)
    rospy.loginfo("Node running.")

    # 30 FPS = 60 Hz
    rate = rospy.Rate(60)

    try:
        while not rospy.is_shutdown():

            if ic.callback_received:
				cv2.imshow('filtered_img', filt.filter(ic.cv_image_rgb))
				cv2.waitKey(1)

            rate.sleep()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
=======
	ic = ImageConverter()
	filt = ColorFilter("C")
	rospy.init_node('image_converter', anonymous=True)
	rospy.loginfo("Node running.")
	FIRST_IMG=True
	# 30 FPS = 60 Hz
	rate = rospy.Rate(60)

	try:
		while not rospy.is_shutdown():

			if ic.callback_received:
				if FIRST_IMG:
					np.save("rgbimg.npy", ic.cv_image_rgb)
					FIRST_IMG = False
				if len(ic.cv_image_rgb) is not 0:
					cv2.imshow('filtered_img', filt.filter(ic.cv_image_rgb))
				cv2.waitKey(1)

			rate.sleep()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()
>>>>>>> master

if __name__ == '__main__':
	main()
