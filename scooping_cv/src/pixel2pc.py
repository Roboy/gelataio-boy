#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from color_filter import ColorFilter
import struct
from math import isnan

# This script listens /zed/zed_node/depth/depth_registered
# and calculates the average distance of the closest objects.
# Then, publishes average distance to shy_roboy/nearest_distance.

class ImageConverter:
	def __init__(self):
		# Initialize depth image listener and average distance publisher.
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback_depth)
		self.rgb_sub = rospy.Subscriber("/zed/zed_node/rgb/image_raw_color", Image, self.callback_rgb)
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
	
	def pixel2pc(self, mask, rgb_img=None):
		
		points = list()
		pc = self.pointcloud
		#print(len(pc.data))
		if rgb_img is None:
			rgb_img = self.cv_image_rgb.copy()
		for (u,v), masked in np.ndenumerate(mask):
			
			if masked:
				#print("+++")
				index = u*pc.row_step + v*pc.point_step
				#print(index)
				index_x = index + pc.fields[0].offset
				#print(index_x)
				index_y = index + pc.fields[1].offset
				#print(index_y)
				index_z = index + pc.fields[2].offset
				#print(index_z)
				#print("---")
				"""
				X = pc.data[index_x:index_x+4]
				X = float(struct.unpack("<f", X)[0])
				#print(X)
				Y = pc.data[index_y:index_y+4]
				Y = float(struct.unpack("<f", Y)[0])
				#print(Y)
				Z = pc.data[index_z:index_z+4]
				Z = float(struct.unpack("<f", Z)[0])
				#print(Z)
				#print((X,Y,Z))
				"""
				XYZ = list(struct.unpack("3f", pc.data[index_x:index_z+4]))
				if not any(map(isnan, XYZ)):
					XYZ.extend(list(rgb_img[u,v,:]))
					points.append(XYZ)

		#print(str(pc.data[index_x:index_z+4]))
		return points


def main():
	ic = ImageConverter()
	filt = ColorFilter("L")
	rospy.init_node('image_converter', anonymous=True)
	rospy.loginfo("Node running.")

	# 30 FPS = 60 Hz
	rate = rospy.Rate(60)

	try:
		while not rospy.is_shutdown():

			if ic.callback_received:
				#cv2.imshow('filtered_img', filt.filter(ic.cv_image_rgb, ret_mask=True))
				rgb_img= ic.cv_image_rgb.copy()
				mask = filt.filter(ic.cv_image_rgb, ret_mask=True)
				XYZRGB = np.asarray(ic.pixel2pc(mask,rgb_img=rgb_img))
				print(XYZRGB.shape)
				np.save("xyzrgb.npy", XYZRGB)
				
				#print(ic.pixel2pc(mask))
				break
				cv2.waitKey(1)

			rate.sleep()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
