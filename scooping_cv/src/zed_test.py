#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import struct
from math import isnan
#from color_filter import ColorFilter
#LOW_COLOR = np.array([4, 68, 0])
#HI_COLOR = np.array([14, 160, 255])
#LOW_COLOR = np.array([13,0,0])
#HI_COLOR = np.array([24,110,255])
#LOW_COLOR = np.array([110,30,80])
#HI_COLOR = np.array([126,109,255])
LOW_COLOR = np.array([101,25,0])
HI_COLOR = np.array([115,83,255])
 


# This script listens /zed/zed_node/depth/depth_registered
# and calculates the average distance of the closest objects.
# Then, publishes average distance to shy_roboy/nearest_distance.

def filter(image, ret_mask=False):
	frame=cv2.GaussianBlur(image,(5,5),0)
 
	hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	mask = cv2.inRange(hsv,LOW_COLOR, HI_COLOR)
	if ret_mask:
		return mask
	mask = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=2)
	mask = cv2.erode(mask, np.ones((5,5),np.uint8), iterations=2)

	contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	if len(contours) > 0:

		c = max(contours, key=cv2.contourArea)
		box = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
		return box

	else:
		return []

class ImageConverter:
	def __init__(self):
		# Initialize depth image listener and average distance publisher.
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.callback_depth)
		self.rgb_sub = rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, self.callback_rgb)
		self.pc_sub = rospy.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_pc)
		self.royale_sub = rospy.Subscriber("/royale_camera_driver/depth_image", Image, self.callback_royale)
		self.royale_pc_sub = rospy.Subscriber("/royale_camera_driver/point_cloud", PointCloud2, self.callback_royale_pc)
		self.royale_depth = []
		self.royale_pc = PointCloud2()
		self.depth_image = []
		self.cv_image_rgb = []
		self.pointcloud = PointCloud2()
		self.callback_received = False

	def callback_royale(self, data):
		try:
			# Read depth image.
			self.royale_depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
		except CvBridgeError as e:
			print(e)
	
	def callback_royale_pc(self, data):
		
		self.royale_pc = data

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

	def pixel2pc(self, mask):
		
		points = list()
		pc = self.royale_pc
		
		for (u,v), masked in np.ndenumerate(mask):
			
			if masked:
				index = u*pc.row_step + v*pc.point_step
				index_x = index + pc.fields[0].offset
				index_y = index + pc.fields[1].offset
				index_z = index + pc.fields[2].offset

				XYZ = list(struct.unpack("3f", pc.data[index_x:index_z+4]))
				if not any(map(isnan, XYZ)):
					points.append(XYZ)

		return np.array(points)

	def pixcnt2pc(self, mask):

		cimg = np.zeros_like(mask)
		contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) > 0:

			c = max(contours, key=cv2.contourArea)
			cv2.drawContours(cimg,[c],0,color=255,thickness=-1)

			return self.pixel2pc(cimg), cimg
		else:
			return np.array(list()), None



def main():
	ic = ImageConverter()
	#filt = ColorFilter("C")
	rospy.init_node('image_converter', anonymous=True)
	rospy.loginfo("Node running.")
	FIRST_IMG=True
	# 30 FPS = 60 Hz
	rate = rospy.Rate(60)
	pts1 = np.float32([[516,228],[503,367],[862,425],[903,301]])
	pts2 = np.float32([[85,59],[83,96],[187,114],[200,79]])

	M = cv2.getPerspectiveTransform(pts1, pts2)
	
	XYZ = list()
	try:
		while not rospy.is_shutdown():

			if ic.callback_received:
				if len(ic.cv_image_rgb) > 0:
					#cv2.imshow("Camera Data", ic.depth_image)
					copy_img = ic.cv_image_rgb.copy()
					box = filter(ic.cv_image_rgb)
					mask = filter(ic.cv_image_rgb, ret_mask=True)
					
					if len(box) > 0:
						cv2.drawContours(copy_img,[box],0,(60,255,255), 3)
					if ic.royale_depth != []:
						cv2.imshow("Royale", ic.royale_depth)
						dim_x, dim_y = ic.royale_depth.shape
						warped_rgb = cv2.warpPerspective(copy_img, M, (dim_y, dim_x))
						warped_mask = cv2.warpPerspective(mask, M, (dim_y, dim_x))
						
						cv2.imshow("Warped", warped_rgb)
						XYZ, cimg = ic.pixcnt2pc(warped_mask)
						if cimg is not None:
							cv2.imshow("Contour", cimg)
					#cv2.imshow("Mask", mask)
					cv2.imshow("Filtered",copy_img)
					#cv2.imshow("Mask",mask)
				else:
					print("No Data")
				#cv2.waitKey(1)
				k = cv2.waitKey(33)
				if k == 32:
					#cv2.imwrite("depth.jpg", 255*cv2.cvtColor(ic.royale_depth, cv2.COLOR_GRAY2RGB))
					#cv2.imwrite("orig.jpg", ic.cv_image_rgb)
					np.save("cnt_points.npy", XYZ)

			rate.sleep()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
