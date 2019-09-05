#!/usr/bin/env python

import cv2
import numpy as np
import struct
from math import isnan
from collections import Counter

import ros_numpy
from ros_numpy import *

class PointDetector:
	
	flavor_color_map = {
		"Chocolate": np.array([[11,0,0],[35,180,255]]),
		"Flakes": np.array([[38,30,0],[66,255,255]])
		}

	rgb_cam = cv2.VideoCapture(2)

	@staticmethod
	def _color_filter(image, low_color, high_color):
		frame = cv2.GaussianBlur(image,(5,5), 0)

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, low_color, high_color)

		cv2.imshow('Ice Cream Mask', mask)
		if cv2.waitKey(1) == 27:
			return
		mask = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=10)
		mask = cv2.erode(mask, np.ones((5,5),np.uint8), iterations=5)

		_, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		if len(contours) > 0:

			c = max(contours, key=cv2.contourArea)
			box = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
			
			cv2.drawContours(image, [box], 0, (255), thickness=7)			
			cv2.imshow('Ice Cream Monitor Box', image)
			if cv2.waitKey(1) == 27:
				return

			return mask, box
		else:
			return []
	
	@staticmethod
	def _crop_pc(box, array):
		points = []
		#for point in array:



		return np.array(points)
	
	@staticmethod
	def detect(royale_depth, royale_pc, flavor):
		if not flavor in PointDetector.flavor_color_map.keys():
			return None

		ret_val, rgb_img = PointDetector.rgb_cam.read()
		
		while not ret_val:
			ret_val, rgb_img = PointDetector.rgb_cam.read()

		color_range = PointDetector.flavor_color_map[flavor]
		mask, box = PointDetector._color_filter(rgb_img, color_range[0], color_range[1])

		if royale_depth is None:
			return

		array = ros_numpy.point_cloud2.pointcloud2_to_array(royale_pc)
		
		return PointDetector._crop_pc(box, array)