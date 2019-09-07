#!/usr/bin/env python

import cv2
import numpy as np
import struct
from math import isnan
from collections import Counter
import copy

class PointDetector:
	
	flavor_color_map = {
		"Chocolate": np.array([[11,0,0],[35,180,255]]),
		"Flakes": np.array([[38,40,0],[66,255,255]])
		}

	rgb_cam = cv2.VideoCapture(2)

	@staticmethod
	def _segment(orig, K=4, ret_labels=True):
		nrow, ncol, nchannel = orig.shape

		X_coords = np.array([[i for i in range(nrow)] for _ in range(ncol)])
		Y_coords = np.array([[i for _ in range(nrow)] for i in range(ncol)])

		coords = np.stack((X_coords.T, Y_coords.T),axis=-1)

		hsv = cv2.cvtColor(orig, cv2.COLOR_BGR2HSV)
		new_hsv = np.concatenate((0.1*np.sqrt(nrow*ncol)*hsv,coords), axis=2)
		features = np.float32(np.reshape(new_hsv,(-1,new_hsv.shape[2])))

		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
		
		K=4
		ret,label,center=cv2.kmeans(features,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
		
		if ret_labels:
			return np.reshape(label, (nrow, ncol))
		
		img = cv2.applyColorMap(np.uint8((255/K)*label), cv2.COLORMAP_JET)
		img = np.reshape(img, (nrow,ncol,nchannel))
		return img

	@staticmethod
	def _getMask(image, low_color, high_color):
		frame=cv2.GaussianBlur(image,(5,5),0)
	
		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		mask = cv2.inRange(hsv, low_color, high_color)

		mask = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=10)
		return cv2.erode(mask, np.ones((5,5),np.uint8), iterations=2)
	
	@staticmethod
	def _pixel2pc(mask, point_cloud):
		
		points = list()
		pc = point_cloud
		print(mask)
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

	@staticmethod
	def _pixcnt2pc(mask, point_cloud):

		cimg = np.zeros_like(mask)
		_, contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) > 0:
			return PointDetector._pixel2pc(cimg, point_cloud)
		else:
			return np.array(list())
	
	@staticmethod
	def detect( royale_depth, royale_pc, flavor):

		if not flavor in PointDetector.flavor_color_map.keys():
			return None

		color_range = PointDetector.flavor_color_map[flavor]
		ret_val, rgb_img = PointDetector.rgb_cam.read()
		
		while not ret_val:
			ret_val, rgb_img = PointDetector.rgb_cam.read()
		
		mask = PointDetector._getMask(rgb_img, color_range[0], color_range[1])
		
		if mask is None:
			return None

		cv2.imshow('Mask', mask)
		if cv2.waitKey(1) == 27:
			return

		dim_x, dim_y = royale_depth.shape
		# Transform rgb view to match depth cam view
		pts1 = np.float32([[516,228],[503,367],[862,425],[903,301]])
		pts2 = np.float32([[85,59],[83,96],[187,114],[200,79]])
		M = cv2.getPerspectiveTransform(pts1, pts2)

		warped_rgb = cv2.warpPerspective(rgb_img, M, (dim_y, dim_x))
		warped_mask = cv2.warpPerspective(mask, M, (dim_y, dim_x))

		labels = PointDetector._segment(warped_rgb, ret_labels=True)
		centers = labels[warped_mask == 255].tolist()

		if len(centers) == 0:
			return None
		
		c = Counter(centers)
		centroid = c.most_common(1)[0][0]
		new_mask = np.uint8(255*(labels == centroid))

		cv2.imshow('New Mask', new_mask)
		if cv2.waitKey(1) == 27:
			return

		return PointDetector._pixcnt2pc(new_mask, royale_pc)
			


