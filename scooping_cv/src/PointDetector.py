#!/usr/bin/env python

import cv2
import numpy as np
import struct
from math import isnan
from collections import Counter

class PointDetector:
	
	flavor_color_map = {
		"chocolate": np.array([[70,25,0],[98,255,255]]),
		"flakes": np.array([[38,40,0],[66,255,255]]),
		"vanilla": np.array([[70,25,0],[98,255,255]]),
		"strawberry": np.array([[70,25,0],[98,255,255]])
		}

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
	def _color_filter(image, low_color, high_color, ret_mask=True):
		frame=cv2.GaussianBlur(image,(5,5),0)
	
		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		mask = cv2.inRange(hsv, low_color, high_color)

		mask = cv2.dilate(mask, np.ones((5,5),np.uint8), iterations=2)
		mask = cv2.erode(mask, np.ones((5,5),np.uint8), iterations=2)

		if ret_mask:
			return mask

		contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) > 0:

			c = max(contours, key=cv2.contourArea)
			box = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
			return box

		else:
			return []
	
	@staticmethod
	def _pixel2pc(mask, point_cloud):
		
		points = list()
		pc = point_cloud
		
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
		contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) > 0:

			c = max(contours, key=cv2.contourArea)
			cv2.drawContours(cimg,[c],0,color=255,thickness=-1)
			box = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
			return PointDetector._pixel2pc(cimg, point_cloud)
		else:
			return np.array(list())
	
	@staticmethod
	def detect(zed_rgb, royale_depth, royale_pc, flavor):

		if not flavor in PointDetector.flavor_color_map.keys():
			return None

		color_range = PointDetector.flavor_color_map[flavor]

		mask = PointDetector._color_filter(zed_rgb, color_range[0], color_range[1], ret_mask=True)
		dim_x, dim_y = royale_depth.shape
		# Transform rgb view to match depth cam view
		pts1 = np.float32([[210,171],[240,274],[468,235],[408,136]])
		pts2 = np.float32([[66,85],[76,126],[155,111],[133,73]])
		M = cv2.getPerspectiveTransform(pts1, pts2)

		warped_rgb = cv2.warpPerspective(zed_rgb, M, (dim_y, dim_x))
		warped_mask = cv2.warpPerspective(mask, M, (dim_y, dim_x))
		"""
		labels = PointDetector._segment(warped_rgb, ret_labels=True)
		centers = labels[warped_mask == 255].tolist()

		if len(centers) == 0:
			return None
		
		centroid = 0
		c = Counter(centers)
		centroid = c.most_common(1)[0][0]
		new_mask = np.uint8(255*(labels == centroid))
		"""
		
		return PointDetector._pixcnt2pc(warped_mask, royale_pc)
			


