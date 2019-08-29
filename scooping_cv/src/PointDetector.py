#!/usr/bin/env python
import cv2
import numpy as np
import struct
from math import isnan
from collections import Counter

def segment(orig, K=4, ret_labels=True):
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

class PointDetector:
	
	def __init__(self):
		
		self.set_colors("Chocolate")
		pts1 = np.float32([[516,228],[503,367],[862,425],[903,301]])
		pts2 = np.float32([[85,59],[83,96],[187,114],[200,79]])

		self.M = cv2.getPerspectiveTransform(pts1, pts2)

	def set_colors(self, flavor):
		if flavor == "Chocolate":
			self.LOW_COLOR = np.array([11,0,0])
			self.HI_COLOR = np.array([35,180,255])
		else:
			### Implement new colors
			self.LOW_COLOR = np.array([11,0,0])
			self.HI_COLOR = np.array([35,180,255])
	
	def color_filter(image, ret_mask=True):
		frame=cv2.GaussianBlur(image,(5,5),0)
	
		hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		mask = cv2.inRange(hsv,self.LOW_COLOR, self.HI_COLOR)

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
	
	def pixel2pc(self, mask, point_cloud):
		
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

	def pixcnt2pc(self, mask, point_cloud):

		cimg = np.zeros_like(mask)
		contours, hieararchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		if len(contours) > 0:

			c = max(contours, key=cv2.contourArea)
			cv2.drawContours(cimg,[c],0,color=255,thickness=-1)
			box = np.int0(cv2.boxPoints(cv2.minAreaRect(c)))
			return self.pixel2pc(cimg, point_cloud)
		else:
			return np.array(list())

	def detect(zed_rgb, royale_depth, royale_pc, flavor):

		# Set correct color as LOW and HIGH
		self.set_colors(flavor)

		mask = self.color_filter(zed_rgb, ret_mask=True)
		dim_x, dim_y = royale_depth.shape
		# Transform rgb view to match depth cam view
		warped_rgb = cv2.warpPerspective(zed_rgb, M, (dim_y, dim_x))
		warped_mask = cv2.warpPerspective(mask, M, (dim_y, dim_x))

		labels = segment(warped_rgb,ret_labels=True)
		centers = labels[warped_mask == 255].tolist()
		centroid = 0
		if len(centers) > 0:
			c = Counter(centers)
			centroid = c.most_common(1)[0][0]
			new_mask = np.uint8(255*(labels == centroid))
			pts = ic.pixcnt2pc(new_mask, royale_pc)
			return {success: True, message: pts}
		else:
			return {success: False, message: "No pixel survived filtering"}




