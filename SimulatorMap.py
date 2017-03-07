#import rospy
import sys
import cv2
import numpy as np
import time
import thread
import math

WINDOW_SIZE = 800
gate_arr = None
Y_ADJUST_RATIO = 9.0 / 12.0

class SimulatorMap:
	robot = None
	entities_ref = None
	map_icon_dictionary = {}
	robot_default_image = None
	fov = None
	entity_image_map_offset_x = None
	entity_image_map_offset_y = None

	def __init__(self,entities, fov):
		self.entities_ref = entities
		self.robot = self.entities_ref[0]
		self.fov = fov / 2.0 * np.pi / 180.0
		self.create_map_icon_dictionary()
		self.draw_map()
		cv2.namedWindow('map', cv2.WINDOW_NORMAL)

	def draw_info(self, simmap, rx_pos, ry_pos, x_pos, y_pos, ent):
		cv2.line(simmap, (int(rx_pos), int(ry_pos)), (int(x_pos), int(y_pos)), (255,0,0))
		#cv2.putText(simmap, str(self.findAngleFromBotToObj(self.robot, ent)), (int(x_pos),int(y_pos)), 1, 1, (255,0,0))
		cv2.putText(simmap, str(self.distance(self.robot, ent)), (int(x_pos), int(y_pos)), 1, 1, (0, 255, 0))


	def draw_map(self):
		blank_map = np.zeros((WINDOW_SIZE,WINDOW_SIZE,4), np.uint8)
		blank_map[:] = (255,255,255, 0)
		entity_map_image_offset_x = self.robot.x_pos - WINDOW_SIZE / 2
		entity_map_image_adjust_y = self.robot.y_pos - Y_ADJUST_RATIO * WINDOW_SIZE
		self.entitityImageAdjustX = entity_map_image_offset_x
		self.entitityImageAdjustY = entity_map_image_adjust_y
		#print entity_map_image_adjust_y
		for ent in self.entities_ref:
			self.draw_entity(blank_map, ent, ent.x_pos - entity_map_image_offset_x, ent.y_pos - entity_map_image_adjust_y)
			self.draw_info(blank_map, self.robot.x_pos - entity_map_image_offset_x, self.robot.y_pos - entity_map_image_adjust_y, ent.x_pos - entity_map_image_offset_x, ent.y_pos - entity_map_image_adjust_y, ent)
		self.draw_entity(blank_map, self.robot, self.robot.x_pos - entity_map_image_offset_x, self.robot.y_pos - entity_map_image_adjust_y)
		self.draw_fov(blank_map)
		cv2.imshow('map',blank_map)
		cv2.waitKey(1)#millisec

	def fixDrawBounds(self, lowerX, upperX, lowerY, upperY, mapXs, mapXe, mapYs, mapYe, draw):
		if upperX > WINDOW_SIZE and lowerX > WINDOW_SIZE:
			return None, None, None, None, None, None, None, None, 0
		if upperY > WINDOW_SIZE and lowerY > WINDOW_SIZE:
			return None, None, None, None, None, None, None, None, 0
		if upperX < 0 and lowerX < 0:
			return None, None, None, None, None, None, None, None, 0
		if upperY < 0 and lowerY < 0:
			return None, None, None, None, None, None, None, None, 0
		if upperX > WINDOW_SIZE:
			upperX = WINDOW_SIZE
			mapXe = upperX - lowerX
		if upperY > WINDOW_SIZE:
			upperY = WINDOW_SIZE
			mapYe = upperY - lowerY
		if lowerX < 0:
			mapXs = -lowerX
			lowerX = 0
		if lowerY < 0:
			mapYs = -lowerY
			lowerY = 0
		draw = 1
		return lowerX, upperX, lowerY, upperY, mapXs, mapXe, mapYs, mapYe, draw
	def cleanIcon(self, icon):
		#gray = cv2.cvtColor(icon, cv2.COLOR_BGR2GRAY)
		#ret, thresh = cv2.threshold(gray, 0, 0, cv2.THRESH_BINARY_INV)
		#return cv2.bitwise_and(icon, icon, mask = thresh)
		return icon

	def draw_entity(self,blank_map, ent, x_pos, y_pos):
		mapIcon = self.map_icon_dictionary[ent.name]
		draw = 1
		mapXs = 0
		mapYs = 0
		mapXe = mapIcon.shape[1]
		mapYe = mapIcon.shape[0]
		lowerY = y_pos - mapIcon.shape[0] / 2 
		upperY = y_pos - mapIcon.shape[0] / 2 + mapIcon.shape[0]
		lowerX = x_pos - mapIcon.shape[1] / 2 
		upperX = x_pos - mapIcon.shape[1] / 2 + mapIcon.shape[1]
		lowerX, upperX, lowerY, upperY, mapXs, mapXe, mapYs, mapYe, draw = self.fixDrawBounds(lowerX, upperX, lowerY, upperY, mapXs, mapXe, mapYs, mapYe, draw)
		if draw is 1:
			upperY,lowerY,mapYs,mapYe,upperX,lowerX,mapXs,mapXe = self.fixBounds(upperY,lowerY,mapYs,mapYe,upperX,lowerX,mapXs,mapXe)
			blank_map[int(lowerY) : int(upperY), int(lowerX) : int(upperX)] = mapIcon[mapYs : mapYe , mapXs : mapXe]

		#time.sleep(.1)
	def fixBounds(self, upperY, lowerY, mapYs, mapYe, upperX, lowerX, mapXs, mapXe):
		while True:
			if upperY - lowerY > mapYe - mapYs:
				if lowerY is WINDOW_SIZE:
					upperY -=1
				else:
					lowerY += 1
			elif upperY - lowerY < mapYe - mapYs:
				if lowerY is 0:
					upperY += 1
				else:
					lowerY -= 1

			elif upperX - lowerX > mapXe - mapXs:
				if lowerX is WINDOW_SIZE:
					upperX -= 1
				else:
					lowerX += 1

			elif upperX - lowerX < mapXe - mapXs:
				if lowerX is 0:
					upperX += 1
				else:
					lowerX -= 1
			else:
				return upperY,lowerY,mapYs,mapYe,upperX,lowerX,mapXs,mapXe


	def handle_rotations(self):
		for key in self.map_icon_dictionary:
			for ent in self.entities_ref:
				if ent.name is key:
					if ent.orientation is not 0:
						imgWidth = self.map_icon_dictionary[ent.name].shape[1]
						imgHeight = self.map_icon_dictionary[ent.name].shape[0]
						diag = np.sqrt(imgWidth * imgWidth + imgHeight * imgHeight)
						diag = int(diag)
						empty_image = np.zeros((diag,diag,4), np.uint8)
						empty_image[:] = (255,255,255, 0)
						empty_image[diag / 2.0 - imgHeight / 2.0 : diag / 2.0 + imgHeight / 2.0, diag / 2.0 - imgWidth / 2.0 : diag / 2.0 + imgWidth / 2.0] = self.map_icon_dictionary[ent.name]

						M = cv2.getRotationMatrix2D( (diag/2,diag/2), int(ent.orientation * 180.0 / np.pi) ,1)
						dst = cv2.warpAffine(empty_image,M,(diag,diag), 0, 0, 1, (255,255,255, 0))
						self.map_icon_dictionary[ent.name] = self.cleanIcon(dst)

	def handle_robot_rotation(self):
		map_image_width = self.robot_default_image.shape[1]
		map_image_height = self.robot_default_image.shape[0]
		diag = np.sqrt(map_image_width * map_image_width + map_image_height * map_image_height)
		diag = int(diag)
		empty_image = np.zeros((diag,diag,4), np.uint8)
		empty_image[:] = (255,255,255, 0)
		empty_image[diag / 2.0 - map_image_height / 2.0 : diag / 2.0 + map_image_height / 2.0, diag / 2.0 - map_image_width / 2.0 : diag / 2.0 + map_image_width / 2.0] = self.robot_default_image
		ent = self.entities_ref[0]
		rotation_matrix = cv2.getRotationMatrix2D( (diag/2,diag/2), int(ent.orientation * 180.0 / np.pi) ,1)

		rotated_robot = cv2.warpAffine(empty_image, rotation_matrix, (diag,diag), 0, 0, 1, (255,255,255, 127))
		self.map_icon_dictionary[self.entities_ref[0].name] = self.cleanIcon(rotated_robot)
	
	def draw_fov(self, simmap):
		x_fov_origin = WINDOW_SIZE / 2
		y_fov_origin = Y_ADJUST_RATIO * WINDOW_SIZE
		origin = int(x_fov_origin), int(y_fov_origin)
		length = 500

		left_edge_h = int(length * np.sin(self.entities_ref[0].orientation + self.fov))
		left_edge_w = int(length * np.cos(self.entities_ref[0].orientation + self.fov))
		left_edge_pt = left_edge_w + origin[0], origin[1] - left_edge_h

		right_edge_h = int(length * np.sin(self.entities_ref[0].orientation - self.fov))
		right_edge_w = int(length * np.cos(self.entities_ref[0].orientation - self.fov))
		right_edge_pt = right_edge_w + origin[0], origin[1] - right_edge_h

		cv2.line(simmap, origin, left_edge_pt, (255,0,0))
		cv2.line(simmap, origin, right_edge_pt, (255,0,0))

	def find_angle_from_bot_to_obj(self, robot, ent):
		dX = ent.x_pos - robot.x_pos
		dY = robot.y_pos - ent.y_pos
		#entAngle = np.arctan2(dY, dX) + np.pi
		entAngle = math.atan2(dY,dX)
		if entAngle < 0:
			entAngle += 2.0 * np.pi
		return entAngle - robot.orientation
	def distance(self, ent1, ent2):
		return np.sqrt(np.square(ent1.x_pos - ent2.x_pos) + np.square(ent1.y_pos - ent2.y_pos))
	def create_map_icon_dictionary(self):

		redb_map = cv2.imread("mapicons/redb_map.png", -1)
		greenb_map = cv2.imread("mapicons/greenb_map.png", -1)
		yellowb_map = cv2.imread("mapicons/yellowb_map.png", -1)
		gate_map = cv2.imread("mapicons/gate_map.png", -1)
		robot_map = cv2.imread("mapicons/robot_map.png", -1)
		ofm_map = cv2.imread("mapicons/ofm_map.png", -1)
		self.robot_default_image = robot_map

		self.map_icon_dictionary['redb'] = redb_map
		self.map_icon_dictionary['greenb'] = greenb_map
		self.map_icon_dictionary['yellowb'] = yellowb_map
		self.map_icon_dictionary['gate'] = gate_map
		self.map_icon_dictionary['robot'] = self.cleanIcon(robot_map)
		self.map_icon_dictionary['ofm'] = ofm_map
		self.handle_rotations()

