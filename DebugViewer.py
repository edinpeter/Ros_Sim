import sys
import cv2
import numpy as np
import math

VIEW_RANGE = 500
WINDOW_SIZE = 1100
CROPPED_WINDOW_SIZE = 800
blank_viewport = np.zeros((WINDOW_SIZE, WINDOW_SIZE, 4), np.uint8)


class DebugViewer:
	#field of view in degrees
	fov = None
	fov_rad = None
	world_item_dictionary = {}

	def __init__(self,entities, fov):
		self.entities_ref = entities
		self.robot = self.entities_ref[0]
		self.create_viewer_image_dictionary()
		self.fov = fov
		self.fov_rad = fov * np.pi / 180.0
		cv2.namedWindow('viewport', cv2.WINDOW_NORMAL)

	def update_view_window(self):
		objects_in_view = self.get_objects_to_draw(self.entities_ref)
		self.draw_viewport(objects_in_view)

	def draw_info(self, viewport):
		cv2.circle(viewport, (WINDOW_SIZE / 2, 0), 6, (255,0,255),-1)
		cv2.circle(viewport, (WINDOW_SIZE / 2, WINDOW_SIZE), 6, (255,0,255),-1)

	def draw_viewport(self, objects):
		blank_viewport[:] = (255,255,255, 0)
		
		objects.sort(self.distance_sort)
		
		for obj in objects:
			angle = self.find_angle_from_bot_to_obj(self.robot, obj)
			distance = self.distance(self.robot, obj)

			obj_perp_planar_distance = np.cos(angle) * distance
			obj_parallel_planar_length = int(2.0 * np.tan(self.fov_rad / 2.0) * obj_perp_planar_distance)
			obj_draw_proportion = obj.width / (obj_parallel_planar_length * 1.0)

			ratio = WINDOW_SIZE / (obj_parallel_planar_length * 1.0)
			obj_draw_x_pos = (obj_parallel_planar_length * ratio / 2 - obj_perp_planar_distance * ratio * np.tan(angle))
			obj_draw_y_pos = WINDOW_SIZE / 2 #change later to make it 3d

			if obj_draw_proportion < 2:

				self.draw_entity(blank_viewport, obj, obj_draw_x_pos, obj_draw_y_pos, obj_draw_proportion)
		cropped_viewport = np.zeros((CROPPED_WINDOW_SIZE,CROPPED_WINDOW_SIZE,4), np.uint8)
		cropped_viewport[:] = (255,255,255, 0)
		cropped_viewport[0:CROPPED_WINDOW_SIZE, 0: CROPPED_WINDOW_SIZE] = blank_viewport[(WINDOW_SIZE - CROPPED_WINDOW_SIZE) / 2 : WINDOW_SIZE - ((WINDOW_SIZE - CROPPED_WINDOW_SIZE) / 2), (WINDOW_SIZE - CROPPED_WINDOW_SIZE) / 2 : WINDOW_SIZE - ((WINDOW_SIZE - CROPPED_WINDOW_SIZE) / 2)]
		cv2.imshow('viewport',cropped_viewport)
		cv2.waitKey(1)#millisec

	def draw_entity(self, image, obj, obj_draw_x_pos, obj_draw_y_pos, obj_draw_proportion):
		img_width = image.shape[1]

		objDrawSizeX = img_width * obj_draw_proportion

		obj_resized = cv2.resize(self.world_item_dictionary[obj.name], None, fx=obj_draw_proportion, fy=obj_draw_proportion, interpolation = cv2.INTER_LINEAR)
		obj_x_start_loc, obj_x_end_loc, obj_image_x_start_loc, obj_image_x_end_loc, obj_y_dim = self.fix_draw_bounds(image, obj_draw_x_pos, obj_resized)

		lower_y = WINDOW_SIZE / 2 - obj_y_dim / 2.0
		upper_y = WINDOW_SIZE / 2 + obj_y_dim / 2.0

		obj_img_ys = 0

		upper_y,lower_y, obj_img_ys, obj_y_dim, obj_x_end_loc, obj_x_start_loc, obj_image_x_start_loc, obj_image_x_end_loc = self.fix_bounds(upper_y, lower_y, obj_img_ys, obj_y_dim, obj_x_end_loc, obj_x_start_loc, obj_image_x_start_loc, obj_image_x_end_loc)
		try:
			image[lower_y : upper_y, obj_x_start_loc: obj_x_end_loc] = obj_resized[obj_img_ys : obj_y_dim, obj_image_x_start_loc: obj_image_x_end_loc]
		except:
			pass
		
	def fix_bounds(self, upper_y, lower_y, map_y_start, map_y_end, upper_x, lower_x, map_x_start, map_x_end):
		while True:
			if upper_y - lower_y > map_y_end - map_y_start:
				if lower_y is WINDOW_SIZE:
					upper_y -=1
				else:
					lower_y += 1
			elif upper_y - lower_y < map_y_end - map_y_start:
				if lower_y is 0:
					upper_y += 1
				else:
					lower_y -= 1

			elif upper_x - lower_x > map_x_end - map_x_start:
				if lower_x is WINDOW_SIZE:
					upper_x -= 1
				else:
					lower_x += 1

			elif upper_x - lower_x < map_x_end - map_x_start:
				if lower_x is 0:
					upper_x += 1
				else:
					lower_x -= 1
			else:
				return upper_y, lower_y, map_y_start, map_y_end, upper_x, lower_x, map_x_start, map_x_end

	def fix_draw_bounds(self, canvas, obj_draw_x_pos, objResized):
		obj_x_dim= objResized.shape[1]
		canvas_width = int(canvas.shape[1])

		obj_x_start_loc= obj_draw_x_pos - obj_x_dim/ 2.0
		obj_x_end_loc= obj_draw_x_pos + obj_x_dim/ 2.0
		obj_y_dim = objResized.shape[0]
		obj_image_x_start_loc = 0
		obj_image_x_end_loc = obj_x_dim

		if obj_x_dim/ 2.0 + obj_draw_x_pos > canvas_width:
			obj_x_start_loc= canvas.shape[1] - obj_draw_x_pos - obj_x_dim/ 2.0
			obj_x_end_loc= canvas.shape[1]
			obj_image_x_end_loc = obj_x_end_loc- obj_x_start_loc
		if obj_draw_x_pos - obj_x_dim/ 2.0 < 0:
			obj_x_start_loc= 0
			obj_x_end_loc= obj_draw_x_pos + obj_x_dim/ 2.0
			obj_image_x_start_loc = obj_x_dim- obj_x_end_loc
			obj_image_x_end_loc = obj_x_dim
		obj_y_dim = objResized.shape[0]
		return int(obj_x_start_loc), int(obj_x_end_loc), int(obj_image_x_start_loc), int(obj_image_x_end_loc), int(obj_y_dim)


	def find_angle_from_bot_to_obj(self, robot, ent):
		dX = ent.x_pos - robot.x_pos
		dY = robot.y_pos - ent.y_pos
		#entAngle = np.arctan2(dY, dX) + np.pi
		entAngle = math.atan2(dY,dX)
		if entAngle < 0:
			entAngle += 2.0 * np.pi
		return entAngle - robot.orientation
		

	def get_objects_to_draw(self, entities):
		drawing_list = []
		for ent in entities[1:]:
			if(self.distance(self.robot, ent) < VIEW_RANGE and np.absolute(self.find_angle_from_bot_to_obj(self.robot, ent)) < self.fov_rad / 2.0):
				if ent.floor_object is 0:
					drawing_list.append(ent)
		return drawing_list

	def distance(self, ent1, ent2):
		return np.sqrt(np.square(ent1.x_pos - ent2.x_pos) + np.square(ent1.y_pos - ent2.y_pos))
	
	def distance_sort(self, a, b):
		if self.distance(b, self.robot) > self.distance(a, self.robot):
			return 1
		elif self.distance(b, self.robot) <= self.distance(a, self.robot):
			return 0
		else:
			return -1
	def create_viewer_image_dictionary(self):

		redb_map = cv2.imread("worlditems/red_buoy.png", -1)
		greenb_map = cv2.imread("worlditems/green_buoy.png", -1)
		yellowb_map = cv2.imread("worlditems/yellow_buoy.png", -1)
		gate_map = cv2.imread("worlditems/gate.png", -1)

		self.world_item_dictionary['redb'] = redb_map
		self.world_item_dictionary['greenb'] = greenb_map
		self.world_item_dictionary['yellowb'] = yellowb_map
		self.world_item_dictionary['gate'] = gate_map
