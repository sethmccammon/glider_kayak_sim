#!/usr/bin/env python
import rospy
import tf
import math
import random
from math import *
from glider_kayak_sim.srv import *
from glider_kayak_sim.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

class KayakState(object):
	latitude	= 0.00
	longitude	= 0.00
	yaw			= 0.00
	def __init__(
					self,
					latitude=0.0,
					longitude=0.0,
					yaw=0.0,
				):
		self.latitude	= latitude
		self.longitude	= longitude
		self.yaw		= yaw

class Kayak(object):

	def __init__(
					self,
					max_turn,
					max_abs_vel,
					wpt_tol,
					control_const,
					world_sim_topic,
					geo_pose_topic,
					verbose=False,
					wpts=[],
					latitude=0.0,
					longitude=0.0,
				):

		# Model parameters
		self.max_turn			= max_turn
		self.max_abs_vel		= max_abs_vel
		self.wpt_tol			= wpt_tol
		self.control_const		= control_const
		self.world_sim_topic	= world_sim_topic
		self.geo_pose_topic		= geo_pose_topic

		# Internal variables
		self.verbose			= verbose
		self.abs_vel			= 0.00
		self.goal_idx			= 0
		self.wpts				= wpts
		self.stu				= STU()
		self.state				= KayakState(latitude=latitude, longitude=longitude)
		self.action				= KayakState()

		# Ros-related
		self.geo_pose_pub		= rospy.Publisher(geo_pose_topic, GeoPose, queue_size=10)
		self.sim_query			= rospy.ServiceProxy(world_sim_topic, SimQuery)

	def __str__(self):
		ret_str = ""
		ret_str += "Kayak Status:"
		ret_str += " State: (%06.2f,%06.2f,%06.2f)" 	% (self.state.latitude, self.state.longitude, self.state.yaw)
		ret_str += " Action: (%06.2f,%06.2f,%06.2f)"	% (self.action.latitude, self.action.longitude, self.action.yaw)
		ret_str += " Temp: %06.2f Sal: %06.2f"			% (self.stu.temperature, self.stu.salinity)
		ret_str += " Current: (%06.2f,%06.2f,%06.2f)"	% (self.stu.current.x, self.stu.current.y, self.stu.current.z)
		if len(self.wpts) > 0:
			ret_str += " Goal[%d]: (%06.2f,%06.2f)"		% (self.goal_idx, self.wpts[self.goal_idx].position.longitude, self.wpts[self.goal_idx].position.latitude)
		return ret_str

	def get_geo_pose(self):
		orientation				= tf.transformations.quaternion_from_euler(0.0, 0.0, self.state.yaw)
		pose					= GeoPose()
		pose.position.latitude	= self.state.latitude
		pose.position.longitude	= self.state.longitude
		pose.position.altitude	= 0.0
		pose.orientation.x		= orientation[0]
		pose.orientation.y		= orientation[1]
		pose.orientation.z		= orientation[2]
		pose.orientation.w		= orientation[3]
		return pose

	def update(self):

		if len(self.wpts) > 0:

			# Setting constant velocity
			self.abs_vel = self.max_abs_vel

			# Next wpt
			goal_wpt	= self.wpts[self.goal_idx]

			# Rectangular coordinates to next wpt
			goal_dx		= goal_wpt.position.longitude	- self.state.longitude
			goal_dy		= goal_wpt.position.latitude	- self.state.latitude

			# Polar coordinates to next wpt
			goal_dist	= sqrt(goal_dx**2 + goal_dy**2)
			goal_angle	= atan2(goal_dy,goal_dx)

			# Checking if goal achieved
			if goal_dist < self.wpt_tol:
				self.goal_idx += 1
				if self.goal_idx >= len(self.wpts):
					self.goal_idx = 0

			# Yaw error
			yaw_error = goal_angle - self.state.yaw
			if yaw_error > math.pi:
				yaw_error -= 2*math.pi
			if yaw_error < -math.pi:
				yaw_error += 2*math.pi

			# Control effort
			self.action.yaw		= self.max_turn*atan(self.control_const*yaw_error)/(0.5*math.pi)
		
		else:
			self.abs_vel		= 0.00

		# Updating linear velocity based on yaw and absolute velocity
		self.action.longitude	= self.abs_vel*cos(self.state.yaw)
		self.action.latitude	= self.abs_vel*sin(self.state.yaw)

		# Updating state (integrating actions)
		self.state.yaw			+= self.action.yaw
		self.state.longitude	+= self.action.longitude
		self.state.latitude		+= self.action.latitude
		if self.state.yaw > math.pi:
			self.state.yaw -= 2*math.pi
		if self.state.yaw < -math.pi:
			self.state.yaw += 2*math.pi

		# GeoPose Message
		geo_pose = self.get_geo_pose()

		# Sensor reading 
		try:
			self.stu = self.sim_query(geo_pose.position).stu
		except rospy.ServiceException, e:
			rospy.loginfo("Service Call Failed: %s" % e)

		# Publishing pose
		self.geo_pose_pub.publish(geo_pose)

		# Printing kayak status
		if self.verbose:
			rospy.loginfo(self)

def main():
	
	rospy.init_node("kayak_sim")

	# Global parameters
	sim_update_rate		= rospy.get_param("/sim_update_rate")
	world_sim_topic		= rospy.get_param("/world_sim_topic")
	geo_pose_topic		= rospy.get_param("/geo_pose_topic")

	# Global Kayak parameters
	max_turn			= rospy.get_param("/kayak_max_turn",		0.10)
	max_abs_vel			= rospy.get_param("/kayak_max_abs_vel",		0.01)
	wpt_tol				= rospy.get_param("/kayak_wpt_tol",			0.01)
	control_const		= rospy.get_param("/kayak_control_const",	4.00)
	verbose				= rospy.get_param("/kayak_verbose",			False)

	# Goal Waypoints
	rnd_loc = 1+random.random();
	wpts_list = [
					(0.0, rnd_loc),
					(rnd_loc, rnd_loc),
					(rnd_loc, 0.0),
					(0.0, 0.0),
				]
	wpts = []
	for wpt in wpts_list:
		w = WayPoint()
		w.position.longitude	= wpt[0]
		w.position.latitude		= wpt[1]
		w.position.altitude		= 0.0
		wpts.append(w)

	# Init Kayak
	kayak = Kayak(
					max_turn		= max_turn,
					max_abs_vel		= max_abs_vel,
					wpt_tol			= wpt_tol,
					control_const	= control_const,
					verbose			= verbose,
					world_sim_topic	= world_sim_topic,
					geo_pose_topic	= geo_pose_topic,
					wpts			=wpts,
				)

	# Running sim
	rate = rospy.Rate(sim_update_rate) 
	while not rospy.is_shutdown():
		kayak.update()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
