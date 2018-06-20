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
					sim_speedup_factor,
					sim_update_rate,
					world_sim_topic,
					underwater_geo_pose_topic,
					verbose=False,
					wpts=[],
					latitude=0.0,
					longitude=0.0,
				):

		# Model parameters
		self.max_turn					= max_turn
		self.max_abs_vel				= max_abs_vel
		self.wpt_tol					= wpt_tol
		self.control_const				= control_const
		self.sim_speedup_factor			= sim_speedup_factor
		self.sim_update_rate			= sim_update_rate
		self.world_sim_topic			= world_sim_topic
		self.underwater_geo_pose_topic	= underwater_geo_pose_topic

		# Internal variables
		self.verbose					= verbose
		self.abs_vel					= 0.00
		self.goal_idx					= 0
		self.wpts						= wpts
		self.stu						= STU()
		self.state						= KayakState(latitude=latitude, longitude=longitude)
		self.action						= KayakState()

		# Ros-related
		self.underwater_geo_pose_pub	= rospy.Publisher(underwater_geo_pose_topic, UnderwaterGeoPose, queue_size=10)
		self.sim_query					= rospy.ServiceProxy(world_sim_topic, SimQuery)

	def __str__(self):
		ret_str = ""
		ret_str += "Kayak Status:"
		ret_str += " State: (%06.2f,%06.2f,%06.2f)" 	% (self.state.latitude, self.state.longitude, self.state.yaw)
		ret_str += " Action: (%06.2f,%06.2f,%06.2f)"	% (self.action.latitude, self.action.longitude, self.action.yaw)
		ret_str += " Temp: %06.2f Sal: %06.2f"			% (self.stu.temperature, self.stu.salinity)
		ret_str += " Current: (%06.2f,%06.2f,%06.2f)"	% (self.stu.current.x, self.stu.current.y, self.stu.current.z)
		if len(self.wpts) > 0:
			ret_str += " Goal[%d]: (%06.2f,%06.2f)"		% (self.goal_idx, self.wpts[self.goal_idx].position.latitude, self.wpts[self.goal_idx].position.longitude)
		return ret_str

	def get_underwater_geo_pose(self):
		orientation				= tf.transformations.quaternion_from_euler(0.0, 0.0, self.state.yaw)
		pose					= UnderwaterGeoPose()
		pose.position.latitude	= self.state.latitude
		pose.position.longitude	= self.state.longitude
		pose.position.depth		= 0.0
		pose.orientation.x		= orientation[0]
		pose.orientation.y		= orientation[1]
		pose.orientation.z		= orientation[2]
		pose.orientation.w		= orientation[3]
		return pose

	def update(self):

		increment_rate = float(self.sim_speedup_factor)/self.sim_update_rate

		if len(self.wpts) > 0:

			# Setting constant velocity
			self.abs_vel = self.max_abs_vel

			# Next wpt
			goal_wpt	= self.wpts[self.goal_idx]

			# Rectangular coordinates to next wpt
			goal_dx		= goal_wpt.position.longitude	- self.state.longitude
			goal_dy		= goal_wpt.position.latitude	- self.state.latitude

			# print 'self.state.longitude: %9.3f goal_wpt.position.longitude: %9.3f goal_dx: %9.3f' %(self.state.longitude, goal_wpt.position.longitude, goal_dx)

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
		self.state.yaw			+= self.action.yaw*increment_rate
		self.state.longitude	+= self.action.longitude*increment_rate
		self.state.latitude		+= self.action.latitude*increment_rate
		if self.state.yaw > math.pi:
			self.state.yaw -= 2*math.pi
		if self.state.yaw < -math.pi:
			self.state.yaw += 2*math.pi

		# UnderwaterGeoPose Message
		underwater_geo_pose = self.get_underwater_geo_pose()

		# Sensor reading 
		try:
			self.stu = self.sim_query(underwater_geo_pose.position).stu
		except rospy.ServiceException, e:
			pass
			# rospy.loginfo("kayak_sim: Service Call Failed: %s" % e)

		# Publishing pose
		self.underwater_geo_pose_pub.publish(underwater_geo_pose)

		# Printing kayak status
		if self.verbose:
			rospy.loginfo('kayak_sim %s' % self.__str__())

def main():
	
	rospy.init_node("kayak_sim")

	# Global parameters
	sim_speedup_factor				= rospy.get_param("/sim_speedup_factor")
	sim_update_rate					= rospy.get_param("/sim_update_rate")
	world_sim_topic					= rospy.get_param("/world_sim_topic")
	underwater_geo_pose_topic		= rospy.get_param("/underwater_geo_pose_topic")

	# Global Kayak parameters
	max_turn						= rospy.get_param("/kayak_max_turn",		0.10)
	max_abs_vel						= rospy.get_param("/kayak_max_abs_vel",		0.01)
	wpt_tol							= rospy.get_param("/kayak_wpt_tol",			0.01)
	control_const					= rospy.get_param("/kayak_control_const",	4.00)
	verbose							= rospy.get_param("/kayak_verbose",			False)

	# Goal Waypoints
	rnd_loc = random.random();
	wpts_list = [
					(-122.550,36.465),
					(-122.530,36.870),
					(-122.200,36.500),
					(-122.050,36.870),
					(-122.900,36.700),
					(-122.570,36.720),
				]
	wpts = []
	for wpt in random.sample(wpts_list,4):
		w = WayPoint()
		w.position.longitude	= wpt[0]
		w.position.latitude		= wpt[1]
		w.position.altitude		= 0.0
		wpts.append(w)

	# Init Kayak
	kayak = Kayak(
					max_turn					= max_turn,
					max_abs_vel					= max_abs_vel,
					wpt_tol						= wpt_tol,
					control_const				= control_const,
					verbose						= verbose,
					sim_speedup_factor			= sim_speedup_factor,
					sim_update_rate				= sim_update_rate,
					world_sim_topic				= world_sim_topic,
					underwater_geo_pose_topic	= underwater_geo_pose_topic,
					wpts						= wpts,
					longitude					= -121.7,
					latitude					= 36.4,
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
