#!/usr/bin/env python

import rospy
import rospkg

from glider_kayak_sim.msg import STU
from glider_kayak_sim.srv import SimQuery
from geometry_msgs.msg import Vector3

class WorldSim(object):

	def __init__(self):
		pass

	def simQueryHandle(self, req):
		res = STU()
		res.temperature = 0.0
		res.salinity = 0.0
		res.current = Vector3(0.0, 0.0, 0.0)
		return res

def main():

	rospy.init_node("world_sim")

	rospack = rospkg.RosPack()

	package_path = rospack.get_path('glider_kayak_sim')

	sim_datafile	= rospy.get_param("simulation_data_file")
	world_sim_topic	= rospy.get_param("world_sim_topic")

	rospy.loginfo("Reading data from: %s%s" % (package_path, sim_datafile))

	simulator = WorldSim()

	#Simulation Service Definitions
	rospy.Service(world_sim_topic, SimQuery, simulator.simQueryHandle)

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
