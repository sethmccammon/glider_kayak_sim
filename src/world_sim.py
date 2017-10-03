#!/usr/bin/env python

import rospy, rospkg

from glider_kayak_sim.msg import STU
from glider_kayak_sim.srv import SimQuery
from geometry_msgs.msg import Vector3





class WorldSim(object):
  """docstring for WorldSim"""
  def __init__(self):
    pass


  def simQueryHandle(self, req):
    res = STU()
    res.temperature = 0.0
    res.salinity = 0.0
    res.current = Vector3(0.0, 0.0, 0.0)

    return SimQueryResposne(res)













def main():
  rospy.init_node("world_sim")


  rospack = rospkg.RosPack()
  package_path = rospack.get_path('glider_kayak_sim')

  sim_datafile = rospy.get_param("simulation_data_file")

  print "Reading data from:", package_path+sim_datafile
  
  simulator = WorldSim()

  #Simulation Service Definitions
  rospy.Service('SimQuery', SimQuery, simulator.simQueryHandle)


  rate = rospy.Rate(10) # 10hz spin
  while not rospy.is_shutdown():
    rate.sleep()
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
