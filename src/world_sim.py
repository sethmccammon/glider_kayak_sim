#!/usr/bin/env python

import rospy, rospkg, pdb

import netCDF4 as nc

from glider_kayak_sim.msg import STU
from glider_kayak_sim.srv import SimQuery
from geometry_msgs.msg import Vector3

class WorldSim(object):
  """docstring for WorldSim"""
  def __init__(self, datafile_path):
    #Read Data in from NETCDF File 
    #We need to decide on a ROMS model to pull data from and we can tune these variables to it
    dataset = nc.Dataset(datafile_path)
    current_u = dataset['u'][:]
    current_v = dataset['v'][:]
    temperature = dataset['temp'][:]
    salinity = dataset['salt'][:]

    print salinity.shape



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
  sim_datafile  = rospy.get_param("simulation_data_file")
  world_sim_topic = rospy.get_param("world_sim_topic")


  datafile_path = package_path + sim_datafile


  rospy.loginfo("Reading data from: %s" % (datafile_path))

  simulator = WorldSim(datafile_path)

  #Simulation Service Definitions
  rospy.Service(world_sim_topic, SimQuery, simulator.simQueryHandle)

  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
