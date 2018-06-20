#!/usr/bin/env python

import rospy, random

import matplotlib.pyplot as plt

from vehicles import ROSSKayak




def main():
  rospy.init_node("ross_kayak")
  rospy.wait_for_service('/SimQuery')

  n_bound = rospy.get_param('/sim_n_bound')
  s_bound = rospy.get_param('/sim_s_bound')
  e_bound = rospy.get_param('/sim_e_bound')
  w_bound = rospy.get_param('/sim_w_bound')
  
  start_lat = 36.804731# random.uniform(n_bound, s_bound)
  start_lon = -121.946976# random.uniform(e_bound, w_bound)
  start_heading = random.uniform(0, 360)

  print "Simulating ROSS Kayak"
  kayak = ROSSKayak(start_lat, start_lon, start_heading)
  rate = rospy.Rate(10) # 10hz spin
  while not rospy.is_shutdown(): 
    kayak.update()
    rate.sleep()

  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
