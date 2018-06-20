#!/usr/bin/env python

import rospy, random

from glider_kayak_sim.msg import UnderwaterGeoPoint

def main():
  rospy.init_node("waypoint_chatter")
  
  #Simulation Service Definitions
  wpt_pub = rospy.Publisher(rospy.get_param('/robot_waypoint_topic'), UnderwaterGeoPoint, queue_size=1)

  while not rospy.has_param('/sim_w_bound'):
    pass

  n_bound = rospy.get_param('/sim_n_bound')
  s_bound = rospy.get_param('/sim_s_bound')
  e_bound = rospy.get_param('/sim_e_bound')
  w_bound = rospy.get_param('/sim_w_bound')


  rate = rospy.Rate(1./rospy.get_param('/waypoint_publish_rate'))
  while not rospy.is_shutdown():
    pt = UnderwaterGeoPoint()
    pt.latitude = s_bound + random.random()*(n_bound - s_bound)
    pt.longitude = w_bound + random.random()*(e_bound - w_bound)
    pt.depth = 0.0
    wpt_pub.publish(pt)
    rate.sleep()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
