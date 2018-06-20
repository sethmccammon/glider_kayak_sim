#!/usr/bin/env python

import rospy, random

from vehicles import SlocumGlider




def main():
  rospy.init_node("slocum_glider")
  rospy.wait_for_service('SimQuery')

  n_bound = rospy.get_param('sim_n_bound')
  s_bound = rospy.get_param('sim_s_bound')
  e_bound = rospy.get_param('sim_e_bound')
  w_bound = rospy.get_param('sim_w_bound')
  
  start_lat = 36.804731# random.uniform(n_bound, s_bound)
  start_lon = -121.946976# random.uniform(e_bound, w_bound)
  start_depth = 0.0
  start_pitch = 0.0
  start_heading = random.uniform(0, 360)

  kayak = SlocumGlider(start_lat, start_lon, start_depth, start_pitch, start_heading)

  rate = rospy.Rate(10) # 10hz spin
  while not rospy.is_shutdown():
    kayak.update()
    print kayak.position
    rate.sleep()
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass