#!/usr/bin/env python

import rospy, rospkg, haversine






def main():
  rospy.init_node("slocum_glider")



  rate = rospy.Rate(10) # 10hz spin
  while not rospy.is_shutdown():
    rate.sleep()
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
