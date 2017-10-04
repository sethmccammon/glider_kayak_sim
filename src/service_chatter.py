#!/usr/bin/env python

import rospy

from glider_kayak_sim.srv import SimQuery
from geographic_msgs.msg import GeoPoint

def main():
  rospy.init_node("service_chatter")
  
  #Simulation Service Definitions
  sim_query = rospy.ServiceProxy('SimQuery', SimQuery)


  rate = rospy.Rate(10) # 10hz spin
  while not rospy.is_shutdown():
    pt = GeoPoint()
    resp = sim_query(pt)
    print resp
  rospy.spin()





if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
