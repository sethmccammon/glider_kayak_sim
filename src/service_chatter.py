#!/usr/bin/env python

import rospy

from glider_kayak_sim.srv import SimQuery
from glider_kayak_sim.msg import UnderwaterGeoPoint

def main():
  rospy.init_node("service_chatter")
  
  #Simulation Service Definitions
  rospy.wait_for_service('SimQuery')
  sim_query = rospy.ServiceProxy('SimQuery', SimQuery)


  rate = rospy.Rate(1) # 10hz spin
  while not rospy.is_shutdown():
    pt = UnderwaterGeoPoint()
    pt.latitude = 36.804731
    pt.longitude = -121.946976
    pt.depth = 3.0
    resp = sim_query(pt)
    print resp
  rospy.spin()





if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
