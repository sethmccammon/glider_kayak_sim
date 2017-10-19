#!/usr/bin/env python

import rospy, rospkg





class AquaticVehicle(object):
  """docstring for AquaticVehicle"""
  def __init__(self, arg):
    super(AquaticVehicle, self).__init__()
    self.arg = arg
    
  def getSensorData():
    pass

  def step():
    pass


class SlocumGlider(AquaticVehicle):
  """docstring for AquaticVehicle"""
  def __init__(self, arg):
    super(SlocumGlider, self).__init__()
    self.arg = arg
    
























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
