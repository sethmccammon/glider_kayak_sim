#!/usr/bin/env python

import rospy, random
from glider_kayak_sim.msg import STU, UnderwaterGeoPose, UnderwaterGeoPoint
from glider_kayak_sim.srv import SimQuery

class KayakSensor(object):
  """docstring for KayakSensor"""
  def __init__(self):
    self.sensor_pub = rospy.Publisher(rospy.get_param('/robot_sensor_topic'), STU, queue_size=1)
    self.pose_pub = rospy.Publisher(rospy.get_param('/robot_pose_topic'), UnderwaterGeoPose, queue_size=1)
    self.ground_truth_sub = rospy.Subscriber('ground_truth_pose', UnderwaterGeoPose, self.groundTruthCallback)
    self.position = None
    self.orientation = None

  def groundTruthCallback(self, data):
    self.position = [data.position.latitude, data.position.longitude, data.position.depth]
    self.orientation = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

def STUClient(position):
  topic = rospy.get_param("/world_sim_topic")
  rospy.wait_for_service(topic)
  try:
    sim_query = rospy.ServiceProxy(topic, SimQuery)
    query = UnderwaterGeoPoint()
    query.latitude = position[0]
    query.longitude = position[1]
    query.depth = position[2]
    resp = sim_query(query)
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
    return None

def main():
  rospy.init_node("kayak_sensor")
  kayak_sensor_rate = rospy.get_param('/kayak_publish_rate')
  sensor = KayakSensor()


  rate = rospy.Rate(1./kayak_sensor_rate) # 10hz
  while not rospy.is_shutdown():
    if sensor.position is not None:
      
      resp = STUClient(sensor.position)
      if resp.stu.temperature > -100.: #I doubt that the kayaks can function at -100 C
        msg = STU()
        sensor.sensor_pub.publish(resp.stu)

      msg = UnderwaterGeoPose()
      msg.position.latitude = sensor.position[0]+random.random()*.00002 - .00004
      msg.position.longitude = sensor.position[1]+random.random()*.00002 - .00004
      msg.position.depth = sensor.position[2]

      msg.orientation.x = sensor.orientation[0]
      msg.orientation.y = sensor.orientation[1]
      msg.orientation.z = sensor.orientation[2]
      msg.orientation.w = sensor.orientation[3]
      sensor.pose_pub.publish(msg)

    rate.sleep()




if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
