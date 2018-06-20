#!/usr/bin/env python

import rospy, rospkg, Queue, math, tf



from haversine import haversine
from glider_kayak_sim.msg import UnderwaterGeoPose, UnderwaterGeoPoint, STU
from glider_kayak_sim.srv import SimQuery
from utils import euclideanDist
import numpy as np



class Vehicle(object):
  """docstring for Vehicle"""
  def __init__(self, lat=0.0, lon=0.0, depth=0.0, roll=0.0, pitch=0.0, heading=0.0):
    self.position = np.array([lat, lon, depth])
    self.orientation = np.array([roll, pitch, heading])
    self.waypoints = Queue.Queue()
    self.vel = 0.0 #m/s
    self.prev_update_time = rospy.get_time()
    self.current_goal = None
    self.verbose = rospy.get_param("/kayak_verbose", False)
    self.pose_publisher = rospy.Publisher("ground_truth_pose", UnderwaterGeoPose, queue_size=1)
    #self.sensor_publisher = rospy.Publisher('sensor_stu', STU, queue_size=1)
    self.sim_query = rospy.ServiceProxy(rospy.get_param('/world_sim_topic'), SimQuery)
    self.waypoint_tolerance = 0.1 #Meters
    #World Boundaries
    self.n_bound = rospy.get_param('/sim_n_bound')
    self.s_bound = rospy.get_param('/sim_s_bound')
    self.e_bound = rospy.get_param('/sim_e_bound')
    self.w_bound = rospy.get_param('/sim_w_bound')
    self.status = 'idle' # Vel = 0
    self.waypoint_sub = rospy.Subscriber('wpt_cmd', UnderwaterGeoPoint, self.wptCallback)


  def wptCallback(self, data):
    self.addWaypoint(data.latitude, data.longitude)

  def getUnderwaterGeoPose(self):
    orientation = tf.transformations.quaternion_from_euler(self.orientation[0], self.orientation[1], self.orientation[2])
    pose = UnderwaterGeoPose()
    pose.position.latitude  = self.position[0]
    pose.position.longitude = self.position[1]
    pose.position.depth = self.position[2]

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

  def getWorldState(self):
    resp = self.sim_query(self.getUnderwaterGeoPose().position)
    temp = resp.stu.temperature
    salinity = resp.stu.salinity
    current_vector = np.array(resp.stu.current)

    return temp, salinity, current_vector

  def isInBounds(self, lat, lon):
    if lat > self.n_bound or lat < self.s_bound:
      return False
    elif lon < self.w_bound or lon > self.e_bound:
      return False
    else:
      return True

  def addWaypoint(self, lat, lon):
    pt = np.array([lat, lon])
    if self.isInBounds(lat, lon):
      #print "addWaypoint"
      self.waypoints.put(pt)
    else:
      raise ValueError




class ROSSKayak(Vehicle):
  def __init__(self, lat=0.0, lon=0.0, heading=0.0):
    Vehicle.__init__(self, lat, lon, 0.0, 0.0, 0.0, heading)
    self.max_vel = rospy.get_param("/kayak_max_vel")
    self.waypoint_tolerance = rospy.get_param("/kayak_waypoint_tolerance")


  def update(self):
    update_time = rospy.get_time()
    temp, sal, current_vec = self.getWorldState()

    

    if not self.waypoints.empty() and self.current_goal is None:
      self.current_goal = self.waypoints.get()
      goal_vec = self.current_goal - self.position[:2]
      self.orientation[2] = math.atan2(goal_vec[0], goal_vec[1])

    if self.current_goal is not None:
      self.vel = 0.01
      goal_vec = self.current_goal - self.position[:2]
      self.orientation[2] = math.atan2(goal_vec[0], goal_vec[1])
      heading = self.orientation[2]
      dt = update_time - self.prev_update_time

      dx = np.array([math.sin(heading), math.cos(heading), 0])*dt*self.vel

      self.position = self.position + dx
      #if haversine(self.current_goal, self.position[:2])*1000. < self.waypoint_tolerance:
      if euclideanDist(self.current_goal, self.position[:2]) < self.waypoint_tolerance:
        #print "Waypoint Achieved!"
        if self.waypoints.empty():
          self.current_goal = None
          self.vel = 0.0
        else:
          self.current_goal = self.waypoints.get()

    msg = self.getUnderwaterGeoPose()
    self.pose_publisher.publish(msg)

    self.prev_update_time = update_time




class SlocumGlider(Vehicle):
  def __init__(self, lat=0.0, lon=0.0, depth=0.0, pitch=0.0, heading=0.0):
    Vehicle.__init__(self, lat, lon, depth, 0.0, pitch, heading)
    self.max_vel = rospy.get_param("/glider_max_vel")
    self.max_depth = rospy.get_param("/glider_max_depth")
    self.min_depth = rospy.get_param("/glider_min_depth")
    self.dive_angle = rospy.get_param("/glider_dive_angle")
    self.orientation[1] =  self.dive_angle*math.pi/180.


  def update(self):
    update_time = rospy.get_time()
    temp, sal, current_vec = self.getWorldState()


    #Maintain Depth Envelope
    if self.position[2] < self.min_depth and self.orientation[1] > 0:
      self.orientation[1] = -self.orientation[1]
    elif self.position[2] > self.max_depth and self.orientation[1] < 0:
      self.orientation[1] = -self.orientation[1]
    
    if not self.waypoints.empty() and self.current_goal is None:
      self.current_goal = self.waypoints.get()
      goal_vec_2d = self.current_goal - self.position[:2]
      self.orientation[2] = math.atan2(goal_vec_2d[0], goal_vec_2d[1])

    if self.current_goal is not None:
      self.vel = 0.01
      goal_vec_2d = self.current_goal - self.position[:2]
      self.orientation[2] = math.atan2(goal_vec_2d[0], goal_vec_2d[1])
      heading = self.orientation[2]
      pitch = self.orientation[1]
      dt = update_time - self.prev_update_time

      dx = np.array([math.sin(heading)*math.sin(pitch), math.cos(heading)*math.sin(pitch), math.cos(pitch)])*dt*self.vel

      self.position = self.position + dx
      if haversine(self.current_goal, self.position[:2])*1000. < self.waypoint_tolerance:
        print "Waypoint Achieved!"
        if self.waypoints.empty():
          self.current_goal = None
          self.vel = 0.0
        else:
          self.current_goal = self.waypoints.get()


      msg = self.getUnderwaterGeoPose()
      self.pose_publisher.publish(msg)

      self.prev_update_time = update_time
    else:
      msg = self.getUnderwaterGeoPose()
      self.pose_publisher.publish(msg)

      self.prev_update_time = update_time


if __name__ == '__main__':
  main()
