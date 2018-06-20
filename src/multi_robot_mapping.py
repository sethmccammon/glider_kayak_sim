#!/usr/bin/env python

import rospy, GPy, rospkg, itertools, time

import netCDF4 as nc
import numpy as np
import matplotlib.pyplot as plt
from glider_kayak_sim.msg import STU, UnderwaterGeoPose




class ObservationSTU(object):
  """docstring for ObservationSTU"""
  def __init__(self, lat, lon, temp, salinity, current_u, current_v):
    self.lat = lat
    self.lon = lon
    self.salinity = salinity
    self.temperature = temp
    self.current_v = current_v
    self.current_u = current_u




class MRSMapper(object):
  """docstring for MRSMapper"""
  def __init__(self):
    num_kayaks = rospy.get_param('num_kayaks')

    sensor_topic = rospy.get_param('robot_sensor_topic')
    pose_topic = rospy.get_param('robot_pose_topic')
    kayak_namespace = rospy.get_param('kayak_namespace')

    sensor_subs = [rospy.Subscriber((kayak_namespace + sensor_topic) % idx, STU, self.sensorCallback, (idx)) for idx in range(num_kayaks)]
    pose_subs = [rospy.Subscriber((kayak_namespace + pose_topic) % idx, UnderwaterGeoPose, self.poseCallback, (idx)) for idx in range(num_kayaks)]

    self.observations = []
    self.positions = [None for idx in range(num_kayaks)]

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('glider_kayak_sim')
    self.dataset = nc.Dataset(package_path + "/data/ca300m_das_2017100103.nc")


    self.surface_temperature = self.dataset['temp'][0,0,:,:]
    
    self.lat = self.dataset['lat'][:]
    self.lon = np.array([x if x < 180. else x-360 for x in self.dataset['lon'][:]])

    self.n_bound = np.max(self.lat)
    self.s_bound = np.min(self.lat)
    self.e_bound = np.max(self.lon)
    self.w_bound = np.min(self.lon)
 

  def sensorCallback(self, data, robot_idx):
    # print 'Sensor:', arg
    if self.positions[robot_idx] is not None:
      obs = ObservationSTU(self.positions[robot_idx][0], self.positions[robot_idx][1], data.temperature, data.salinity, 0.0, 0.0)
      self.observations.append(obs)


  def poseCallback(self, data, robot_idx):
    # print 'Pose:', arg
    self.positions[robot_idx] = [data.position.latitude, data.position.longitude]

  def trainGP(self, num_Z=10):
    X = [(obs.lat, obs.lon) for obs in self.observations]
    Y = [obs.temperature for obs in self.observations]

    print "Input:", np.min(Y), np.max(Y)

    X = np.array(X).reshape((len(X), 2))
    Y = np.array(Y).reshape((len(X), 1))
    k = GPy.kern.RBF(input_dim = 2, variance = .1, lengthscale = .01, ARD=True)
    m = GPy.models.SparseGPRegression(X,Y,k,num_inducing=num_Z)
    m.Z.unconstrain()
    m.optimize('bfgs')

    query = np.array(list(itertools.product(self.lat, self.lon)))

    zz = m.predict(np.array(query))
    prediction_mean = np.array(zz[0]).reshape((len(self.lat), len(self.lon)))

    res = np.ma.array(prediction_mean, mask=self.surface_temperature.mask, fill_value=-9999)
    res = res.filled()
    return res



def writeNetCDF(roms_data, gp_data):
  rospack = rospkg.RosPack()
  package_path = rospack.get_path('glider_kayak_sim')
  rootgrp = nc.Dataset(package_path + "/data/ros_data_out.nc", 'r+')

  # nLats = len(roms_data.dimensions["lat"])
  # nLons = len(roms_data.dimensions["lon"])
  # nDepths = len(roms_data.dimensions['depth'])

  # depth = rootgrp.createDimension("depth", nDepths)
  # time = rootgrp.createDimension("time", None)
  # lat = rootgrp.createDimension("lat", nLats)
  # lon = rootgrp.createDimension("lon", nLons)

  # depths = rootgrp.createVariable('depth','f4',('depth',))
  # depth.units = 'meters'
  # depth.long_name = 'depth'

  # latitudes = rootgrp.createVariable("lat","f4",("lat",))
  # latitudes.units = "degrees_north"
  # latitudes.long_name = "Latitude"

  # longitudes = rootgrp.createVariable("lon","f4",("lon",))
  # longitudes.units = 'degrees_east'
  # longitudes.long_name = 'Longitude'

  # temp = rootgrp.createVariable("temp","f4",("time", "depth", "lat", "lon"))
  # temp.long_name = 'Temperature'
  # temp.units = 'degrees C'
  # temp.missing_value = -9999



  # latitudes[:] = roms_data['lat'][:]
  # longitudes[:] = roms_data['lon'][:]
  # depth[:] = roms_data['depth'][:]

  surface_temp = np.ones((1, 24, 216, 326))*-9999
  surface_temp[0,0,:,:] = gp_data

  rootgrp['temp'][:,:,:,:] = surface_temp

  rootgrp.close()


def main():
  rospy.init_node("kayak_sensor")
  kayak_sensor_rate = rospy.get_param('/kayak_publish_rate')
  mrs_mapper = MRSMapper()
  num_Z = 20
  rate = rospy.Rate(1) # Spin Every Minute
  while not rospy.is_shutdown():
    if len(mrs_mapper.observations) > num_Z:
      print "Training GP..."
      start_time = time.time()
      gp_bel = mrs_mapper.trainGP(num_Z)
      writeNetCDF(mrs_mapper.dataset, gp_bel)
      # print "Output:", np.min(gp_bel), np.max(gp_bel)

      # print "Elapsed:", time.time() - start_time
      # print len(mrs_mapper.observations)
      # fig = plt.figure(1)
      # plt.clf()
      # plt.scatter([x.lon for x in mrs_mapper.observations], [x.lat for x in mrs_mapper.observations])
      # img = plt.imshow(gp_bel, vmin=11., vmax=17., origin='lower', extent=[mrs_mapper.w_bound, mrs_mapper.e_bound, mrs_mapper.s_bound, mrs_mapper.n_bound])
      # cbar = fig.colorbar(img)
      # plt.draw()
      # plt.show(block=False)

    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
