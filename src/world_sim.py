#!/usr/bin/env python

import rospy, rospkg, pdb, datetime, time
import numpy as np
import netCDF4 as nc
import matplotlib.pyplot as plt
from scipy.interpolate import RegularGridInterpolator
from glider_kayak_sim.msg import STU, Array3D
from glider_kayak_sim.srv import SimQuery, SimSnapshot
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock

from get_ROMS import getROMS, getBathymetry

from utils import getTimestamp


class ROMSSnapshot(object):
  def __init__(self, idx, roms_dates, roms_files):
    self.idx = idx
    self.date = roms_dates[idx]
    dataset = nc.Dataset(roms_files[idx])

    #Science Data
    self.current_u = dataset['u'][0,:,:,:]
    self.current_v = dataset['v'][0,:,:,:]
    self.temperature = dataset['temp'][0,:,:,:]
    self.salinity = dataset['salt'][0,:,:,:]
    
    #Axes
    self.lat = dataset['lat'][:]
    self.lon = np.array([x if x < 180. else x-360 for x in dataset['lon'][:]])
    self.depth = dataset['depth'][:]


class WorldSim(object):
  # World Simulator
  # Contains a Clock Server
  def __init__(self):
    print "Loading ROMS Data....."
    start_date = rospy.get_param('start_date') 
    start_date = datetime.datetime.strptime(start_date, '%Y/%m/%d')

    end_date = rospy.get_param('end_date') 
    end_date = datetime.datetime.strptime(end_date, '%Y/%m/%d')



    #roms_files is an ordered list of ROMS NETCDF files containing simulated data
    #The actual files containing data get located in the /data folder

    # self.roms_data = {}
    self.roms_dates, self.roms_files = getROMS(start_date, end_date)
    self.past_roms = ROMSSnapshot(0, self.roms_dates, self.roms_files)
    self.future_roms = ROMSSnapshot(1, self.roms_dates, self.roms_files)


    dataset = nc.Dataset(self.roms_files[0])
    self.roms_lat = dataset['lat'][:]
    self.roms_lon = np.array([x if x < 180. else x-360 for x in dataset['lon'][:]])
    self.depth = dataset['depth'][:]
    print self.depth

    print len(self.roms_lat)
    print len(self.roms_lon)
    print len(self.depth)

    roms_n_bound = np.max(self.roms_lat)
    roms_s_bound = np.min(self.roms_lat)
    roms_e_bound = np.max(self.roms_lon)
    roms_w_bound = np.min(self.roms_lon)
    self.max_depth = np.max(self.depth)

    # for dt, file_path in zip(self.roms_dates, roms_files):
    #   self.roms_data[dt] = nc.Dataset(file_path)

    print "Loading ROMS Data Complete"

    print "Loading Bathymetry Data......"
    bathymetry_file = getBathymetry(roms_n_bound, roms_s_bound, roms_e_bound, roms_w_bound)
    dataset = nc.Dataset(bathymetry_file)
    self.bathymetry_lat = dataset['lat']
    self.bathymetry_lon = dataset['lon']
    self.bathymetry = dataset['Band1']
    self.bathymetry_n_bound = np.max(self.bathymetry_lat)
    self.bathymetry_s_bound = np.min(self.bathymetry_lat)
    self.bathymetry_e_bound = np.max(self.bathymetry_lon)
    self.bathymetry_w_bound = np.min(self.bathymetry_lon)

    print "Loading Bathymetry Data Complete"

    self.sim_factor = rospy.get_param('sim_speedup_factor')

    print "Beginning Simulation with sim factor:", self.sim_factor
    # print "ROMS Bounds:"
    # print "\tNorth", roms_n_bound, "deg Lat"
    # print "\tSouth", roms_s_bound, "deg Lat"
    # print "\tEast", roms_e_bound, "deg Lon"
    # print "\tWest", roms_w_bound, "deg Lon"

    # print "Bathymetry Bounds:"
    # print "\tNorth", bathymetry_n_bound, "deg Lat"
    # print "\tSouth", bathymetry_s_bound, "deg Lat"
    # print "\tEast", bathymetry_e_bound, "deg Lon"
    # print "\tWest", bathymetry_w_bound, "deg Lon"

    self.n_bound = min(self.bathymetry_n_bound, roms_n_bound)
    self.s_bound = max(self.bathymetry_s_bound, roms_s_bound)
    self.e_bound = min(self.bathymetry_e_bound, roms_e_bound)
    self.w_bound = max(self.bathymetry_w_bound, roms_w_bound)

 

    print "Simulation Bounds:"
    print "\tNorth", self.n_bound, "deg Lat"
    print "\tSouth", self.s_bound, "deg Lat"
    print "\tEast", self.e_bound, "deg Lon"
    print "\tWest", self.w_bound, "deg Lon"

    rospy.set_param('sim_n_bound', float(self.n_bound))
    rospy.set_param('sim_s_bound', float(self.s_bound))
    rospy.set_param('sim_e_bound', float(self.e_bound))
    rospy.set_param('sim_w_bound', float(self.w_bound))



    print "Initializing Publishers and Services"
    self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)

    rospy.Service(rospy.get_param("world_sim_topic"), SimQuery, self.simQueryHandle)
    rospy.Service('WorldSnapshot', SimSnapshot, self.simSnapshotHandle)

    self.sim_time = self.roms_dates[0] #Initialize simulated clock with beginning of ROMS Data

    self.prev_time = datetime.datetime.now()
    msg = Clock()
    msg.clock = rospy.Time.from_sec(getTimestamp(self.sim_time))
    self.clock_pub.publish(msg)



  def stepSimulator(self):
    current_time = datetime.datetime.now()
    elapsed = current_time - self.prev_time
    self.sim_time += elapsed * self.sim_factor
    msg = Clock()
    msg.clock = rospy.Time.from_sec(getTimestamp(self.sim_time))
    self.clock_pub.publish(msg)
    self.prev_time = current_time

    if self.sim_time > self.future_roms.date:
      print "ROMS Rollover"
      #Increment Simulator
      self.past_roms = self.future_roms
      self.future_roms = ROMSSnapshot(self.past_roms.idx + 1, self.roms_dates, self.roms_files)





    




    # surface_current_u = self.current_u[0]
    # surface_current_v = self.current_v[0]

    # # print
    # plt.figure()
    # CS = plt.imshow(self.temperature[0], interpolation='none', origin='lower')
    # cbar = plt.colorbar(CS)
    # cbar.set_label("Ocean Temperature (Celsius)")
    # plt.title("Monterey Bay Ocean Conditions: October 16 3:00 PM")
    # # #plt.show()

    # # print self.current_u.shape
    # lat_mesh, lon_mesh = np.meshgrid(range(self.current_u.shape[2]), range(self.current_u.shape[1]))

    # # plt.figure()
    # # plt.imshow(self.current_u[0], interpolation='none', origin='lower')

    # # plt.figure()
    # # plt.imshow(self.current_v[0], interpolation='none', origin='lower')
    # scale_factor = 10
    # # plt.figure()
    # plt.quiver(lat_mesh[::scale_factor, ::scale_factor], lon_mesh[::scale_factor, ::scale_factor], surface_current_u[::scale_factor, ::scale_factor], surface_current_v[::scale_factor, ::scale_factor])
    # plt.show()
    
    # print self.current_u.shape
    # self.interp3d(0, 0)
  
  def interp4d(self, loc, data_type):
    pt = [loc.depth, loc.latitude, loc.longitude]
    total_sec = (self.future_roms.date - self.past_roms.date).total_seconds()
    past_sec = (self.sim_time - self.past_roms.date).total_seconds()
    future_sec = (self.future_roms.date - self.sim_time).total_seconds()

    if data_type == "Temperature":
      pastInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.past_roms.temperature)
      futureInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.future_roms.temperature)
      past_pt = pastInterp(pt)
      future_pt = futureInterp(pt)
      try:
        return (1-(past_sec/total_sec))*past_pt + (1-(future_sec/total_sec))*future_pt
      except ZeroDivisionError:
        return past_pt

    elif data_type == "Salinity":
      pastInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.past_roms.salinity)
      futureInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.future_roms.salinity)
      past_pt = pastInterp(pt)
      future_pt = futureInterp(pt)
      try:
        return (1-(past_sec/total_sec))*past_pt + (1-(future_sec/total_sec))*future_pt
      except ZeroDivisionError:
        return past_pt

    elif data_type == "CurrentU":
      pastInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.past_roms.current_u)
      futureInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.future_roms.current_u)
      past_pt = pastInterp(pt)
      future_pt = futureInterp(pt)
      try:
        return (1-(past_sec/total_sec))*past_pt + (1-(future_sec/total_sec))*future_pt
      except ZeroDivisionError:
        return past_pt

    elif data_type == "CurrentV":
      pastInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.past_roms.current_v)
      futureInterp = RegularGridInterpolator((self.depth, self.roms_lat, self.roms_lon), self.future_roms.current_v)
      past_pt = pastInterp(pt)
      future_pt = futureInterp(pt)
      try:
        return (1-(past_sec/total_sec))*past_pt + (1-(future_sec/total_sec))*future_pt
      except ZeroDivisionError:
        return past_pt

    else:
      print "Unknown Data Type"
      return float("NaN")

    # # pt = [(self.depth[0] + self.depth[1])/2, (self.lat[52] + self.lat[53])/2, (self.lon[52] + self.lon[53])/2]

    # return interpTemp(pt)


  def pointInBounds(self, geopoint):
    if geopoint.latitude > self.n_bound or geopoint.latitude < self.s_bound:
      return False
    elif geopoint.longitude > self.e_bound or geopoint.longitude < self.w_bound:
      return False
    else:
      return True



  def simQueryHandle(self, req):
    if self.pointInBounds(req.loc):
      res = STU()
      res.temperature = self.interp4d(req.loc, "Temperature")
      res.salinity = self.interp4d(req.loc, "Salinity")
      res.current = Vector3(self.interp4d(req.loc, "CurrentU"), self.interp4d(req.loc, "CurrentV"), 0.0)
    else:
      res = STU()
      res.temperature = -1e9
      res.salinity = -1e9
      res.current = Vector3(-1e9, -1e9, -1e9)
    return res

  def simSnapshotHandle(self, req):
      res = Array3D()
      depth, height, width = self.past_roms.temperature.shape
      res.height = height #Latitude
      res.width = width #Longitude
      res.depth = depth #Depth
      res.e_bound = self.e_bound
      res.w_bound = self.w_bound
      res.s_bound = self.s_bound
      res.n_bound = self.n_bound

      if req.ocean_feature == 0:
        #Temperature
        res.data = self.past_roms.temperature.flatten()

      elif req.ocean_feature == 1:
        #Salinity
        res.data = self.past_roms.salinity.flatten()

      elif req.ocean_feature == 2:
        #Current U
        res.data = self.past_roms.current_u.flatten()

      elif req.ocean_feature == 3:
        #Current V
        res.data = self.past_roms.current_v.flatten()

      elif req.ocean_feature == 4:
        #Bathymetry
        height, width = self.bathymetry.shape
        res.height = height #Latitude
        res.width = width #Longitude
        res.depth = 1 #Depth
        res.e_bound = self.bathymetry_e_bound
        res.w_bound = self.bathymetry_w_bound
        res.s_bound = self.bathymetry_s_bound
        res.n_bound = self.bathymetry_n_bound

        res.data = self.bathymetry.flatten()

      else:
        res.data = self.past_roms.temperature.flatten()

      return res


def main():

  rospy.init_node("world_sim")
  simulator = WorldSim()
  print "Simulator Online"
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    # print "Sim Time:", simulator.sim_time
    simulator.stepSimulator()
    rate.sleep()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
