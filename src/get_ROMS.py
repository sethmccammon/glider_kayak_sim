#!/usr/bin/env python

import rospkg, datetime, math, urllib, rospy, os, pdb

from utils import dateRange

def getROMS(start_date, end_date):
  # year = 2017
  # month = 10
  # day = 16
  # hour = 3
  rospack = rospkg.RosPack()
  package_path = rospack.get_path('glider_kayak_sim')

  print "Loading ROMS Data"
  files = []
  dates = []
  for dt in dateRange(start_date, end_date, datetime.timedelta(hours=1)):
    filename = "ca300m_das_%04d%02d%02d%02d.nc" % (dt.year, dt.month, dt.day, dt.hour)
    url = "http://west.rssoffice.com:8080/thredds/fileServer/roms/CA300m-nowcast/" + filename
    file_path = package_path + "/data/" + filename
    if not os.path.isfile(file_path):
      try:
        testfile = urllib.URLopener()
        testfile.retrieve(url, file_path)
        print dt, "\tLoading Data"
        files.append(file_path)
        dates.append(dt)
        # print url
        print file_path
      except IOError:
        pass
      print dt, "\tNo Data"
      # pass
    else:
      # pass
      print dt, "\tFile Already Exists"
      files.append(file_path)
      dates.append(dt)
  return dates, files

  # print urllib2.urlopen(url)
  # filename = wget.download(url)

def getBathymetry(n_bound, s_bound, e_bound, w_bound):
  rospack = rospkg.RosPack()
  package_path = rospack.get_path('glider_kayak_sim')
  print "Loading Bathymetry Data"
  filename = "ca300m_bathymetry.nc"
  url = "https://maps.ngdc.noaa.gov/mapviewer-support/wcs-proxy/wcs.groovy?filename=%s&request=getcoverage&version=1.0.0&service=wcs&coverage=etopo1&CRS=EPSG:4326&format=netcdf&resx=0.016666666666666667&resy=0.016666666666666667&bbox=%f,%f,%f,%f" % (filename, w_bound, s_bound, e_bound ,n_bound)
  file_path = package_path + "/data/" + filename
  if not os.path.isfile(file_path):
    while True: #Possible Infinite redirect loop
      try:
        testfile = urllib.URLopener()
        testfile.retrieve(url, file_path)
        print "\tLoading Data:", n_bound, s_bound, e_bound, w_bound
        break
      except IOError as err:
        if err[1] == 301: #Redirect Error
          url = err[3]['Location']
        else:
          print "Unknown Error loading Data"
          break
  else:
    print "File already Exists"
  return file_path




def getClosestHour(dt):
  base = 6
  hour = int(base * math.floor(float(dt.hour-3)/base)) + 3
  if hour < 0:
    dt = dt - datetime.timedelta(1)
    return datetime.datetime(dt.year, dt.month, dt.day, 21)
  else:
    return datetime.datetime(dt.year, dt.month, dt.day, hour)

if __name__ == '__main__':
  getBathymetry(37.045, 36.4, -121.773986816, -122.74899292)
