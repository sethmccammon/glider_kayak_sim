#!/usr/bin/env python

import rospkg, datetime, math, urllib

def main():
  year = 2016
  month = 1
  day = 1
  hour = 3
  rospack = rospkg.RosPack()
  package_path = rospack.get_path('glider_kayak_sim')
  # dt = datetime.datetime(year, month, day, hour)
  dt = datetime.datetime.now() - datetime.timedelta(1)
  dt = getClosestHour(dt)
  # http://west.rssoffice.com:8080/thredds/fileServer/roms/CA300m-nowcast/ca300m_das_2017101621.nc
  filename = "ca300m_das_%04d%02d%02d%02d.nc" % (dt.year, dt.month, dt.day, dt.hour)
  url = "http://west.rssoffice.com:8080/thredds/fileServer/roms/CA300m-nowcast/" + filename
  print url
  print package_path + "/data/" + filename
  testfile = urllib.URLopener()
  testfile.retrieve(url, package_path + "/data/" + filename)

  # print urllib2.urlopen(url)
  # filename = wget.download(url)


def getClosestHour(dt):
  base = 6
  hour = int(base * math.floor(float(dt.hour-3)/base)) + 3
  if hour < 0:
    dt = dt - datetime.timedelta(1)
    return datetime.datetime(dt.year, dt.month, dt.day, 21)
  else:
    return datetime.datetime(dt.year, dt.month, dt.day, hour)

if __name__ == '__main__':
  # print range(24)
  # print [getClosestHour(x) for x in range(24)]
  main()