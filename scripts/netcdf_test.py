import netCDF4 as nc

import numpy as np


def main():
  rootgrp = nc.Dataset("../data/foobar.nc", "w", format="NETCDF4")


  depth = rootgrp.createDimension("depth", 24)
  time = rootgrp.createDimension("time", None)
  lat = rootgrp.createDimension("lat", 216)
  lon = rootgrp.createDimension("lon", 326)

  latitudes = rootgrp.createVariable("lat","f4",("lat",))
  latitudes.units = "degrees_north"
  latitudes.long_name = "Latitude"

  longitudes = rootgrp.createVariable("lon","f4",("lon",))
  longitudes.units = 'degrees_east'
  longitudes.long_name = 'Longitude'

  temp = rootgrp.createVariable("temp","f4",("time", "depth", "lat", "lon"))
  temp.long_name = 'Temperature'
  temp.units = 'degrees C'
  temp.missing_value = -9999


  lats =  np.arange(0, 216)
  lons =  np.arange(0, 326)

  latitudes[:] = lats
  longitudes[:] = lons


  surface_temp = np.random.uniform(size=(1, 24, 216, 326))
  print surface_temp.shape
  temp[:,:,:,:] = surface_temp





  rootgrp.close()



if __name__ == '__main__':
  main()