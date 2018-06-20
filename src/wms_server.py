#!/usr/bin/env python

import rospy
import os
import subprocess
import xml.etree.ElementTree as ET
from netCDF4 import Dataset
import numpy as np

def generate_server_cfg(cfg_path,wms_server_params,wms_datasets):

	config		= ET.Element('config')
	datasets	= ET.SubElement(config,'datasets')
	cache		= ET.SubElement(config,'cache')
	server		= ET.SubElement(config,'server')
  
  	# Database parameters
	cache.set('enabled', str(wms_server_params['cache']).lower())
	ET.SubElement(server,'title').text = wms_server_params['title']

	# Datasets
	tile_dataset_vars = ['id', 'title', 'location', 'updateInterval']
	for tile_dataset in wms_datasets:
		dataset		= ET.SubElement(datasets, 'dataset')
		for v in tile_dataset_vars:
			dataset.set(v, wms_datasets[tile_dataset][v])

	# Storing file
	tree = ET.ElementTree(config)
	tree.write(cfg_path, encoding='UTF-8')

def main():

	rospy.init_node("wms_server")

	# Parameters
	server_cfg_path		= os.path.expanduser('~')+'/.ncWMS2/'
	package_path		= rospy.get_param('package_path')
	wms_server_params	= rospy.get_param('wms_server')	
	wms_datasets		= rospy.get_param('wms_datasets')
	for d in wms_datasets:
		wms_datasets[d]['location'] = package_path+'/'+wms_datasets[d]['location']

	# Generating server config file
	p = subprocess.Popen('mkdir -p '+server_cfg_path, shell=True)
	p.wait()
	rospy.loginfo('wms_server: Server Starting...')
	generate_server_cfg(server_cfg_path+'config.xml',wms_server_params,wms_datasets)
	rospy.loginfo('wms_server: Server Running.')

	# Running Server
	port_command = '--httpPort=%d' % wms_server_params['port']
	jar_name = 'map_server.jar'
	jar_path = '%s/web/%s' % (package_path,jar_name)
	if wms_server_params['verbose']:
		subprocess.call(['java', '-jar', jar_path, port_command])
	else:
		with open(os.devnull, 'w') as shutup:
			subprocess.call(['java', '-jar', jar_path, port_command], stdout=shutup, stderr=shutup)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
