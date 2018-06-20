#!/usr/bin/env python

import rospy
import BaseHTTPServer
import threading
import json
import mimetypes
from glider_kayak_sim.srv import *
from glider_kayak_sim.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

# =========================================================
# Web Server Request Handler Class
# =========================================================
def HandlerFactory(dataset):
	class http_handler(BaseHTTPServer.BaseHTTPRequestHandler, object):

		def __init__(self, *args, **kwargs):
			self.dataset = dataset
			super(http_handler, self).__init__(*args, **kwargs)

		def do_HEAD(self):
			self.send_response(200)
			self.send_header('Content-type', 'text/html')
			self.end_headers()

		def do_GET(self):

			# Requested file
			if self.path == '/':
				ret_file = self.path+'index.html'
			else:
				ret_file = self.path
			ret_file = self.dataset['params']['package_path']+'/web'+ret_file

			try:
				with open(ret_file, 'rb') as f:
					ret_contents = f.read()
				self.send_response(200)
				self.send_header('Content-type', mimetypes.guess_type(ret_file)[0])
				self.end_headers()
				self.wfile.write(ret_contents)
			except:
				self.send_error(404, 'File not found')

		def log_message(self, format, *args):
			return

		def do_POST(self):
			self.send_response(200)
			self.send_header('Content-type', 'application/json')
			self.end_headers()
			req = self.rfile.read(int(self.headers['Content-length']))
			self.wfile.write(json.dumps(self.dataset[req]))


	return http_handler

# =========================================================
# ROS Callbacks
# =========================================================

def ugeo_pose_callback(data, robot):
	robot['pos'] = [data.position.longitude, data.position.latitude]

# =========================================================
# ROS Node
# =========================================================
def main():

	# Initializing node
	rospy.init_node('ros_web_interface')

	# Getting Parameters
	dataset				= {}
	dataset['robots']	= {}
	dataset['params']	= rospy.get_param('')

	server_address		= (
		dataset['params']['web_viz']['host_name'],
		dataset['params']['web_viz']['web_gui_port']
	)

	robot_topics_list	= [
		(dataset['params']['kayak_namespace'],	dataset['params']['num_kayaks']),
		(dataset['params']['glider_namespace'],	dataset['params']['num_gliders'])
	]

	for robot_namespace, num_robots in robot_topics_list:
		for topic_name in [(robot_namespace+dataset['params']['robot_pose_topic']) % i for i in range(num_robots)]:
			dataset['robots'][topic_name] = {
				'ros_topic': topic_name,
				'name': topic_name.split('/')[1],
				'type': robot_namespace.split('/')[1].split('_')[0],
				'pos': [0.0,0.0],
			}
			rospy.Subscriber(topic_name, UnderwaterGeoPose, ugeo_pose_callback, dataset['robots'][topic_name], queue_size=dataset['params']['viz_int']['topic_queue_size'])

	httpd = BaseHTTPServer.HTTPServer(server_address, HandlerFactory(dataset))
	rospy.loginfo('ros_web_interface: Server Started - %s:%s' % server_address)
	try:
		thread = threading.Thread(target = httpd.serve_forever)
		thread.daemon = True
		thread.start()
	except:
		pass
	rospy.loginfo('ros_web_interface: Server Running - %s:%s' % server_address)

	rospy.spin()

	httpd.server_close()

	rospy.loginfo('ros_web_interface: Server Stopped - %s:%s' % server_address)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
