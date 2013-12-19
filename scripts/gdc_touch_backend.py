#!/usr/bin/env python

#general imports
import sys, json

#import ROS stuff
import rospy
from std_msgs.msg import String

#global variables
pub = None

def subscriberCallback(msg):
	try:
		obj = json.loads(msg.data)

		#TODO: handle received message

	except e:
		rospy.logerr("An error occurred in subscriberCallback: %s", e)

def main():
	global pub

	rospy.init_node("gdc_touch_backend")

	#TODO: do init work such as opening MySQL connection, etc.

	pub = rospy.Publisher("gdc_touch_server", String)
	rospy.Subscriber("gdc_touch_client", String, subscriberCallback)
	
	rospy.loginfo("gdc_touch_backend now listening.")

	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		
		#TODO: check for messages that need to be resent; do other periodic work

		r.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException, e:
		rospy.logerr("An error occurred in main: %s", e)
		sys.exit(1)
