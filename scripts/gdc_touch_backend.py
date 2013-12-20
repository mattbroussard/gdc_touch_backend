#!/usr/bin/env python

#general imports
import sys
import rospy

#allow our other modules to
# - run things after rospy is initialized
initfuncs = []
# - run things in the main loop
loopfuncs = []
# - run things to cleanup on shutdown
donefuncs = []

#import other modules of ours
import messaging
import test_printer

def main():
	global initfuncs, loopfuncs, donefuncs

	rospy.init_node("gdc_touch_backend")

	for x in initfuncs:
		x()

	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		for x in loopfuncs:
			x()
		r.sleep()

	for x in donefuncs:
		x()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException, e:
		rospy.logerr("An error occurred in main: %s", e)
		sys.exit(1)
