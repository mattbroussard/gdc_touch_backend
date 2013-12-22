#!/usr/bin/env python

#an ordered list of our modules
modules = ["messaging", "test_printer"]

#general imports
import sys
import rospy

#load our modules in a slightly magical way
initfuncs = []
loopfuncs = []
donefuncs = []
for i in modules:
	x = __import__(i)
	if hasattr(x, "_init"):
		initfuncs.append(getattr(x, "_init"))
	if hasattr(x, "_loop"):
		loopfuncs.append(getattr(x, "_loop"))
	if hasattr(x, "_done"):
		donefuncs.append(getattr(x, "_done"))

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
