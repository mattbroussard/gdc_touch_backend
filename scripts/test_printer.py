
#don't allow this to be run as a standalone
if __name__ == "__main__":
	import sys
	sys.exit(0)

import rospy
import messaging

def handle(payload):
	print payload

messaging.registerMessageHandler("test", handle)