
#this is just a test script for sending a simple repeating message and printing any messages that come in for a certain call
#probably soon to be deleted

#don't allow this to be run as a standalone
if __name__ == "__main__":
	import sys
	sys.exit(0)

import rospy
import messaging

def handle(payload):
	print payload

def _loop():
	messaging.sendMessage("test2", {"hello" : "world"}, "3N")

messaging.registerMessageHandler("test", handle)