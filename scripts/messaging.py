
#don't allow this to be run as a standalone
if __name__ == "__main__":
	import sys
	sys.exit(0)

#general imports
import rospy
from std_msgs.msg import String
import threading
import json
from time import time
from collections import deque

#globals for messaging
pub = None
callbacks = {}
sent = {}
received = deque(max_len=500)
nextid = 0

#locks for messaging globals, since messaging methods can be called from main thread or subscriber callback thread
id_lock = threading.Lock()
sent_lock = threading.Lock()
received_lock = threading.Lock()

#generates a new message id
def newMessageID():
	global nextid, id_lock
	with id_lock:
		nextid += 1
		return nextid

#called by other modules to declare they can handle incoming messages
def registerMessageHandler(call, handler):
	callbacks[call] = handler

#called by other modules to send a message with given payload and call to the given destination
def sendMessage(call, payload, dest):
	global sent_lock, sent, pub

	msg = {
		"meta" : {
			"src" : "server",
			"dest" : dest,
			"id" : newMessageID()
		},
		"call" : call,
		"payload" : payload
	}

	with sent_lock:
		sent[msg["meta"]["id"]] = {
			"timestamp" : time(),
			"msg" : msg
		}
	
	pub.publish(String(json.dumps(msg)))

#calls other things to actually process the message
def receiveMessage(msg):
	#TODO: finish implementation
	pass

#sends a receipt for the given message metadata
def sendReceipt(msg_meta):
	#TODO: finish implementation
	pass

#called when a new message comes in
# - if receipt, process appropriately.
# - decides if we've received it already
# - sends a receipt
# - calls receiveMessage if needed
def processIncoming(msg):
	
	if "meta" not in msg:
		return

	if "receipt" in msg:
		with sent_lock:
			id = msg["meta"]["id"]
			if id in sent:
				del sent[id]
		return

	if "call" not in msg:
		return

	#TODO: finish implementation
	pass


#called periodically to resend any messages for which receipts have not been received
def processOutgoing():
	#TODO: finish implementation
	pass

#raw ROS message subscription callback
def subscriberCallback(msg):
	try:
		processIncoming(json.loads(msg.data))
	except e:
		rospy.logerr("An error occurred in subscriberCallback: %s", e)

#called to initialize publisher/subscriber
def initMessaging():
	global pub
	pub = rospy.Publisher("gdc_touch_server", String)
	rospy.Subscriber("gdc_touch_client", String, subscriberCallback)
	rospy.loginfo("Now listening on /gdc_touch_client")

#register our stuff
import gdc_touch_backend
gdc_touch_backend.initfuncs.append(initMessaging)
gdc_touch_backend.loopfuncs.append(processOutgoing)