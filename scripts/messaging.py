
#don't allow this to be run as a standalone
if __name__ == "__main__":
	import sys
	sys.exit(0)

#configuration
config = {
	"max_attempts" : 5,
	"retry_delay" : 5,
	"received_queue_length" : 500
}

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
received = deque(maxlen=config["received_queue_length"])
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
	global callbacks
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
			"attempts" : 1,
			"msg" : msg
		}
	
	pub.publish(String(json.dumps(msg)))

#sends a receipt for the given message metadata
def sendReceipt(msg_meta):
	msg = {
		"receipt" : True,
		"meta" : msg_meta
	}
	pub.publish(String(json.dumps(msg)))

#returns True if we have received a message with the given metadata recently
def haveReceived(msg_meta):
	global received_lock, received
	with received_lock:
		for i in received:
			if i == msg_meta:
				return True
	return False

#called when a new message comes in
def processIncoming(msg):
	global sent_lock, sent, received, received_lock, callbacks
	
	#does the message have valid metadata?
	if "meta" not in msg:
		return
	if "dest" not in msg["meta"]:
		return
	if "src" not in msg["meta"]:
		return
	if "id" not in msg["meta"]:
		return

	#is the message a receipt?
	if "receipt" in msg:
		with sent_lock:
			id = msg["meta"]["id"]
			if id in sent:
				del sent[id]
		return

	#is it for us?
	if msg["meta"]["dest"] != "server":
		return

	#if it's not a receipt, and it is for us, does it have a valid call attribute?
	if "call" not in msg:
		return
	if msg["call"] not in callbacks:
		return

	#have we seen this already?
	if haveReceived(msg["meta"]):
		return

	with received_lock:
		received.append(msg["meta"])

	#send a receipt
	sendReceipt(msg["meta"])

	#get the payload and call the callback
	payload = None
	if "payload" in msg:
		payload = msg["payload"]
	callback = callbacks[msg["call"]]
	callback(payload)

#called periodically to resend any messages for which receipts have not been received
def processOutgoing():
	global sent, sent_lock, config

	now = time()
	resend = []
	with sent_lock:
		drop = []
		for i in sent:
			if sent[i]["attempts"] >= config["max_attempts"]:
				rospy.loginfo("Dropped message: %s", json.dumps(sent[i]["msg"]))
				drop.append(i)
				continue
			if now - sent[i]["timestamp"] > config["retry_delay"]:
				sent[i]["attempts"] += 1
				sent[i]["timestamp"] = now
				resend.append(sent[i])
		for i in drop:
			del sent[i]

	for i in resend:
		pub.publish(String(json.dumps(i["msg"])))


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
	rospy.loginfo("Now publishing on /gdc_touch_server")
	rospy.Subscriber("gdc_touch_client", String, subscriberCallback)
	rospy.loginfo("Now listening on /gdc_touch_client")

#special calls
def _init():
	initMessaging()
def _loop():
	processOutgoing()