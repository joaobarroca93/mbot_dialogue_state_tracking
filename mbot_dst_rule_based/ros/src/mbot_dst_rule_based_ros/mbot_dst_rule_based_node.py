#!/usr/bin/env python

import rospy
import rospkg

import json

#from std_msgs.msg import int32
from mbot_nlu_bert.msg import InformSlot, DialogAct, DialogActArray
from mbot_dst_rule_based.mbot_dst_rule_based_common import DialogueStateTracking


# parameters 
LOOP_RATE = 10.0
PUB_TOPIC_NAME = '/belief'
SUB_TOPIC_NAME = '/dialogue_acts'
NODE_NAME = 'dialogue_state_tracking'
SLOTS = ['intent', 'person', 'object', 'source', 'destination']
ONTHOLOGY_PATH = '/ros/src/mbot_dst_rule_based_ros/onthology.json'


# NEED TO ADD A SUB TOPIC TO INDICATE THE TURN NUMBER !!!

class DSTNode(object):

	def __init__(self, name):

		rospack = rospkg.RosPack()

		rospy.init_node(name, anonymous=False)

		slots = SLOTS
		onthology_path = rospack.get_path("mbot_dst_rule_based") + ONTHOLOGY_PATH

		with open(onthology_path, 'r') as fp:
			onthology_dict = json.load(fp)
		belief = {slot: {value: 0.0 for value in onthology_dict[slot] } for slot in slots }

		self.dst_object = DialogueStateTracking(slots=slots, initial_belief=belief)

		self.dst_request_received = False

		self.loop_rate = rospy.Rate(LOOP_RATE)

		rospy.Subscriber(SUB_TOPIC_NAME, DialogActArray, self.dstCallback, queue_size=1)
		#self.pub_belief = rospy.Publisher(PUB_TOPIC_NAME, DialogActArray, queue_size=1)
		rospy.loginfo("%s initialized, ready to accept requests" % NODE_NAME)


	def dstCallback(self, dialogue_act_array_msg):

		self.dst_request_received = True
		self.dialogue_acts = dialogue_act_array_msg.dialogue_acts


	def begin(self):

		while not rospy.is_shutdown():

			if self.dst_request_received == True:

				self.dst_request_received = False

				dialogue_acts = [{
					"d-type": dialogue_act.dtype,
					"slots": { slot.slot: slot.value  for slot in dialogue_act.slots },
					"probability": 0.8
				} for dialogue_act in self.dialogue_acts ]

				rospy.loginfo("\n\n")
				rospy.loginfo("[Received] %s" % dialogue_acts)

				last_system_response = {
					"d-type": "hello",
					"slots": {
						"intent": "none",
						"object": "none",
						"person": "none",
						"source": "none",
						"destination": "none"
					},
					"requestable": []
				}

				self.dst_object.update_belief(dialogue_acts, last_system_response)
				rospy.loginfo("[Belief] %s" % self.dst_object.belief)

			self.loop_rate.sleep()


def main():

	dst_node = DSTNode(name=NODE_NAME)
	dst_node.begin()
