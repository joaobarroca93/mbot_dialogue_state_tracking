#!/usr/bin/env python

import rospy
import rospkg

import json
import os

#from std_msgs.msg import int32 # TURN NUMBER !!
from mbot_dst_rule_based.msg import DialogState
#from mbot_dst_rule_based.mbot_dst_rule_based_common_v2 import DialogueStateTracking
from mbot_dst_rule_based.mbot_dst_rule_based_common import DialogueAct as MbotDialogueAct
from mbot_dst_rule_based.mbot_dst_rule_based_common import Slot as MbotSlot
from mbot_dst_rule_based.mbot_dst_rule_based_common import DialogueStateTracking

from mbot_nlu_pytorch.msg import (InformSlot, DialogAct,
								DialogActArray, ASRHypothesis,
								ASRNBestList)


# parameters 
# '~loop_rate'
# '~node_name'
# '~slots'
# '~dialogue_acts_topic_name'
# '~belief_topic_name'
# '~onthology_full_name'

# NEED TO ADD A SUB TOPIC TO INDICATE THE TURN NUMBER !!!



"""
Description: This function helps logging parameters as debug verbosity messages.

Inputs:
	- param_dict: a dictionary whose keys are the parameter's names and values the parameter's values.
"""
def logdebug_param(param_dict):
	[ rospy.logdebug( '{:<20}\t{}'.format(param[0], param[1]) ) for param in param_dict.items() ]



class DSTNode(object):

	def __init__(self, debug=False):

		# get useful parameters, and if any of them doesn't exist, use the default value
		rate 			= rospy.get_param('~loop_rate', 10.0)
		slots 			= rospy.get_param('~slots', ['intent', 'person', 'object', 'source', 'destination'])
		node_name 		= rospy.get_param('~node_name', 'dialogue_state_tracking')
		d_state_topic 	= rospy.get_param('~d_state_topic_name', '/dialogue_state')
		d_acts_topic 	= rospy.get_param('~dialogue_acts_topic_name', '/dialogue_acts')
		system_response = rospy.get_param('~system_response_topic_name', '/system_response')
		onthology_name 	= rospy.get_param('~onthology_full_name', 'ros/src/mbot_dst_rule_based_ros/onthology.json')

		# initializes the node (if debug, initializes in debug mode)
		if debug == True:
			rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
			rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
		else:
			rospy.init_node(node_name, anonymous=False)
			rospy.loginfo("%s node created" % node_name)

		# set parameters to make sure all parameters are set to the parameter server
		rospy.set_param('~loop_rate', rate)
		rospy.set_param('~slots', slots)
		rospy.set_param('~node_name', node_name)
		rospy.set_param('~d_state_topic_name', d_state_topic)
		rospy.set_param('~dialogue_acts_topic_name', d_acts_topic)
		rospy.set_param('~system_response_topic_name', system_response)
		rospy.set_param('~onthology_full_name', onthology_name)

		rospy.logdebug('=== NODE PRIVATE PARAMETERS ============')
		logdebug_param(rospy.get_param(node_name))

		rospack = rospkg.RosPack()
		# get useful paths
		generic_path 	= rospack.get_path("mbot_dst_rule_based")
		onthology_path 	= os.path.join(generic_path, onthology_name)
		logdebug_param({'generic_path': generic_path, 'onthology_full_path': onthology_path})

		"""
		with open(onthology_path, 'r') as fp:
			onthology_dict = json.load(fp)
		belief = {slot: {value: 0.0 for value in onthology_dict[slot] } for slot in slots }
		rospy.logdebug('=== DIALOGUE INITIAL BELIEF ============')
		rospy.logdebug(belief)
		"""

		#self.dst_object = DialogueStateTracking(slots=slots, initial_belief=belief)
		self.dst_object = DialogueStateTracking()
		rospy.loginfo('dialogue state tracking object created')

		self.dst_request_received = False
		self.loop_rate = rospy.Rate(rate)
		self.last_system_response = {
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

		rospy.Subscriber(d_acts_topic, DialogActArray, self.dstCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", d_acts_topic)

		rospy.Subscriber(system_response, DialogAct, self.systemResponseCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", system_response)

		self.pub_dialogue_state = rospy.Publisher(d_state_topic, DialogState, queue_size=1)
		
		rospy.loginfo("%s initialization completed! Ready to accept requests" % node_name)


	def dstCallback(self, dialogue_act_array_msg):

		rospy.loginfo('[Message received]')
		rospy.logdebug('{}'.format(dialogue_act_array_msg))

		self.dst_request_received = True
		self.dialogue_acts = dialogue_act_array_msg.dialogue_acts


	def systemResponseCallback(self, dialogue_act_msg):

		#rospy.loginfo('[System response updated]')
		#rospy.logdebug('{}'.format(dialogue_act_msg))

		self.last_system_response = MbotDialogueAct(
			dtype=dialogue_act_msg.dtype,
			slots=[
				MbotSlot(type=slot.slot, value=slot.value)
			for slot in dialogue_act_msg.slots]
		)


	def begin(self):

		THRESHOLD = 0.1

		while not rospy.is_shutdown():

			if self.dst_request_received == True:

				rospy.loginfo('[Handling message]')
				self.dst_request_received = False

				dialogue_acts = [
					MbotDialogueAct(
						dtype=dialogue_act.dtype,
						confidence=dialogue_act.d_type_confidence,
						slots=[MbotSlot(
							type=slot.slot,
							value=slot.value,
							confidence=slot.confidence
						) for slot in dialogue_act.slots if slot.known == True]
					)
				for dialogue_act in self.dialogue_acts ]

				"""
				dialogue_acts = [{
					"d-type": dialogue_act.dtype,
					"slots": { slot.slot: {'value': slot.value, 'probability': slot.confidence}  for slot in dialogue_act.slots if slot.known == True},
					"probability": dialogue_act.d_type_confidence
				} for dialogue_act in self.dialogue_acts ]
				"""

				rospy.logdebug('dialogue_acts_dict: {}'.format(
					[dialogue_act.as_dict() for dialogue_act in dialogue_acts]
				))


				rospy.loginfo('[Updating belief]')
				self.dst_object.update_belief(dialogue_acts, self.last_system_response, normalize=False)
				rospy.loginfo('belief: {}'.format(self.dst_object.belief.as_dict()))

				dialogue_state = DialogState()
				dialogue_state.slots.extend([
					InformSlot(
						slot=slot.type,
						value=slot.value,
						confidence=slot.confidence,
						known=True
					)for slot in self.dst_object.belief.slots]
				)

				self.pub_dialogue_state.publish(dialogue_state)
				

			self.loop_rate.sleep()


def main():

	dst_node = DSTNode(debug=True)
	dst_node.begin()
