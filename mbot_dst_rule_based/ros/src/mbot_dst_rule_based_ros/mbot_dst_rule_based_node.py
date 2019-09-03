#!/usr/bin/env python

import rospy
import rospkg

import json
import os
import yaml

from std_msgs.msg import String
from mbot_dst_rule_based.msg import DialogState
from mbot_dst_rule_based.mbot_dst_rule_based_common import DialogueAct as MbotDialogueAct
from mbot_dst_rule_based.mbot_dst_rule_based_common import Slot as MbotSlot
from mbot_dst_rule_based.mbot_dst_rule_based_common import DialogueStateTracking

from mbot_nlu_pytorch.msg import (InformSlot, DialogAct,
								DialogActArray, ASRHypothesis,
								ASRNBestList)


class DSTNode(object):

	def __init__(self):

		rospack = rospkg.RosPack()
		generic_path = rospack.get_path("mbot_dst_rule_based")

		try:
			# need to download a .yaml config with all the needed parameters !
			rospy.loginfo("Loading node config")
			config_path = rospy.myargv()[1]
			config = yaml.load(open(config_path))
		except IndexError:
			rospy.loginfo("Loading node config")
			config_path = os.path.join(generic_path, "ros/config/config_mbot_dst_rule_based.yaml")
			config = yaml.load(open(config_path))

		node_name 				= config["node_params"]["name"]
		rate 					= config["node_params"]["rate"]
		debug 					= config["node_params"]["debug"]
		restart_threshold 		= config["node_params"]["restart_threshold"]
		d_state_topic 			= config["node_params"]["d_state_topic"]
		d_acts_topic 			= config["node_params"]["d_acts_topic"]
		dialogue_status_topic	= config["node_params"]["dialogue_status_topic"]
		system_response_topic 	= config["node_params"]["system_response_topic"]

		# initializes the node (if debug, initializes in debug mode)
		if debug == True:
			rospy.init_node(node_name, anonymous=False, log_level=rospy.DEBUG)
			rospy.loginfo("%s node created [DEBUG MODE]" % node_name)
		else:
			rospy.init_node(node_name, anonymous=False)
			rospy.loginfo("%s node created" % node_name)


		self.dst_object = DialogueStateTracking(restart_threshold=restart_threshold)
		rospy.loginfo('dialogue state tracking object created')

		self.dst_request_received = False
		self.dialogue_status_received = False
		self.loop_rate = rospy.Rate(rate)
		self.last_system_response = None

		rospy.Subscriber(d_acts_topic, DialogActArray, self.dstCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", d_acts_topic)

		rospy.Subscriber(system_response_topic, DialogAct, self.systemResponseCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", system_response_topic)

		rospy.Subscriber(dialogue_status_topic, String, self.dialogueStatusCallback, queue_size=1)
		rospy.loginfo("subscribed to topic %s", dialogue_status_topic)

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

	def dialogueStatusCallback(self, dialogue_status_msg):

		if dialogue_status_msg.data == "finish":
			self.dst_object.initialize_belief()
			self.dialogue_status_received = True


	def begin(self):

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

				rospy.logdebug('dialogue_acts_dict: {}'.format(
					[dialogue_act.as_dict() for dialogue_act in dialogue_acts]
				))


				rospy.loginfo('[Updating belief]')
				self.dst_object.update_belief(dialogue_acts, self.last_system_response, normalize=False)
				rospy.loginfo('belief: {}'.format(self.dst_object.belief.as_dict()))

				allow_slots = ["intent", "object", "person", "source", "destination"]

				dialogue_state = DialogState()
				dialogue_state.slots.extend([
					InformSlot(
						slot=slot.type,
						value=slot.value,
						confidence=slot.confidence,
						known=True
					) for slot in self.dst_object.belief.slots if slot.type in allow_slots]
				)

				self.pub_dialogue_state.publish(dialogue_state)

			if self.dialogue_status_received:

				self.dialogue_status_received = False

				dialogue_state = DialogState()
				dialogue_state.slots.extend([
					InformSlot(
						slot=slot.type,
						value=slot.value,
						confidence=slot.confidence,
						known=True
					) for slot in self.dst_object.belief.slots if slot.type in allow_slots]
				)
				self.pub_dialogue_state.publish(dialogue_state)
				

			self.loop_rate.sleep()


def main():

	dst_node = DSTNode()
	dst_node.begin()
