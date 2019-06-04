#!/usr/bin/env python

import rospy
import rospkg

import json
import yaml
import os

from mbot_nlu_bert.msg import InformSlot, DialogAct


"""
Description: This function helps logging parameters as debug verbosity messages.

Inputs:
	- param_dict: a dictionary whose keys are the parameter's names and values the parameter's values.
"""
def logdebug_param(param_dict):
	[ rospy.logdebug( '{:<25}\t{}'.format(param[0], param[1]) ) for param in param_dict.items() ]



class SystemResponseNode(object):

	def __init__(self, debug=False):

		# get useful parameters, and if any of them doesn't exist, use the default value
		rate 			= rospy.get_param('~loop_rate', 0.1)
		slots 			= rospy.get_param('~slots', ['intent', 'person', 'object', 'source', 'destination'])
		node_name 		= rospy.get_param('~node_name', 'dialogue_management')
		system_response_topic = rospy.get_param('~system_response_topic_name', '/system_response')
		system_response_filename 	= rospy.get_param('~system_response_filename', 'ros/src/mbot_dst_rule_based_ros/system_response.json')

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
		rospy.set_param('~system_response_topic_name', system_response_topic)
		rospy.set_param('~system_response_filename', system_response_filename)

		rospy.logdebug('=== NODE PRIVATE PARAMETERS ============')
		logdebug_param(rospy.get_param(node_name))

		rospack = rospkg.RosPack()
		# get useful paths
		generic_path = rospack.get_path("mbot_dst_rule_based")
		system_response_path = os.path.join(generic_path, system_response_filename)
		logdebug_param({'generic_path': generic_path, 'system_response_full_path': system_response_path})

		self.loop_rate = rospy.Rate(rate)
		self.system_response_path = system_response_path
		self.slots = slots

		self.pub_belief = rospy.Publisher(system_response_topic, DialogAct, queue_size=1)
		rospy.loginfo("publishing to topic %s", system_response_topic)


	def begin(self):

		while not rospy.is_shutdown():

			with open(self.system_response_path, 'r') as fp:
				system_response = json.load(fp)
				system_response_dump = json.dumps(system_response)
				system_response = yaml.safe_load(system_response_dump)
			rospy.logdebug('=== SYSTEM RESPONSE ============')
			rospy.logdebug(system_response)

			system_act = DialogAct()

			system_act.dtype = system_response['d-type']
			[ system_act.slots.append( InformSlot(slot=slot, value=system_response['slots'][slot])) for slot in self.slots ]
			rospy.logdebug(system_act)

			self.pub_belief.publish(system_act)

			self.loop_rate.sleep()



if __name__ == '__main__':

	system_response_node = SystemResponseNode(debug=True)
	system_response_node.begin()
