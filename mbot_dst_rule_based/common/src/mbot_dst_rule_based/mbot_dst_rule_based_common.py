#!/usr/bin/env python

import json
import copy
import os

import numpy as np


import traceback

DEBUG = False

class DialogueStateTracking(object):

	"""
	Class constructor.

	Desciption: Creates an instance of the DialogueStateTracking class.

	Inputs:
		- slots [list]: the possible slots the dialogue can have.
		- initial_belief [dict]: the initial belief of the dialogue.

	Objects:
		- slots: the possible slots the dialogue can have.
		- initial_belief the initial belief of the dialogue.
		- belief: the current belief of the dialogue.
		- belief_not_null: the current belief of the dialogue without slots-value pairs that have zero probability.
	"""
	def __init__(self, slots, initial_belief):

		self.__slots = slots
		self.__initial_belief = initial_belief

		self.__belief = None
		self.__belief_seen_slots = None

		self.initialize_belief()




	"""
	Class method.

	Description: Initializes the belief of the dialogue with the initial belief.
	"""
	def initialize_belief(self):

		self.__belief = copy.deepcopy(self.__initial_belief)
		self.__belief_seen_slots = {key: {} for key in self.__belief.keys()}




	"""
	Class method.

	Description: Splits the dialogue acts.

	Inputs:
		- dialogue_acts: the slot dialogue acts we want to split.

	Outputs:
		- a list of single slot dialogue acts.
	"""
	def split(self, dialogue_acts):
		single_slot_d_acts = []
		for dialogue_act in dialogue_acts:
			single_slot_d_acts.extend(self.__split_dialogue_act(dialogue_act))
		return single_slot_d_acts




	"""

	[NEED TO ENSURE THE DIALOGUE ACTS ARE SINGLE SLOT !!]

	Class method.

	Description: Merge the single slot dialogue acts.

	Inputs:
		- dialogue_acts: single slot dialogue acts.
		- normalize: flag that enables normalization of the dialogue acts probabilities.

	Outputs:
		- the merged dialogue acts.
	"""
	def merge(self, dialogue_acts, normalize=True):
		merged_d_acts = []
		for d_act in dialogue_acts:
			if self.__slot_value_in_dialogue_acts(d_act['slots'], merged_d_acts):
				merged_d_acts = self.__find_transform_probs(merged_d_acts, d_act.copy())
			else:
				merged_d_acts.append(d_act.copy())
		if normalize:
			merged_d_acts = self.__normalize_probs(merged_d_acts)
		return merged_d_acts




	"""
	Class method.

	Description: Splits and merges the dialogue acts.

	Inputs:
		- dialogue_acts: the dialogue acts.

	Outputs:
		- the split-merged dialogue acts.
	"""
	def split_merge(self, dialogue_acts):
		return self.merge(self.split(dialogue_acts))



	"""
	Class method.

	Description: Updates the belief of the tracker, using the dialogue acts of the current turn, and the system response of the last turn.

	Inputs:
		- dialogue_acts: the dialogue acts of the current turn.
		- last_system_response: system response of the previous turn.
	"""
	def update_belief(self, dialogue_acts, last_system_response):
		self.__apply_rules(self.split_merge(dialogue_acts), last_system_response)


	

	"""
	Class private method.

	Description: This function deletes the slot-value pairs on the current belief that have zero probability.


	"""
	def __clean_belief_not_null(self):
		null_slot_value_pairs = ( slot_value_pairsfor for slot_value_pairs in \
			list(self.__belief_seen_slots.items()) if 0.0 in slot_value_pairs.values() )



	"""
	Class private staticmethod.

	Description: This function creates a dialogue act.

	Inputs:
		- d_type: the dialogue act type.
		- slots: a dictionary with slot-value pairs, e.g. {slot1: value1, slot2: value2}
		- probability: the dialogue act probability.

	Outputs:
		- a dictionary representing a dialogue act, with the keys <d-type, slots, probability]
	"""
	@staticmethod
	def __produce_dialogue_act(d_type, slots, probability):
		return {'d-type': d_type,
				'slots': slots,
				'probability': probability }




	"""
	Class private staticmethod.

	Description: Checks if a dialogue act is of the type <d_type>.

	Inputs:
		- dialogue_act: the dialogue act we want to compare.
		- d_type: the <d_type>.

	Outputs:
		- True if the dialogue act if of the type <d_type>.
		- False, otherwise.
	"""
	@staticmethod
	def __dialogue_act_type_is(dialogue_act, d_type):
		return dialogue_act['d-type'] == d_type





	"""
	Class private staticmethod.

	Description: Checks if the value of a slot is <value>.

	Inputs:
		- slot_value: the slot value we want to compare.
		- value: the <value>.

	Outputs:
		- True if the slot has the value <value>.
		- False, otherwise.
	"""
	@staticmethod
	def __slot_value_is(slot_value, value):
		return slot_value == value




	"""
	Class private staticmethod.

	Description: Splits a dialogue act of the type inform, into single slot dialogue acts.
		e.g. inform(slot1=value1, slot2=value2) -> inform(slot1=value1) and inform(slot2=value2)

	Inputs:
		- dialogue_act: the slot dialogue act we want to split.

	Outputs:
		- a list of single slot dialogue acts [only one slot has a value different from <none>].
	"""
	@staticmethod
	def __split_inform_dialogue_act(dialogue_act):
		# get the slots on the dialogue_act whose values are not <none>.
		slots = (slot for slot in dialogue_act['slots'] if not \
			DialogueStateTracking.__slot_value_is(
				dialogue_act['slots'][slot], value='none'
			)
		)
		# for each slot whose value is not <none>, creates a single dialogue act.
		return [ DialogueStateTracking.__produce_dialogue_act(
					d_type=dialogue_act['d-type'], slots={slot: dialogue_act['slots'][slot]}, probability=dialogue_act['probability']
				) for slot in slots ]




	"""
	Class private staticmethod.

	Description: Splits a dialogue act.
		If the dialogue act is of the type inform or deny, splits it into single slots dialogue acts.
		If it is of the type affirm, negate (types that don't have slots values), creates a dialogue act without slots (only the type and probability)

	Inputs:
		- dialogue_act: the slot dialogue act we want to split.

	Outputs:
		- a list of single slot dialogue acts.
	"""
	@staticmethod
	def __split_dialogue_act(dialogue_act):
		if not DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='inform') and not DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='deny'):
			return [ DialogueStateTracking.__produce_dialogue_act(d_type=dialogue_act['d-type'], slots={}, probability=dialogue_act['probability']) ]
		return DialogueStateTracking.__split_inform_dialogue_act(dialogue_act)



	"""
	Class private staticmethod.

	Description: Checks if a there is a slot with the value <slot_value> in the dialogue acts.

	Inputs:
		- slot_value: the <slot_value>.
		- dialogue_acts: the dialogue acts we want to check.

	Outputs:
		- True if there is a slot with value <slot_value>.
		- False, otherwise.
	"""
	@staticmethod
	def __slot_value_in_dialogue_acts(slot_value, dialogue_acts):
		return slot_value in [d_act['slots'] for d_act in dialogue_acts]



	"""
	Class private staticmethod.

	Description: Updates the probabilities of the previous dialogue acts that correspond to the new dialogue act.

	Inputs:
		- dialogue_acts: single slot dialogue acts.
		- new_dialog_act: new single slot dialogue act.

	Outputs:
		- returns the previous dialogue acts with the updated proabilities.
	"""
	@staticmethod
	def __find_transform_probs(dialogue_acts, new_dialogue_act):
		for d_act in dialogue_acts:
			if new_dialogue_act['slots'] == d_act['slots']:
				d_act['probability'] = d_act['probability'] + new_dialogue_act['probability']
		return dialogue_acts



	"""
	Class private staticmethod.

	Description: Normalize the probabilities using exponencial normalization.

	Inputs:
		- x: vector of probabilities.

	Outputs:
		- the normalized probabilities.
	"""
	@staticmethod
	def __exp_normalize(x):
		if len(x) > 1:
			b = x.max()
			y = np.exp(x - b)
			return y / y.sum()
		return x



	"""
	Class private staticmethod.

	Description: Normalize the probabilities using standard normalization.

	Inputs:
		- x: vector of probabilities.

	Outputs:
		- the normalized probabilities.
	"""
	@staticmethod
	def __normalize(x):
		if len(x) > 1:
			return x / x.sum()
		return x



	"""
	Class private staticmethod.

	Description: Normalizes the dialogue acts probabilities.

	Inputs:
		- dialogue_acts: dialogue acts to normalize.

	Outputs:
		- the normalized dialogue acts.
	"""
	@staticmethod
	def __normalize_probs(dialogue_acts):
		norm_probs = DialogueStateTracking.__exp_normalize(
			np.array([ d_act['probability'] for  d_act in dialogue_acts])
		)
		for d_act, prob in zip(dialogue_acts, norm_probs):
			d_act['probability'] = prob
		return dialogue_acts


# ------------------------------------------------------------------------------------------------------------------------> HERE


	"""
	Class private staticmethod.

	Description: Updates the probability of an event.

	Inputs:
		- prev_prob: previous probability of the event.
		- probability of the new occurence of the event.
		- occurence: if the event ocurred.

	Outputs:
		- the updated probability.
	"""
	@staticmethod
	def __update_probability(prev_prob, prob, occurence=True):
		if occurence is True:
			return 1 - (1-prev_prob)*(1-prob)
		return prev_prob*(1-prob)



	"""
	Class private method.

	Description: Update the belief of the dialogue with "inform rule" if the dialogue act is of the type inform.

	Inputs:
		- dialogue_act: the dialogue act we want to use to update the belief.
	"""
	def __apply_inform_rule(self, dialogue_act):
		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='inform'):
			if DEBUG is True: print("Applying inform rule")
			slot = list(dialogue_act['slots'].keys())[0]
			value = dialogue_act['slots'][slot]
			prob = dialogue_act['probability']
			try:
				self.__belief[slot][value] = self.__update_probability(self.__belief[slot][value], prob, occurence=True)
				self.__belief_seen_slots[slot][value] = self.__belief[slot][value]
			except KeyError:
				raise Exception(
					"The value [{}] of the slot [{}] on the dialogue act [{}] is not specified in the initial belief".format(value, slot, dialogue_act)
				)




	"""
	Class private method.

	Description: Update the belief of the dialogue with "deny rule" if the dialogue act is of the type deny.

	Inputs:
		- dialogue_act: the dialogue act we want to use to update the belief.
	"""
	def __apply_deny_rule(self, dialogue_act):
		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='deny'):
			if DEBUG is True: print("Applying deny rule")
			slot = list(dialogue_act['slots'].keys())[0]
			value = dialogue_act['slots'][slot]
			prob = dialogue_act['probability']
			try:
				self.__belief[slot][value] = self.__update_probability(self.__belief[slot][value], prob, occurence=False)
				self.__belief_seen_slots[slot][value] = self.__belief[slot][value]
			except KeyError:
				raise Exception(
					"The value [{}] of the slot [{}] on the dialogue act [{}] is not specified in the initial belief".format(value, slot, dialogue_act)
				)



	"""
	Class private method.

	Description: 

	Inputs:
		- 

	Outputs:
		- 
	"""
	def __apply_affirm_rule(self, dialogue_act, slot_value_pairs):
		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='affirm'):
			if DEBUG is True: print("Applying affirm rule")
			for slot in list(slot_value_pairs.keys()):
				if not self.__slot_value_is(slot_value_pairs[slot], value='none'):
					value = slot_value_pairs[slot]
					prob = dialogue_act['probability']
					try:
						self.__belief[slot][value] = self.__update_probability(self.__belief[slot][value], prob, occurence=True)
						self.__belief_seen_slots[slot][value] = self.__belief[slot][value]
					except KeyError:
						raise Exception(
							"The value [{}] of the slot [{}] on the dialogue act [{}] is not specified in the initial belief".format(value, slot, dialogue_act)
						)




	"""
	Class private method.

	Description: 

	Inputs:
		- 

	Outputs:
		- 
	"""
	def __apply_negate_rule(self, dialogue_act, slot_value_pairs):
		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='negate'):
			if DEBUG is True: print("Applying negate rule")
			for slot in list(slot_value_pairs.keys()):
				if not self.__slot_value_is(slot_value_pairs[slot], value='none'):
					value = slot_value_pairs[slot]
					prob = dialogue_act['probability']
					try:
						self.__belief[slot][value] = self.__update_probability(self.__belief[slot][value], prob, occurence=False)
						self.__belief_seen_slots[slot][value] = self.__belief[slot][value]
					except KeyError:
						raise Exception(
							"The value [{}] of the slot [{}] on the dialogue act [{}] is not specified in the initial belief".format(value, slot, dialogue_act)
						)




	"""
	Class private method.

	Description: 

	Inputs:
		- 

	Outputs:
		- 
	"""
	def __apply_rules(self, d_acts, last_system_response):
		for d_act in d_acts:
			try:
				self.__apply_inform_rule(d_act)
				self.__apply_deny_rule(d_act)
				self.__apply_affirm_rule(d_act, last_system_response['slots'])
				self.__apply_negate_rule(d_act, last_system_response['slots'])
			except Exception:
				if DEBUG is True: print(traceback.format_exc())
				continue




	"""
	Class property.

	Description: 

	Outputs:
		- return the belief of the dialogue without slots with null probability.
	"""
	@property
	def belief(self):
		return self.__belief_seen_slots


	"""
	Class property.

	Description: 
	
	Outputs:
		- return the initial belief.
	"""
	@property
	def initial_belief(self):
		return self.__initial_belief
	

	"""
	Class property.

	Description: 
	
	Outputs:
		- return the possible slots of the dialogue.
	"""
	@property
	def slots(self):
		return self.__slots

if __name__ == '__main__':

	with open('onthology.json', 'r') as fp:
		onthology_dict = json.load(fp)

	slots = ['intent', 'person', 'object', 'source', 'destination']
	#slots = ['intent', 'person', 'object', 'source']

	belief = {slot: {value: 0.0 for value in onthology_dict[slot] } for slot in slots }

	dst_object = DialogueStateTracking(slots=slots, initial_belief=belief)