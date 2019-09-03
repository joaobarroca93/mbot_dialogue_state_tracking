#!/usr/bin/env python

import json
import copy

import rospy
import traceback


class Slot(object):

	def __init__(self, type, value=None, confidence=0.0):

		self.type = type
		self.value = value
		self.confidence = confidence

	def __eq__(self, other):
		if not isinstance(other, Slot):
			return NotImplemented

		return self.value == other.value and self.type == other.type

	def as_dict(self):
		return {
			"type": self.type,
			"value": self.value,
			"confidence": self.confidence
		}


class DialogueAct(object):

	def __init__(self, dtype=None, slots=[], confidence=0.0):

		self.dtype = dtype
		self.slots = slots
		self.confidence = confidence

	def as_dict(self):
		return {
			"dtype": self.dtype,
			"confidence": self.confidence,
			"slots": [
				{
					"type": slot.type,
					"value": slot.value,
					"confidence": slot.confidence
				}
				for slot in self.slots]
		}


class Belief(object):

	def __init__(self, slots=[]):

		self.slots = slots

	def as_dict(self):
		return {
			"belief": [
				{
					"type": slot.type,
					"value": slot.value,
					"confidence": slot.confidence
				}
				for slot in self.slots]
		}


class DialogueStateTracking(object):

	def __init__(self, restart_threshold=0.1):

		rospy.logdebug("Initializing DialogueStateTracking object")
		self.__initial_belief = Belief()
		self.__belief = None
		self.initialize_belief()

		self.restart_threshold = restart_threshold

	# CHECKED
	def split(self, dialogue_acts):

		rospy.logdebug("Spliting dialogue acts")
		rospy.logdebug([dialogue_act.as_dict() for dialogue_act in dialogue_acts])

		single_slot_d_acts = []
		for dialogue_act in dialogue_acts:
			single_slot_d_acts.extend(self.__split_dialogue_act(dialogue_act))

		rospy.logdebug("Single slot dialogue acts")
		rospy.logdebug([dialogue_act.as_dict() for dialogue_act in single_slot_d_acts])

		return single_slot_d_acts

	# CHECKED
	def merge(self, dialogue_acts, normalize=False):

		rospy.logdebug("Merging single slot dialogue acts")
		rospy.logdebug([dialogue_act.as_dict() for dialogue_act in dialogue_acts])

		merged_d_acts = []
		for d_act in dialogue_acts:
			if self.__slot_value_in_dialogue_acts(d_act, merged_d_acts):
				merged_d_acts = self.__find_transform_probs_slots(copy.deepcopy(d_act), merged_d_acts)
			elif self.__affirm_negate_in_dialogue_acts(d_act, merged_d_acts):
				merged_d_acts = self.__find_transform_probs_dtype(copy.deepcopy(d_act), merged_d_acts)
			else:
				merged_d_acts.append(copy.deepcopy(d_act))

		if normalize:
			#rospy.logdebug("Normalizing probabilities")
			raise NotImplemented("NORMALIZE NOT IMPLEMENTED!")
			#merged_d_acts = self.__normalize_probs(merged_d_acts)

		rospy.logdebug("Merged dialogue acts")
		rospy.logdebug([dialogue_act.as_dict() for dialogue_act in merged_d_acts])

		return merged_d_acts

	# CHECKED
	def split_merge(self, dialogue_acts, normalize=False):
		return self.merge(self.split(dialogue_acts), normalize=normalize)

	def update_belief(self, dialogue_acts, last_system_response, normalize=False):
		self.__apply_rules(self.split_merge(dialogue_acts, normalize=normalize), last_system_response)

	def __apply_rules(self, d_acts, last_system_response):
		for d_act in d_acts:
			try:
				self.__apply_inform_rule(d_act)
				self.__apply_affirm_rule(d_act, last_system_response)
				self.__apply_negate_rule(d_act, last_system_response)
				self.__apply_restart_rule(d_act)
			except AssertionError:
				rospy.logwarn("Error applying rules to {}".format(d_act))
				rospy.logdebug(traceback.format_exc())
				continue
			except Exception:
				rospy.logdebug("Dialogue State Restarted!")
				break

	def __apply_inform_rule(self, dialogue_act):

		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type="inform"):

			rospy.logdebug("Applying inform rule to {}".format(dialogue_act.as_dict()))
			assert (len(dialogue_act.slots) == 1)

			slot = dialogue_act.slots[0]
			confidence = slot.confidence
			found = False
			for slot_in_belief in self.__belief.slots:
				if slot == slot_in_belief:
					slot_in_belief.confidence = self.__update_probability(
						slot_in_belief.confidence, confidence, occurence=True
					)
					found = True
					break

			if not found:
				self.__belief.slots.append(copy.deepcopy(slot))

	def __apply_affirm_rule(self, dialogue_act, system_response_act):

		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='affirm'):

			rospy.logdebug("Applying affirm rule to {}".format(dialogue_act.as_dict()))
			assert system_response_act
			assert system_response_act.slots

			confidence = dialogue_act.confidence
			for slot in system_response_act.slots:
				for slot_in_belief in self.__belief.slots:
					if slot == slot_in_belief:
						slot_in_belief.confidence = self.__update_probability(
							slot_in_belief.confidence, confidence, occurence=True
						)

	def __apply_negate_rule(self, dialogue_act, system_response_act):

		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='negate'):

			rospy.logdebug("Applying negate rule to {}".format(dialogue_act.as_dict()))
			assert system_response_act
			assert system_response_act.slots

			confidence = dialogue_act.confidence
			for slot in system_response_act.slots:
				for slot_in_belief in self.__belief.slots:
					if slot == slot_in_belief:
						slot_in_belief.confidence = self.__update_probability(
							slot_in_belief.confidence, confidence, occurence=False
						)

	def __apply_restart_rule(self, dialogue_act):

		if DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='restart'):
			if dialogue_act.confidence >= self.restart_threshold:
				rospy.logdebug("Applying restart rule to {}".format(dialogue_act.as_dict()))
				self.initialize_belief()
				raise Exception("Restarting Belief")

	# CHECKED
	def initialize_belief(self):
		rospy.logdebug("Initializing belief")
		self.__belief = copy.deepcopy(self.__initial_belief)

	# CHECKED
	@staticmethod
	def __produce_dialogue_act(dtype=None, slots=[], confidence=None):
		return DialogueAct(dtype=dtype, slots=slots, confidence=confidence)

	# CHECKED
	@staticmethod
	def __dialogue_act_type_is(dialogue_act, d_type):
		return dialogue_act.dtype == d_type

	# CHECKED
	@staticmethod
	def __slot_value_is(slot, value):
		return slot.value == value

	# CHECKED
	@staticmethod
	def __split_inform_dialogue_act(dialogue_act):
		return [DialogueStateTracking.__produce_dialogue_act(
			dtype=dialogue_act.dtype,
			slots=[slot],
			confidence=dialogue_act.confidence
		) for slot in dialogue_act.slots if slot.value != "none"]

	# CHECKED
	@staticmethod
	def __split_dialogue_act(dialogue_act):
		if not DialogueStateTracking.__dialogue_act_type_is(dialogue_act, d_type='inform'):
			return [ DialogueStateTracking.__produce_dialogue_act(
				dtype=dialogue_act.dtype,
				slots=[],
				confidence=dialogue_act.confidence
			)]
		return DialogueStateTracking.__split_inform_dialogue_act(dialogue_act)

	# CHECKED
	@staticmethod
	def __slot_value_in_dialogue_acts(d_act, dialogue_acts):
		if dialogue_acts:
			for slot in d_act.slots:
				for dialogue_act in dialogue_acts:
					assert (len(dialogue_act.slots) <= 1)
					if dialogue_act.slots:
						slot_i = dialogue_act.slots[0]
						if slot == slot_i:
							return True
		return False

	@staticmethod
	def __affirm_negate_in_dialogue_acts(d_act, dialogue_acts):
		if dialogue_acts and d_act.dtype != "inform":
			for dialogue_act in dialogue_acts:
				if d_act.dtype == dialogue_act.dtype:
					return True
		return False

	@staticmethod
	def __find_transform_probs_dtype(new_dialogue_act, dialogue_acts):
		for d_act in dialogue_acts:
			if new_dialogue_act.dtype == d_act.dtype:
				d_act.confidence = d_act.confidence + new_dialogue_act.confidence
		return dialogue_acts

	# CHECKED
	@staticmethod
	def __find_transform_probs_slots(new_dialogue_act, dialogue_acts):
		assert (len(new_dialogue_act.slots) <= 1)
		for d_act in dialogue_acts:
			assert (len(d_act.slots) <= 1)
			if new_dialogue_act.slots and d_act.slots:
				if new_dialogue_act.slots[0] == d_act.slots[0]:
					d_act.slots[0].confidence = d_act.slots[0].confidence + new_dialogue_act.slots[0].confidence
		return dialogue_acts

	@staticmethod
	def __update_probability(prev_prob, prob, occurence=True):
		if occurence is True:
			return 1 - (1-prev_prob)*(1-prob)
		return prev_prob*(1-prob)

	@property
	def belief(self):
		return self.__belief


if __name__ == '__main__':

	dst_object = DialogueStateTracking()

	num_d_acts = 4
	prob = 1/num_d_acts
	dialogue_acts = [
		DialogueAct(
			dtype="inform",
			slots=[
				Slot(type="intent", value="take", confidence=0.991*prob),
				Slot(type="object", value="book", confidence=0.982*prob)
			],
			confidence=0.997*prob
		),
		DialogueAct(
			dtype="inform",
			slots=[
				Slot(type="intent", value="take", confidence=0.996*prob),
				Slot(type="object", value="book", confidence=0.982 * prob),
				Slot(type="source", value="table", confidence=0.962*prob)
			],
			confidence=0.997*prob
		),
		DialogueAct(
			dtype="inform",
			slots=[
				Slot(type="intent", value="take", confidence=0.996 * prob),
				Slot(type="object", value="book", confidence=0.982 * prob),
			],
			confidence=0.997 * prob
		),
		DialogueAct(
			dtype="inform",
			slots=[
				Slot(type="intent", value="take", confidence=0.996 * prob),
			],
			confidence=0.997 * prob
		)
	]

	system_response = DialogueAct(
		dtype="hello",
		slots=[]
	)

	dst_object.update_belief(dialogue_acts, last_system_response=system_response)
	print(json.dumps(dst_object.belief.as_dict(), indent=4))

	num_d_acts = 4
	prob = 1 / num_d_acts
	dialogue_acts = [
		DialogueAct(
			dtype="affirm",
			slots=[],
			confidence=0.997 * prob
		),
		DialogueAct(
			dtype="affirm",
			slots=[],
			confidence=0.997 * prob
		),
		DialogueAct(
			dtype="inform",
			slots=[],
			confidence=0.997 * prob
		),
		DialogueAct(
			dtype="inform",
			slots=[],
			confidence=0.997 * prob
		)
	]

	system_response = DialogueAct(
		dtype="confirm",
		slots=[
			Slot(type="intent", value="take"),
			Slot(type="object", value="book"),
			Slot(type="source", value="table")
		])

	dst_object.update_belief(dialogue_acts, last_system_response=system_response)
	print(json.dumps(dst_object.belief.as_dict(), indent=4))

