#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, Ocean Systems Laboratory, Heriot-Watt University, UK.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Heriot-Watt University nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Original authors:
#   Valerio De Carolis, Marian Andrecki, Corina Barbalata, Gordon Frost
from __future__ import division

import re
import roslib
roslib.load_manifest('modem_tools')

# Messages
from std_msgs.msg import Header
from auv_msgs.msg import NavSts
from vehicle_interface.msg import PilotRequest, String

# Add a desired topic name here (1-255)
TOPICS_ID_TO_STRING = {
    1:      '/modem/body_req',
    2:      '/pilot/body_req',
    5:      '/modem/position_req',
    6:      '/pilot/position_req',
    10:     '/modem/nav_sts',
    11:     '/nav/nav_sts',
}

# TOPIC_STRING_TO_ID = {value: key for key, value in TOPICS_ID_TO_STRING.items()}
TOPIC_STRING_TO_ID = dict((value, key) for key, value in TOPICS_ID_TO_STRING.items())

# Add messages types here and in the set below (1-255)
ROS_MSG_ID_TO_TYPE = {
    1:      PilotRequest,
    # 2:      NavSts,  # has too many sub types for now
    3:      String,
}

# TODO: use comprehension dictionaries later (not available in Python 2.6 [Emily boat])
# ROS_MSG_TYPE_TO_ID = {value: key for key, value in ROS_MSG_ID_TO_TYPE.items()}
ROS_MSG_TYPE_TO_ID = dict((value, key) for key, value in ROS_MSG_ID_TO_TYPE.items())

# all types of messages that can be converted to structs
# ROS_MSG_TYPES = {Header}  # additional message types
ROS_MSG_TYPES = set()
ROS_MSG_TYPES.add(Header)
ROS_MSG_TYPES = ROS_MSG_TYPES.union(set(ROS_MSG_ID_TO_TYPE.values()))  # union with already defined ones


def ros_msg_string2type(msg_name):
    for msg_type in ROS_MSG_TYPES:
        if msg_type.__name__ == msg_name:
            return msg_type
    raise KeyError('%s message is not described in message_config.py' % msg_name)

PRIMITIVES = {
    'bool':                 '?',
    'uint8':                'B',
    'uint16':               'H',
    'uint32':               'I',
    'uint64':               'L',
    'int8':                 'b',
    'int16':                'h',
    'int32':                'i',
    'int64':                'l',
    'float32':              'f',
    'float64':              'd',
    'string':               's',
}


class MessageContainer(object):
    def __init__(self, msg_type, address, payload_body):
        # origin or target depending on the context
        self.address = address
        self.type = msg_type
        self.payload_body = payload_body
        self.id = None

MULTI_MSG = 'multi_msg'
SINGLE_MSG = 'single_msg'


class Tracker(object):
    def __init__(self):
        # when was the message sent last time?
        self.t_last_action = None
        # how many times has it been sent already?
        self.retries = 0

    def inc_retries(self):
        self.retries += 1

    def get_retries(self):
        return self.retries

    def update_last_time(self, time):
        self.t_last_action = time

    def get_last_time(self):
        return self.t_last_action


class SingleMessageTracker(Tracker):
    def __init__(self, msg_box):
        self.msg_box = msg_box
        # self.payload = payload

        super(SingleMessageTracker, self).__init__()


class MultiMessageTracker(Tracker):
    def __init__(self, number_of_parts):
        self.origin_address = -1
        self.boxes = [None for i in range(number_of_parts)]

        super(MultiMessageTracker, self).__init__()

    def add_part(self, index, msg_box):
        self.boxes[index] = msg_box

    def get_number_of_parts(self):
        return len(self.boxes)

    def get_part(self, index):
        return self.boxes[index]

    def set_address(self, address):
        self.origin_address = address

    def get_address(self):
        return self.origin_address

    def get_empty_slots_indices(self):
        empty_slots = []
        for part, content in enumerate(self.boxes):
            if content is None:
                empty_slots.append(part)

        return empty_slots

    def is_complete(self):
        # true if no empty fields in the list
        return all(self.boxes)

    def combine(self):
        combined = ''
        for box in self.boxes:
            combined += box.payload_body
        return combined


# TODO: General messages in progress
#
# DYNAMIC_TYPES = set('string')
#
# FORMAT_UINT16 = 'H'
#
# BRACKET_OPEN = '['
# BRACKET_CLOSE = ']'
#
# DYNAMIC_LENGTH = -1
#
# def get_value_in_brackets(slot):
#     """
#
#     :param slot: a string describing the ROS type of the slot
#     :return: an integer indicating number of elements in the slot and bool describing whether the list is dynamic
#     """
#     s = re.findall(r'\[[0-9]*\]', slot)
#     s = s.lstrip(BRACKET_OPEN)
#     s = s.rstrip(BRACKET_CLOSE)
#     if len(s) > 0:
#         return int(s), False
#     else:
#         return -1, True
#
# class GeneralMessage(object):
#     def __init__(self, msg):
#         self.msg = msg
#         self.type = type(msg)
#         self.type_str = msg._type
#         self.slots = self.type._get_types(msg)
#         self.format = ''
#         self.values = []
#
#     def get_format_and_values(self):
#         for slot in self.slots:
#             value = getattr(self.type, slot)
#
#             # check if the slot is a list and determine its length
#             if BRACKET_OPEN in slot:
#                 length, dynamic = get_value_in_brackets(slot)
#                 slot_type = slot.split(BRACKET_OPEN)[0]
#
#             elif slot in DYNAMIC_TYPES:
#                 length = -1
#                 dynamic = True
#                 slot_type = slot
#
#             else:
#                 length = 1
#                 dynamic = False
#                 slot_type = slot
#
#             # precede dynamic list with length of the list
#             if dynamic is True:
#                 length = len(value)
#                 self.format += FORMAT_UINT16
#                 self.values.append(length)
#
#             # add values from the slot
#             if slot_type in PRIMITIVES.keys():
#                 self.format += length*PRIMITIVES[slot_type]
#                 if type(value) is list:
#                     self.values.extend(value)
#                 else    :
#                     self.values.append(value)
#                 else:
#                     pass







