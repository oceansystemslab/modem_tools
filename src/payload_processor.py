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

import struct
import math
import numpy as np
import collections
import roslib
roslib.load_manifest('modem_tools')
import rospy

import message_config as mc

# Messages
from auv_msgs.msg import NavSts
from vehicle_interface.msg import AcousticModemPayload, PilotRequest, String, AcousticDeconstructionStatus
from eurathlon_msgs.msg import MissionStatus, Command

# TODO: add general message sending

# types of payloads
_POSITION_REQUEST = 'position_request'
_BODY_REQUEST = 'body_request'
_NAV = 'nav'
_MISSION_STS = 'mission_sts'
_MISSION_CMD = 'mission_cmd'
_STRING_IMAGE = 'string_image'
_ACK = 'ack'
_MM_ACK = 'multi_message_ack'
_MM_REQUEST = 'multi_message_request'  # for requesting missing parts of the message
_MULTI_MESSAGE = 'multi_message'
_ROS_MESSAGE = 'ros_message'
_ROS_SERVICE = 'ros_service'

# other strings
HEADER = 'header'
MM_HEADER = 'multi_message_header'
MM_MSG_PART = 'multi_message_part'

# default topics
TOPICS = {
    'modem_incoming':       '/modem/burst/in',
    'modem_outgoing':       '/modem/burst/out',

    'body_incoming':        '/modem/unpacker/body_req',
    'body_outgoing':        '/modem/packer/body_req',
    'position_incoming':    '/modem/unpacker/position_req',
    'position_outgoing':    '/modem/packer/position_req',
    'mission_sts_incoming': '/modem/unpacker/mission_sts',
    'mission_sts_outgoing': '/modem/packer/mission_sts',
    'mission_cmd_incoming': '/modem/unpacker/command',
    'mission_cmd_outgoing': '/modem/packer/command',
    'nav_incoming':         '/modem/unpacker/nav_sts',
    'nav_outgoing':         '/modem/packer/nav_sts',
    'image_string_incoming':'/modem/unpacker/image',
    'image_string_outgoing':'/modem/packer/image',

    'node_status':          '/modem/unpacker/status'
}

# messages which require ack
REQUIRING_ACK = [
    _POSITION_REQUEST,
    _BODY_REQUEST,
    _MISSION_CMD
]

DEFAULT_CONFIG = {
    'topics':           TOPICS,
    'loop_rate':        4,  # Hz
    'requiring_ack':    REQUIRING_ACK,
    'retries':          3,  # if no ack received
    'retry_delay':      60,  # seconds
    'target_address':   5
}


# mapping from user friendly name to compact id (1-255)
TYPE_TO_ID = {
    # fixed messages
    _POSITION_REQUEST:      1,
    _BODY_REQUEST:          2,
    _NAV:                   5,
    _STRING_IMAGE:          10,
    _MISSION_STS:           11,
    _MISSION_CMD:           12,

    # general messages described in the config
    _ACK:                   64,
    _MM_ACK:                65,
    _MM_REQUEST:            66,
    _ROS_MESSAGE:           128,
    _ROS_SERVICE:           129,
    _MULTI_MESSAGE:         130,

}

# TODO: use comprehension dictionaries later (not available in Python 2.6 [Emily boat])
# create inverse dictionary
# ID_TO_TYPE = {value: key for key, value in TYPE_TO_ID.items()}
ID_TO_TYPE = dict((value, key) for key, value in TYPE_TO_ID.items())

# struct formats for encoding/decoding parts of the messages
# types that can be used can be found here:
#   - https://docs.python.org/2/library/struct.html
FORMAT = {
    # protocol specific
    HEADER:                'BHd',  # payload type, msg id, stamp in seconds
    MM_HEADER:             'HBB',  # multi message id, part number, total parts

    # hardcoded ROS messages
    _POSITION_REQUEST:      'ffffff',  # requested pose on 6 axes
    _BODY_REQUEST:          'ffffff',  # requested pose on 6 axes
    _NAV:                   'ddd',  # latitude, longitude, stamp in seconds
    # _MISSION_STS:           'ddfBdB',  # lat, long, yaw, mode, elapsed_time, packed_bools
    _MISSION_STS:           'ffffdB',  # north, east, depth, yaw, elapsed_time, packed_bools
    _MISSION_CMD:           'BB',  # numerical command
    _ACK:                   'H',  # msg id
    _MM_ACK:                'H',  # mm msg id
    _MM_REQUEST:            'HB',  # mm msg id, number of messages, msg ids

    MM_MSG_PART:           'B'  # repeated once for every part
}

# other constants
MAX_FULL_MSG_LEN = 300
MAX_MULTI_MSG_BODY_LEN = MAX_FULL_MSG_LEN - (struct.calcsize(FORMAT[HEADER]) + struct.calcsize(FORMAT[MM_HEADER]))
MULTI_MSG_TIMEOUT = 5  # seconds
MULTI_MSG_RETRY_LIMIT = 3

def deep_update(source_dict, overriding_dict):
    """Update a nested dictionary or similar mapping.

    Modify ``source`` in place.
    """
    for key, value in overriding_dict.iteritems():
        if isinstance(value, collections.Mapping) and value:
            returned = deep_update(source_dict.get(key, {}), value)
            source_dict[key] = returned
        else:
            source_dict[key] = overriding_dict[key]
    return source_dict


class PackerParser(object):
    def __init__(self, name, config, outgoing, incoming):
        self.name = name

        topics = config['topics']
        self.target_address = config['target_address']
        self.requiring_ack = config['requiring_ack']

        self.retry_delay = config['retry_delay']
        self.retries = config['retries']

        self.msg_out_cnt = 0
        self.msg_in_cnt = 0
        self.multi_msg_out_cnt = 0

        self.single_msgs_out = {}
        self.multi_msgs_out = {}
        self.multi_msgs_in = {}
        self.outgoing_msg_buffer = collections.deque()

        self.parse = {
            _POSITION_REQUEST:     self.parse_position_req,
            _BODY_REQUEST:         self.parse_body_req,
            _NAV:                  self.parse_nav,
            _STRING_IMAGE:         self.parse_string,
            _MISSION_STS:          self.parse_mission_sts,
            _MISSION_CMD:          self.parse_mission_cmd,
            _ACK:                  self.parse_ack,
            _MM_ACK:               self.parse_mm_ack,
            _MM_REQUEST:           self.parse_mm_request,
            _MULTI_MESSAGE:        self.parse_multi_message,
        }

        # Publishers
        self.pub_modem = rospy.Publisher(topics['modem_outgoing'], AcousticModemPayload, tcp_nodelay=True, queue_size=1)

        self.pub_nav = rospy.Publisher(topics['nav_incoming'], NavSts, tcp_nodelay=True, queue_size=1)
        self.pub_position = rospy.Publisher(topics['position_incoming'], PilotRequest, tcp_nodelay=True, queue_size=1)
        self.pub_body = rospy.Publisher(topics['body_incoming'], PilotRequest, tcp_nodelay=True, queue_size=1)
        self.pub_string = rospy.Publisher(topics['image_string_incoming'], String, tcp_nodelay=True, queue_size=1)
        # eurathlon specific
        self.pub_mission_sts = rospy.Publisher(topics['mission_sts_incoming'], MissionStatus, tcp_nodelay=True, queue_size=1)
        self.pub_mission_cmd = rospy.Publisher(topics['mission_cmd_incoming'], Command, tcp_nodelay=True, queue_size=1)

        # publishers for incoming general messages (based on the description in config)
        # maps from topic id to publisher
        # self.pub_incoming = {mc.TOPIC_STRING_TO_ID[gm['publish_topic']]:
        #                          rospy.Publisher(gm['publish_topic'], mc.ros_msg_string2type(gm['message_type']), tcp_nodelay=True, queue_size=1) for gm in incoming}

        self.pub_status = rospy.Publisher(topics['node_status'], AcousticDeconstructionStatus, tcp_nodelay=True, queue_size=1)

        # Subscribers
        self.sub_modem = rospy.Subscriber(topics['modem_incoming'], AcousticModemPayload, self.handle_burst_msg, tcp_nodelay=True, queue_size=1)

        self.sub_nav = rospy.Subscriber(topics['nav_outgoing'], NavSts, self.handle_nav, tcp_nodelay=True, queue_size=1)
        self.sub_position = rospy.Subscriber(topics['position_outgoing'], PilotRequest, self.handle_position, tcp_nodelay=True, queue_size=1)
        self.sub_body = rospy.Subscriber(topics['body_outgoing'], PilotRequest, self.handle_body, tcp_nodelay=True, queue_size=1)
        self.sub_string = rospy.Subscriber(topics['image_string_outgoing'], String, self.handle_string, tcp_nodelay=True, queue_size=1)
        # eurathlon specific
        self.sub_mission_sts = rospy.Subscriber(topics['mission_sts_outgoing'], MissionStatus, self.handle_mission_sts, tcp_nodelay=True, queue_size=1)
        self.sub_mission_cmd = rospy.Subscriber(topics['mission_cmd_outgoing'], Command, self.handle_mission_cmd, tcp_nodelay=True, queue_size=1)

        # subscribers for outgoing general messages (based on the description in config)
        # self.sub_outgoing = [rospy.Subscriber(gm['subscribe_topic'],
        #                                       mc.ros_msg_string2type(gm['message_type']),
        #                                       self.parse_general,
        #                                       gm['publish_topic'],
        #                                       tcp_nodelay=True,
        #                                       queue_size=1) for gm in outgoing]

    def handle_nav(self, ros_msg):
        payload_type = _NAV
        payload_body = struct.pack(FORMAT[payload_type],
                                   ros_msg.global_position.latitude, ros_msg.global_position.longitude,
                                   ros_msg.header.stamp.to_sec())
                                   # ros_msg.position.north, ros_msg.position.north, ros_msg.position.north,
                                   # ros_msg.orientation.roll, ros_msg.orientation.pitch, ros_msg.orientation.yaw)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def handle_body(self, ros_msg):
        payload_type = _BODY_REQUEST
        payload_body = struct.pack(FORMAT[payload_type], *ros_msg.position)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def handle_position(self, ros_msg):
        payload_type = _POSITION_REQUEST
        payload_body = struct.pack(FORMAT[payload_type], *ros_msg.position)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def handle_string(self, ros_msg):
        payload_type = _STRING_IMAGE
        payload_body = ros_msg.payload

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def handle_mission_sts(self, ros_msg):
        payload_type = _MISSION_STS

        # fields with geo referenced position
        # bundled_fields = (ros_msg.latitude, ros_msg.longitude, ros_msg.yaw, ros_msg.mode, ros_msg.elapsed_time.to_sec(),
        #                   ros_msg.packed_bools)

        bundled_fields = (ros_msg.north, ros_msg.east, ros_msg.depth, ros_msg.yaw,
                          ros_msg.elapsed_time.to_sec(), ros_msg.packed_bools)
        payload_body = struct.pack(FORMAT[payload_type], *bundled_fields)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def handle_mission_cmd(self, ros_msg):
        payload_type = _MISSION_CMD
        payload_body = struct.pack(FORMAT[payload_type], ros_msg.mode, ros_msg.resume_point)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def add_to_buffer(self, msg_box):
        msg_box.id = self.msg_out_cnt
        self.msg_out_cnt += 1

        if len(msg_box.payload_body) + struct.calcsize(FORMAT[HEADER]) > MAX_FULL_MSG_LEN:
            self.generate_multi_message(msg_box)
            # note that ack will not be expected
            return

        if msg_box.type in self.requiring_ack:
            msg_tracker = mc.SingleMessageTracker(msg_box)
            msg_tracker.update_last_time(rospy.Time.now().to_sec())
            self.single_msgs_out.update({msg_box.id: msg_tracker})

        self.outgoing_msg_buffer.appendleft(msg_box)

    def send_ack(self, msg_id):
        payload_type = _ACK
        payload_body = struct.pack(FORMAT[payload_type], msg_id)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def send_mm_ack(self, mm_msg_id):
        payload_type = _MM_ACK
        payload_body = struct.pack(FORMAT[payload_type], mm_msg_id)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def send_mm_request(self, mm_msg_id):
        payload_type = _MM_REQUEST

        missed_parts = []
        for part, box in enumerate(self.multi_msgs_in[mm_msg_id].boxes):
            if box is None:
                missed_parts.append(part)
        missed_parts = self.multi_msgs_in[mm_msg_id].get_empty_slots_indices()

        rospy.logwarn('%s: Requesting resending of parts: %s of multi message with id %s' % (self.name, missed_parts, mm_msg_id))

        payload_body = struct.pack(FORMAT[payload_type], mm_msg_id, len(missed_parts)) +\
                       struct.pack(str(len(missed_parts)) + FORMAT[MM_MSG_PART], *missed_parts)

        msg_box = mc.MessageContainer(payload_type, self.target_address, payload_body)

        self.add_to_buffer(msg_box)

    def generate_multi_message(self, msg_box):
        payload_type = _MULTI_MESSAGE

        header = struct.pack(FORMAT[HEADER], TYPE_TO_ID[msg_box.type], msg_box.id, rospy.Time.now().to_sec())
        content = '{0}{1}'.format(header, msg_box.payload_body)

        total_parts = int(math.ceil(len(content) / MAX_MULTI_MSG_BODY_LEN))
        mean_length = int(math.ceil(len(content)/total_parts))

        multi_msg = mc.MultiMessageTracker(total_parts)

        for i in range(total_parts):
            multi_msg_header = struct.pack(FORMAT[MM_HEADER], self.multi_msg_out_cnt, i, total_parts)
            content_section = content[i*mean_length: (i+1)*mean_length]

            msg_box = mc.MessageContainer(payload_type, self.target_address, content_section)
            msg_box.id = self.msg_out_cnt

            msg_box.payload_body = '{0}{1}'.format(multi_msg_header, content_section)

            multi_msg.add_part(i, msg_box)
            self.msg_out_cnt += 1

        self.multi_msgs_out.update({self.multi_msg_out_cnt: multi_msg})
        self.multi_msg_out_cnt += 1

        # to induce failure in sending first part do:
        # for box in multi_msg.boxes[1:]:
        for box in multi_msg.boxes:
            self.outgoing_msg_buffer.appendleft(box)

    def handle_burst_msg(self, msg):
        self.parse_top_level(msg.payload, msg.address)

    def parse_top_level(self, payload, address):
        header = payload[:struct.calcsize(FORMAT[HEADER])]
        body = payload[struct.calcsize(FORMAT[HEADER]):]

        header_values = struct.unpack(FORMAT[HEADER], header)

        payload_type = ID_TO_TYPE[header_values[0]]
        msg_id = header_values[1]
        time_packed = header_values[2]
        time_unpacked = rospy.Time.now().to_sec()

        self.msg_in_cnt += 1

        ads = AcousticDeconstructionStatus()
        ads.header.stamp = rospy.Time.now()
        ads.time_packed = time_packed
        ads.time_unpacked = time_unpacked
        ads.length = len(payload)

        # ads.message = payload
        # ns.info = [KeyValue(key, str(value)) for key, value in info.items()]
        self.pub_status.publish(ads)

        self.parse.get(payload_type, self.parse_unknown)(payload_type, msg_id, time_packed, body, address)
        rospy.loginfo('%s: Received message of type %s with id %s from %s' % (self.name, payload_type, msg_id, address))

        if payload_type in self.requiring_ack:
            self.send_ack(msg_id)

    def parse_nav(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)

        nav_msg = NavSts()
        nav_msg.global_position.latitude, nav_msg.global_position.longitude = values[0:2]
        nav_msg.header.stamp = rospy.Time.from_sec(values[2])
        # nav_msg.position.north, nav_msg.position.east, nav_msg.position.depth = values[2:5]
        # nav_msg.orientation.roll, nav_msg.orientation.pitch, nav_msg.orientation.yaw = values[5:8]

        self.pub_nav.publish(nav_msg)

    def parse_position_req(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)

        pilot_msg = PilotRequest()
        pilot_msg.header.stamp = rospy.Time.from_sec(dispatch_time)
        pilot_msg.position = list(values[0:6])

        self.pub_position.publish(pilot_msg)

    def parse_body_req(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)

        pilot_msg = PilotRequest()
        pilot_msg.header.stamp = rospy.Time.from_sec(dispatch_time)
        pilot_msg.position = list(values[0:6])

        self.pub_body.publish(pilot_msg)

    def parse_string(self, payload_type, id, dispatch_time, body, origin_address):
        msg = String()
        msg.header.stamp = rospy.Time.from_sec(dispatch_time)
        msg.payload = body
        self.pub_string.publish(msg)

    def parse_mission_sts(self, payload_type, id, dispatch_time, body, origin_address):
        NUMBER_OF_PACKED_BOOLS = 5

        print len(body)

        values = struct.unpack(FORMAT[payload_type], body)

        msg = MissionStatus()
        msg.header.stamp = rospy.Time.from_sec(dispatch_time)

        msg.north, msg.east, msg.depth,\
            msg.yaw, elapsed_time_seconds, msg.packed_bools = values

        msg.elapsed_time = rospy.Time.from_sec(elapsed_time_seconds)

        binary_string = '{0:b}'.format(msg.packed_bools)
        binary_chars = list(binary_string.zfill(NUMBER_OF_PACKED_BOOLS))
        binary_chars.reverse()
        bool_array = np.array(binary_chars).astype('bool')

        msg.gate_found = bool_array[0]
        msg.opi_found = bool_array[1]
        msg.plume_reached = bool_array[2]
        msg.pipe_reached = bool_array[3]
        msg.valve_found = bool_array[4]
        # msg.home_reached = bool_array[5]

        self.pub_mission_sts.publish(msg)

    def parse_mission_cmd(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)

        msg = Command()
        msg.header.stamp = rospy.Time.from_sec(dispatch_time)
        msg.mode = values[0]
        msg.resume_point = values[1]

        self.pub_mission_cmd.publish(msg)

    def parse_ack(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)
        msg_id = values[0]
        rospy.loginfo('%s: Message with id %s was delivered' % (self.name, msg_id))
        if msg_id in self.single_msgs_out.keys():
            self.single_msgs_out.pop(msg_id)

    def parse_mm_ack(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body)
        mm_msg_id = values[0]
        rospy.loginfo('%s: Multi message with id %s was delivered' % (self.name, mm_msg_id))
        self.multi_msgs_out.pop(mm_msg_id)

    def parse_mm_request(self, payload_type, id, dispatch_time, body, origin_address):
        values = struct.unpack(FORMAT[payload_type], body[:struct.calcsize(FORMAT[payload_type])])
        mm_msg_id, parts_amount = values

        # list of all parts that where not received
        parts = list(struct.unpack(str(parts_amount)+FORMAT[MM_MSG_PART], body[struct.calcsize(FORMAT[payload_type]):]))

        for part in parts:
            box = self.multi_msgs_out[mm_msg_id].boxes[part]
            self.outgoing_msg_buffer.appendleft(box)

    def parse_multi_message(self, payload_type, id, dispatch_time, body, origin_address):
        multi_msg_header = struct.unpack(FORMAT[MM_HEADER], body[:struct.calcsize(FORMAT[MM_HEADER])])
        content = body[struct.calcsize(FORMAT[MM_HEADER]):]
        multi_msg_id, part, total_parts = multi_msg_header

        if multi_msg_id not in self.multi_msgs_in.keys():
            multi_msg = mc.MultiMessageTracker(total_parts)
            multi_msg.address = origin_address
            self.multi_msgs_in.update({multi_msg_id: multi_msg})

        msg_box = mc.MessageContainer(payload_type, origin_address, content)
        msg_box.id = id

        self.multi_msgs_in[multi_msg_id].add_part(part, msg_box)
        self.multi_msgs_in[multi_msg_id].t_last_action = rospy.Time.now().to_sec()

        self.check_mm_completeness()

    def check_mm_completeness(self):
        for mm_id, multi_msg in self.multi_msgs_in.items():
            if multi_msg.is_complete():
                assembled_payload = multi_msg.combine()
                self.parse_top_level(assembled_payload, multi_msg.address)
                self.multi_msgs_in.pop(mm_id)
                self.send_mm_ack(mm_id)

    def parse_unknown(self, payload_type, id, dispatch_time, body):
        rospy.logwarn('%s: Message of unknown type %s with id %s was delivered' % (self.name, payload_type, id))

    def parse_general(self, payload_type, id, dispatch_time, body):
        pass

    def check_multi_msg_request_timeout(self):
        time_now = rospy.Time.now().to_sec()

        # for partially received multi messages
        for mm_msg_id, multi_message in self.multi_msgs_in.items():
            if time_now - multi_message.t_last_action > MULTI_MSG_TIMEOUT:
                # increase the retry counter
                multi_message.retries += 1

                if multi_message.retries > MULTI_MSG_RETRY_LIMIT:
                    rospy.logwarn('%s: Failed to obtain full multi message with id %s, forgetting the parts!' % (self.name, mm_msg_id))
                    self.multi_msgs_in.pop(mm_msg_id)
                else:
                    # send mm request
                    self.send_mm_request(mm_msg_id)
                    multi_message.t_last_action = rospy.Time.now().to_sec()

    def check_single_msg_resend_timeout(self):
        time_now = rospy.Time.now().to_sec()

        for msg_id, msg_tracker in self.single_msgs_out.items():
            if time_now - msg_tracker.t_last_action > self.retry_delay:
                # increase the retry counter
                msg_tracker.retries += 1

                if msg_tracker.retries > self.retries:
                    rospy.logwarn('%s: Failed to deliver message with id %s after %s retries. Giving up!'
                                  % (self.name, msg_id, self.retries))
                    self.single_msgs_out.pop(msg_id)
                else:
                    # send mm request
                    msg_tracker.t_last_action = rospy.Time.now().to_sec()
                    self.outgoing_msg_buffer.appendleft(msg_tracker.msg_box)

    def send_from_buffer(self):
        if len(self.outgoing_msg_buffer) == 0:
            return

        msg_box = self.outgoing_msg_buffer.pop()

        header = struct.pack(FORMAT[HEADER], TYPE_TO_ID[msg_box.type], msg_box.id, rospy.Time.now().to_sec())
        payload = '{0}{1}'.format(header, msg_box.payload_body)

        modem_msg = AcousticModemPayload()
        modem_msg.header.stamp = rospy.Time.now()
        modem_msg.address = msg_box.address

        modem_msg.payload = payload

        if msg_box.id in self.single_msgs_out.keys():
            self.single_msgs_out[msg_box.id].update_last_time(rospy.Time.now().to_sec())

        rospy.loginfo('%s: Sending message of type %s with id %s to %s' % (self.name, msg_box.type, msg_box.id, msg_box.address))
        self.pub_modem.publish(modem_msg)

    def loop(self):
        self.send_from_buffer()
        self.check_multi_msg_request_timeout()
        self.check_single_msg_resend_timeout()

if __name__ == '__main__':
    rospy.init_node('packer_parser')
    name = rospy.get_name()

    config = DEFAULT_CONFIG.copy()
    # load global parameters
    param_config = rospy.get_param('~packer_config', {})

    # Update default settings with user specified params
    # config.update(param_config)
    deep_update(config, param_config)

    general_outgoing = rospy.get_param('~general_messages_outgoing', {})
    general_incoming = rospy.get_param('~general_messages_incoming', {})

    rospy.loginfo('%s: Loaded config is: %s' % (name, config))
    rospy.loginfo('%s: Outgoing messages are: %s' % (name, general_outgoing))
    rospy.loginfo('%s: Incoming messages are: %s' % (name, general_incoming))

    pp = PackerParser(name, config, general_outgoing, general_incoming)
    loop_rate = rospy.Rate(config['loop_rate'])

    while not rospy.is_shutdown():
        try:
            pp.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('%s caught exception and dying!', name)
        #     sys.exit(-1)



