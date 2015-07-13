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
import roslib
roslib.load_manifest('modem_tools')

import rospy

# Messages
from auv_msgs.msg import NavSts
from vehicle_interface.msg import AcousticModemPayload, PilotRequest, String

# TODO: add splitting of messages
# TODO: add storage of messages waiting for ack
# TODO: add retrying after some time

# Constants
TOPICS = {
    'modem_incoming':       '/modem/burst/out',
    'modem_outgoing':       '/modem/burst/out',

    'body_incoming':        '/modem/unpacker/body_req',
    'body_outgoing':        '/modem/packer/body_req',
    'position_incoming':    '/modem/unpacker/position_req',
    'position_outgoing':    '/modem/packer/position_req',
    'nav_incoming':         '/modem/unpacker/nav_sts/',
    'nav_outgoing':         '/modem/packer/nav_sts/',
    'image_string_incoming':'/modem/unpacker/image',
    'image_string_outgoing':'/modem/packer/image',
}

# messages which require ack
REQUIRING_ACK = [
    'body_request',
    'position_request'
]

DEFAULT_CONFIG = {
    'topics':           TOPICS,
    'loop_rate':        1,
    'requiring_ack':    REQUIRING_ACK,
    'retries':          3,  # if no ack received
    'retry_delay':      30,  # seconds
    'target_address':   5
}

LOOP_RATE = 1  # Hz

# mapping from user friendly name to compact id (1-255)
TYPE_TO_ID = {
    'position_request':     1,
    'body_request':         2,
    'nav':                  5,
    'image':                10,
    'ack':                  255
}

# create inverse dictionary
ID_TO_TYPE = {value: key for key, value in TYPE_TO_ID.items()}

# struct formats for encoding/decoding parts of the payload with structs
FORMAT = {
    # protocol specific
    'header':               'BH',  # payload type, msg id,

    # hardcoded ROS messages
    'position_request':     'ffffff',  # requested pose on 6 axes
    'body_request':         'ffffff',  # requested pose on 6 axes
    'nav':                  'ddffffff',  # latitude, longitude, *pose
    'custom':               'HH',  # element number, total elements
    'ack':                  'H',  # msg id

    # primitives
    'uint8':                'B',
    'uint16':               'H',
    'double':               'd',
}


TARGET_ADDRESS = 1

class PackerParser(object):
    def __init__(self, name, config):
        self.name = name

        topics = config['topics']

        self.target_address = config['target_address']
        self.header_length = struct.calcsize(FORMAT['header'])
        self.msg_cnt = 0

        self.parse = {
            'position_request':     self.parse_position_req,
            'body_request':         self.parse_body_req,
            'nav':                  self.parse_nav,
            'custom':               self.parse_string,
            'ack':                  self.parse_ack
        }

        # Publishers
        self.pub_modem = rospy.Publisher(topics['modem_incoming'], AcousticModemPayload)

        self.pub_nav = rospy.Publisher(topics['nav_incoming'], NavSts)
        self.pub_position = rospy.Publisher(topics['position_incoming'], PilotRequest)
        self.pub_body = rospy.Publisher(topics['body_incoming'], PilotRequest)
        self.pub_string = rospy.Publisher(topics['image_string_incoming'], String)

        # Subscribers
        self.sub_modem = rospy.Subscriber(topics['modem_outgoing'], AcousticModemPayload, self.handle_burst_msg)

        self.sub_nav = rospy.Subscriber(topics['nav_outgoing'], NavSts, self.handle_nav)
        self.sub_position = rospy.Subscriber(topics['position_outgoing'], PilotRequest, self.handle_position)
        self.sub_body = rospy.Subscriber(topics['body_outgoing'], PilotRequest, self.handle_body)
        self.sub_string = rospy.Subscriber(topics['image_string_outgoing'], String, self.handle_string)

    def handle_nav(self, ros_msg):
        payload_type = 'nav'
        payload_body = struct.pack(FORMAT[payload_type],
                                   ros_msg.global_position.latitude, ros_msg.global_position.longitude,
                                   ros_msg.position.north, ros_msg.position.north, ros_msg.position.north,
                                   ros_msg.orientation.roll, ros_msg.orientation.pitch, ros_msg.orientation.yaw)
        self.send_message(payload_type, payload_body)

    def handle_body(self, ros_msg):
        payload_type = 'body_request'
        payload_body = struct.pack(FORMAT[payload_type], *ros_msg.position)
        self.send_message(payload_type, payload_body)

    def handle_position(self, ros_msg):
        payload_type = 'position_request'
        payload_body = struct.pack(FORMAT[payload_type], *ros_msg.position)
        self.send_message(payload_type, payload_body)

    def handle_string(self, msg):
        pass

    def send_message(self, payload_type, payload_body):
        header = struct.pack(FORMAT['header'], TYPE_TO_ID[payload_type], self.msg_cnt)

        payload = '{0}{1}'.format(header, payload_body)
        rospy.loginfo('%s: Sending message of type %s with id %s to %s' % (self.name, payload_type, self.msg_cnt, self.target_address))
        # rospy.loginfo('%s: Message payload: %s' % (self.name, repr(payload)))
        self.msg_cnt += 1

        modem_msg = AcousticModemPayload()
        modem_msg.header.stamp = rospy.Time.now()
        modem_msg.address = self.target_address
        modem_msg.payload = payload

        self.pub_modem.publish(modem_msg)

    def send_ack(self, msg_id):
        payload_type = 'ack'
        payload_body = struct.pack(FORMAT[payload_type], msg_id)
        self.send_message(payload_type, payload_body)

    def handle_burst_msg(self, msg):
        header = msg.payload[:self.header_length]
        body = msg.payload[self.header_length:]

        header_values = struct.unpack(FORMAT['header'], header)

        payload_type = ID_TO_TYPE[header_values[0]]
        msg_id = header_values[1]

        self.parse.get(payload_type, self.parse_unknown)(payload_type, msg_id, body)
        rospy.loginfo('%s: Received message of type %s with id %s from %s' % (self.name, payload_type, msg_id, msg.address))

        if payload_type in REQUIRING_ACK:
            self.send_ack(msg_id)

    def parse_nav(self, payload_type, id, body):
        values = struct.unpack(FORMAT[payload_type], body)

        nav_msg = NavSts()
        nav_msg.global_position.latitude, nav_msg.global_position.longitude = values[0:2]
        nav_msg.position.north, nav_msg.position.east, nav_msg.position.depth = values[2:5]
        nav_msg.orientation.roll, nav_msg.orientation.pitch, nav_msg.orientation.yaw = values[5:8]

        self.pub_nav.publish(nav_msg)

    def parse_position_req(self, payload_type, id, body):
        values = struct.unpack(FORMAT[payload_type], body)

        pilot_msg = PilotRequest()
        pilot_msg.position = list(values[0:6])

        self.pub_position.publish(pilot_msg)

    def parse_body_req(self, payload_type, id, body):
        values = struct.unpack(FORMAT[payload_type], body)

        pilot_msg = PilotRequest()
        pilot_msg.position = list(values[0:6])

        self.pub_body.publish(pilot_msg)

    # TODO: finish custom msg
    def parse_string(self, payload_type, id, payload):
        pass

    def parse_ack(self, payload_type, id, body):
        rospy.loginfo('%s: Message with id %s was delivered' % (self.name, id))

    def parse_unknown(self):
        pass

    def loop(self):
        pass

if __name__ == '__main__':
    rospy.init_node('packer_parser')
    name = rospy.get_name()

    config = DEFAULT_CONFIG.copy()
    # load global parameters
    param_config = rospy.get_param('~packer_config', {})
    # Update default settings with user specified params
    config.update(param_config)

    rospy.loginfo('%s: Loaded config is: %s' % (name, config))

    pp = PackerParser(name, config)
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



