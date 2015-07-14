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

TOPIC_STRING_TO_ID = {value: key for key, value in TOPICS_ID_TO_STRING.items()}

# Add messages types here and in the set below (1-255)
ROS_MSG_ID_TO_TYPE = {
    1:      PilotRequest,
    # 2:      NavSts,  # has too many sub types for now
    3:      String,
}
ROS_MSG_TYPE_TO_ID = {value: key for key, value in ROS_MSG_ID_TO_TYPE.items()}

# all types of messages that can be converted to structs
ROS_MSG_TYPES = {Header}  # additional message types
ROS_MSG_TYPES.union(set(ROS_MSG_ID_TO_TYPE.values()))  # union with already defined ones

def ros_msg_string2type(msg_name):
    for msg_type in ROS_MSG_TYPES:
        if msg_type.__name__ == msg_name:
            return msg_type
    raise KeyError('%s message is not described in message_config.py' % msg_name)
