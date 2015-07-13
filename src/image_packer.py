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

import numpy as np
import cv2
import roslib
roslib.load_manifest('modem_tools')

import rospy

# Messages
from diagnostic_msgs.msg import KeyValue

# Services
from vehicle_interface.msg import String
from vehicle_interface.srv import BooleanService, BooleanServiceResponse

# Constants
TOPIC_MODEM_CONSTRUCTOR = '/modem/packer/image'
SRV_SIGNAL = '/image_packer/signal'
LOOP_RATE = 0.1  # Hz

MAX_SIZE = 128*128  # pixels
QUALITY = 50  # from 0 (worst) to 100

class ImagePacker(object):
    def __init__(self, name):
        self.name = name

        self.go = True

        # Publishers
        self.pub_modem = rospy.Publisher(TOPIC_MODEM_CONSTRUCTOR, String)

        # Services
        self.srv_signal = rospy.Service(SRV_SIGNAL, BooleanService, self.handle_signal)

    def handle_signal(self, srv):
        self.go = srv.request
        return BooleanServiceResponse(srv.request)

    def generate_image_string(self):
        im = cv2.imread('diver.jpg')
        im = cv2.cvtColor(im, cv2.COLOR_RGB2GRAY)

        # scale down the image
        scale = np.sqrt(MAX_SIZE/im.size)
        scale = min(scale, 1)
        im = cv2.resize(im, (0, 0), fx=scale, fy=scale)
        shape = im.shape

        success, im = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])

        # first two bytes are dimensions of the image
        s = ''
        for value in im:
            s += chr(value)

        rospy.loginfo('%s: Image length: %s' % (self.name, len(s)))
        return s

    def publish_image(self):
        s = self.generate_image_string()
        self.pub_modem.publish(String(header=rospy.Time.now(), payload=s))

    def loop(self):
        if self.go:
            self.publish_image()
            # self.go = False


if __name__ == '__main__':
    rospy.init_node('image_packer')
    name = rospy.get_name()

    packer = ImagePacker(name)
    loop_rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        try:
            packer.loop()
            loop_rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo('%s caught ros interrupt!', name)
        # except Exception as e:
        #     rospy.logfatal('%s', e)
        #     rospy.logfatal('%s caught exception and dying!', name)
        #     sys.exit(-1)



