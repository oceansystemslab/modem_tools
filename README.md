modem_tools
===============

This package contains high level tools for sending messages via acoustic modem.

payload_processor node
==================

This node subscribes to a number of topics specified by the user. The types of messages that can appear on these topics
are hardcoded. Once the node receives a specific message it converts to a string and passes it to modem driver.
Then the message is sent over the acoustic link.

The node can also parse messages from the modem. Once the modem driver publishes a payload that arrived at the modem
the payload_processor node will attempt to identify the type of message, reconstruct it and publish to the topic that 
corresponds to this type of message.

Currently the payload_processor allows for sending burst messages only (they can be larger than instant messages - up to
1024 bytes and are more likely to be delivered).

Currently only messages of specific types can be sent. These types include:
  - _POSITION_REQUEST = 'position_request'
  - _BODY_REQUEST = 'body_request'
  - _NAV = 'nav'
  - _STRING_IMAGE = 'string_image'
  - _ACK = 'ack'
  
To send a message of different type you have to add it to the software. How to do this is specified in "Adding new
message type" below. In the future a functionality will be added that allows for sending of any ROS message.

Adding new message type
=======================

To use a new type of message you will have to add the following:
  0) add new ROS message type in the imports
  1) new type of payload constant
  2) new topic in the TOPICS dictionary
  3) if it requires ack add new type to REQUIRING_ACK list
  4) id of the new type in TYPE_TO_ID dictionary
  5) struct format of the message in FORMAT dictionary
  6) subscriber for the new topic
  7) handle for the new topic
  8) parser for the new message
  9) add new message type and parser to the parse dictionary in PackerParser class
  9) publisher for the new topic