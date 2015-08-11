#!/usr/bin/env bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <N> <E> <D> <YAW>"
    echo "Send the body request over acoustic link."
    echo ""
    echo "Mandatory arguments:"
    echo "  <N>: translation in vehicle's North in metres"
    echo "  <E>: translation in vehicle's East in metres"
    echo "  <D>: translation in vehicle's depth in metres"
    echo "  <YAW>: rotation in radians"
    echo ""
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
N=$1
E=$2
D=$3
# TODO: convert to degrees
YAW=$4

rostopic pub -1 /modem/packer/body_req vehicle_interface/PilotRequest "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
priority: 0
position: [${N}, ${E}, ${D}, 0.0, 0.0, ${YAW}]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
limit_velocity: [0, 0, 0, 0, 0, 0]
disable_axis: [0, 0, 0, 0, 0, 0]"

