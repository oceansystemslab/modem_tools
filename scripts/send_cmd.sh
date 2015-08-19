#!/usr/bin/env bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <MODE> <RESUME_POINT>"
    echo "Send the body request over acoustic link."
    echo ""
    echo "Mandatory arguments:"
    echo "  <MODE>: "
    echo "  <RESUME_POINT>: "
    echo ""
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# vars
MODE=$1
RESUME_POINT=$2

rostopic pub -1 /modem/packer/command eurathlon_msgs/Command "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
mode: ${MODE}
resume_point: ${RESUME_POINT}"
