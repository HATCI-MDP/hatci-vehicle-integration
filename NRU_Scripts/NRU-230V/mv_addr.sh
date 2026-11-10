#!/bin/bash

# Ensure to only move address once after each boot

if [ "$EUID" -ne 0 ]
  then echo "Please run with sudo"
  exit
fi

order_moved=0

# Check if video node moved
if [[ -e "/tmp/video_order_moved.log" ]]; then  
	LINE=$(wc -l < "/tmp/video_order_moved.log")

	if [ "$LINE" -eq 2 ]
	then
    order_moved=1
    echo "/dev/video node order adjusted."
	  exit
	fi
echo 
fi

if [ "$order_moved" -eq 0 ]
  then echo "About to adjust /dev/video order."  
fi



# 0->10, 1->11, 2->12, 3->13, 4->14, 5->15, 6->16, 7->17
# 10->5, 11->7, 12->4, 13->6, 14->1, 15->3, 16->0, 17->2


if [[ -e "/dev/video0" ]]; then
   sudo mv /dev/video0 /dev/video10
fi

if [[ -e "/dev/video1" ]]; then
   sudo mv /dev/video1 /dev/video11
fi

if [[ -e "/dev/video2" ]]; then
   sudo mv /dev/video2 /dev/video12
fi

if [[ -e "/dev/video3" ]]; then
   sudo mv /dev/video3 /dev/video13
fi

if [[ -e "/dev/video4" ]]; then
   sudo mv /dev/video4 /dev/video14
fi

if [[ -e "/dev/video5" ]]; then
   sudo mv /dev/video5 /dev/video15
fi

if [[ -e "/dev/video6" ]]; then
   sudo mv /dev/video6 /dev/video16
fi

if [[ -e "/dev/video7" ]]; then
   sudo mv /dev/video7 /dev/video17
fi

#------------------------------------------

if [[ -e "/dev/video10" ]]; then
   sudo mv /dev/video10 /dev/video5
fi

if [[ -e "/dev/video11" ]]; then
   sudo mv /dev/video11 /dev/video7
fi

if [[ -e "/dev/video12" ]]; then
   sudo mv /dev/video12 /dev/video4
fi

if [[ -e "/dev/video13" ]]; then
   sudo mv /dev/video13 /dev/video6
fi

if [[ -e "/dev/video14" ]]; then
   sudo mv /dev/video14 /dev/video1
fi

if [[ -e "/dev/video15" ]]; then
   sudo mv /dev/video15 /dev/video3
fi

if [[ -e "/dev/video16" ]]; then
   sudo mv /dev/video16 /dev/video0
fi

if [[ -e "/dev/video17" ]]; then
   sudo mv /dev/video17 /dev/video2
fi


echo "7" > /tmp/video_order_moved.log; echo "7" >> /tmp/video_order_moved.log
echo "/dev/video node order adjusted!"



