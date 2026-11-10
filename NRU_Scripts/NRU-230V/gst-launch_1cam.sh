#!/bin/bash
gnome-terminal -- gst-launch-1.0 -v v4l2src device=/dev/video$1 ! fpsdisplaysink name=fps$1 video-sink=xvimagesink
