#!/usr/bin/env python

# An enumeration of drone statuses for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

class DroneStatus(object):
	Emergency = 0
	Inited    = 1
	Landed    = 2
	Flying    = 3
	Hovering  = 4
	Test      = 5
	Looping   = 6
	TakingOff = 7
	GotoHover = 8
	Landing   = 9
