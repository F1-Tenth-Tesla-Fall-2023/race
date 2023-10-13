#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	# Get the angle range properties from the LaserScan message
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    # Calculate the index corresponding to the specified angle
    # Convert angle to radians
    angle_rad = math.radians(angle) - (math.pi / 2)
    # Angle withn valid range
    #if angle_rad < angle_min:
    #    angle_rad = angle_min
    #elif angle_rad > angle_max:
    #    angle_rad = angle_max
    # Calculate the index for the specified angle
    index = int((angle_rad - angle_min) / angle_increment)
    # Check for NaN values and return a default
    if math.isnan(ranges[index]):
        return 0.0
    # Return the range value for the specified angle
    #print(ranges[index])
    return ranges[index]



def callback(data):
	global forward_projection

	theta = 50 # you need to try different values for theta
        #print("A distance")
	a = getRange(data,theta) # obtain the ray distance for theta
        #print("B distance")
	b = getRange(data,0)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	swing = math.radians(theta)

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	# TODO: implement
	# Calculate Alpha (right wall)
        alpha = math.atan2((a * math.cos(swing) - b), (a * math.sin(swing)))
    # Calculate distance AB from the right wall
        AB = b * math.cos(alpha)
    # Calculate the projected distance from the wall (CD)
        CD = AB + forward_projection * math.sin(alpha)
    # Calculate the error as the difference between the desired_distance and CD
        error = desired_distance - CD

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_8/scan",LaserScan,callback)
	rospy.spin()
