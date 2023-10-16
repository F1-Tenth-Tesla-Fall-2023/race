#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global error
	global start_time
	global angle
	integral = 0.0
	error = 0.0
	prev_error = 0.0
	angle = 0.0
	print("PID Control Node is Listening to error")
	dt = rospy.Time.now()-start_time
	start_time = rospy.Time.now()
	dt = dt.to_sec()

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller

	# 1. Scale the error
	scale_factor = 1 # Idk what to put it to.
	error = data.pid_error * scale_factor

	# 2. Apply the PID equation on error to compute steering
	proportional = kp * error
	integral += ki * dt
	derivative = kd * (error - prev_error)
	angle = proportional + integral + derivative

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	#angle = max(min(angle, 100), -100)
        #angle *= 5
	command.steering_angle = angle

	# TODO: Dynamic Velocity Scaling
  vel_range = 10
	max_vel = vel_input + vel_range
	min_vel = vel_input - vel_range
	a = 10 # Aggresiveness of sigmoid
	b = -5 # Shift of sigmoid
	vel_input = (max_vel - min_vel) / (1 + math.exp(a * math.abs(error) + b)) + min_vel # https://www.desmos.com/calculator/ppbv9va1tt
	
	# TODO: Make sure the velocity is within bounds [0,100]
	vel_input = max(min(vel_input, 100), 0)
	command.speed = vel_input

	# Move the car autonomously
	command_pub.publish(command)

	# Update the previous error for the next iteration
	prev_error = error

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	global start_time
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	start_time = rospy.Time.now()
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
