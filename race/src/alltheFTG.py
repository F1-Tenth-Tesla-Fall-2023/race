#!/usr/bin/env python
import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
import numpy as np

import test_gap

global car_width # width of the car, used to calculate the number of samples to cover half the car width
global threshold # threshold for a disparity
global limit_threshold # threshold that when hit the car will go towards center of gap
global tolerance # tolerance for the number of samples to cover half the car width

command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)

disparity_pub = rospy.Publisher('/car_8/disparity', LaserScan, queue_size = 1)

def calculateSamplesToCoverHalfCarWidth(distance1, distance2, car_width, angle_increment):
    global tolerance
    # Calculate the span across the car that each lidar sample covers
    # Adding the spans across the car width due to two consecutive readings
    span_per_increment = math.sin(angle_increment) * distance1 + math.sin(angle_increment) * distance2
    # Calculate the number of samples required to cover half the car's width plus a tolerance
    num_samples = (car_width / 2 * (1 + tolerance)) / span_per_increment
    # Return the ceiling of num_samples because we can't have a fraction of a sample and we need at least this many
    return int(math.ceil(num_samples))

def extendDisparities(data):

    global car_width
    global threshold

    angle_increment = data.angle_increment
    ranges = data.ranges

    new_ranges = list(ranges)
    n = len(ranges)

    # Find the disparities, where two subsequent points are further apart than the threshold
    for i in range(n - 1):

        # dismiss if either of the points are inf, or less than car_width
        if np.isinf(ranges[i]):
            new_ranges[i] = 0
            continue
        if np.isinf(ranges[i + 1]):
            new_ranges[i + 1] = 0
            continue
        
        # maybe help with corner detection
        # if ranges[i] < car_width or ranges[i + 1] < car_width:
        #     new_ranges[i] = 0
        #     continue

        if abs(ranges[i] - ranges[i + 1]) > threshold:
            closer_distance = min(ranges[i], ranges[i + 1])
            samples_to_update = calculateSamplesToCoverHalfCarWidth(ranges[i], ranges[i + 1], car_width, angle_increment)
            
            # For each pair of pounts in a disparity,
            # - calculate the number of lidar samples to cover half the width of the car, plus tolerance
            # - for the side that is further, set that number of samples to the closer distance - but don't overwrite any closer samples
            if ranges[i] > ranges[i + 1]:
                # Extend to the left
                for j in range(i, max(i - samples_to_update, -1), -1):
                    if new_ranges[j] > closer_distance:
                        new_ranges[j] = closer_distance
                        # new_ranges[j] = 0
            else:
                # Extend to the right
                for j in range(i + 1, min(i + 1 + samples_to_update, n)):
                    if new_ranges[j] > closer_distance:
                        new_ranges[j] = closer_distance
                        # new_ranges[j] = 0
    
    return new_ranges

def pickGap5(scan):
    gapFollower = test_gap.GapFollower()
    gapFollower.process_lidar(scan.ranges)

def pickGap4(scan):
    proccessedRanges = scan.ranges
     # Find the closest point 
    index = 0
    close_distance = 9999
    for i in range(len(proccessedRanges)):
        if proccessedRanges[i] > 0.05 and proccessedRanges[i] < close_distance:
            index = i 
            close_distance = proccessedRanges[i]
    close_angle = math.atan((car_width / 2 + tolerance) / close_distance)
    
    # Set the bubble to 0
    bubble_samples_needed = calculate_samples_needed(close_distance, scan.angle_increment)
    for i in range(int(max(0, index - bubble_samples_needed)), int(min(index + bubble_samples_needed, len(proccessedRanges)))):
        proccessedRanges[i] = 0.0
        
    # Pick the largest gap and choose a direction
    target_angle = 0

    # Head towards the deeepest point in the gap
    deepest_point = 0.0 
    deepest_index = 0
    if(close_angle >= 0.0): 
        sublist = proccessedRanges[:index]
        target_angle = math.atan((car_width / 2 + tolerance) / max(sublist)) 
        deepest_point = max(sublist)
        deepest_index = sublist.index(deepest_point)
    else: 
        sublist = proccessedRanges[index:]
        target_angle = math.atan((car_width / 2 + tolerance) / max(sublist)) 
        deepest_point = max(sublist)
        deepest_index = index + sublist.index(deepest_point)
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle

# car_width2 = 0.62 # Car width = half of the car plus tolerance
width_tolerance = 1.75
disparity_threshold = 2.0
scan_width = 270.0
lidar_max_range = 7.0
turn_clearance = 0.25
max_turn_angle = 24.0
min_speed = 5 # Last tunable param
max_speed = 7.0  # Max Val : 20
min_distance = 0.25
max_distance = 6.0  # Tune this param
right_extreme = 180
left_extreme = 900
coefficient_of_friction = 0.62
wheelbase_width = 0.3302
gravity = 9.81998  # sea level
prev_threshold_angle = 0
integral_prior = 0
def threshold_speed(velocity, forward_distance, straight_ahead_distance):
    if straight_ahead_distance > max_distance:
        velocity = max_speed
    elif forward_distance < min_distance:
        velocity = -0.5
    else:
        velocity = (straight_ahead_distance / max_distance) * velocity
    if velocity < min_speed:
        velocity = min_speed
    return velocity
def adjust_turning_for_safety(left_distances, right_distances, angle):
    min_left = min(left_distances)
    min_right = min(right_distances)
    # Increase the turn_clearance. Also try to reduce the straight ahead distance. This will cause  it to turn late. But, what the fuck...
	# Try to change the min_left to reflect average not min.
    if min_left <= turn_clearance and angle > 0.0:  # .261799:
        angle = 0.0
    elif min_right <= turn_clearance and angle < 0.0:  # -0.261799:
        angle = 0.0
    else:
        return angle
    return angle
def calculate_min_turning_radius(angle):
    if abs(angle) < 0.0872665:  # if the angle is less than 5 degrees just go as fast possible
        return max_speed
    else:
        turning_radius = (wheelbase_width / (2*math.sin(abs(angle))))
        maximum_velocity = math.sqrt(coefficient_of_friction * gravity * turning_radius)
        if maximum_velocity < max_speed:
            maximum_velocity = maximum_velocity * (maximum_velocity / max_speed)
        else:
            maximum_velocity = max_speed
    return maximum_velocity
def threshold_angle(angle):
    max_angle_radians = max_turn_angle * (math.pi / 180)
    if angle < (-max_angle_radians):
        return -max_angle_radians
    elif angle > max_angle_radians:
        return max_angle_radians
    else:
        return angle
def calculate_target_distance(arr):
    if (len(arr) == 1):
        return arr[0], 0
    else:
        mid = int(len(arr) / 2)
        return arr[mid], mid
def calculate_angle(index):
    factor = (left_extreme - 540)/90
    angle = (index - 540) / factor
    if -5 < angle < 5:
        angle = 0.0
    rad = (angle * math.pi) / 180
    return rad
def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))
def pickGap3(data):
    # new_ranges = extend_disparities(limited_ranges, disparities, car_width)
    new_ranges = data.ranges
    gauss_range = np.arange(0,1080,1)
    gaussian_weights = gaussian(gauss_range,540,150) 
    gaussian_weights = gaussian_weights + (1 - gaussian_weights[0])
    new_ranges = new_ranges*gaussian_weights
    max_value = max(new_ranges)
    target_distances = np.where(new_ranges >= (max_value - 1))[0]  #index values

    driving_distance_index, chosen_index = calculate_target_distance(target_distances)
    driving_angle = calculate_angle(driving_distance_index)
    thresholded_angle = threshold_angle(driving_angle)

    behind_car = np.asarray(data.ranges)
    behind_car_right = behind_car[0:right_extreme]
    behind_car_left = behind_car[(left_extreme+1):]

    #change the steering angle based on whether we are safe
    thresholded_angle = adjust_turning_for_safety(behind_car_left, behind_car_right, thresholded_angle)
    velocity = calculate_min_turning_radius(thresholded_angle)
    velocity = threshold_speed(velocity, new_ranges[driving_distance_index], new_ranges[540])

fov = 240
def getAngleFromIndex(angle_inc, index, fov):
    return (fov / -2.0) + (float(index) * math.degrees(angle_inc))
def pickGap2(scans):
    ranges = scans.ranges
    # Find the closest point 
    index = 0
    close_distance = 9999
    for i in range(len(ranges)):
        if ranges[i] > 0.05 and ranges[i] < close_distance:
            index = i 
            close_distance = ranges[i]
    close_angle = getAngleFromIndex(scans.angle_increment, index, fov) 
    # Set the bubble to 0
    bubble_index_offset = int(math.degrees(math.atan(car_width / close_distance))) * (1.0 / math.degrees(scans.angle_increment))
    for i in range(int(max(0, index - bubble_index_offset)), int(min(index + bubble_index_offset, len(ranges)))):
        ranges[i] = 0.0
    # Pick the largest gap and choose a direction
    target_angle = 0
    angle_mult = 1.0 # Angle scalar
    # Head towards the deeepest point in the gap
    deepest_point = 0.0 
    deepest_index = 0
    if(close_angle >= 0.0): 
        sublist = ranges[:index]
        target_angle = getAngleFromIndex(scans.angle_increment, sublist.index(max(sublist)), fov) 
        deepest_point = max(sublist)
        deepest_index = sublist.index(deepest_point)
    else: 
        sublist = ranges[index:]
        target_angle = getAngleFromIndex(scans.angle_increment, index + sublist.index(max(sublist)), fov)
        deepest_point = max(sublist)
        deepest_index = index + sublist.index(deepest_point)
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle

def pickGap(data):
    global limit_threshold
    # Define the indexing range for -90 to 90 degrees
    index_min = int((np.radians(-90) - data.angle_min) / data.angle_increment)
    index_max = int((np.radians(90) - data.angle_min) / data.angle_increment)
    # Calculate the max gap and its center
    max_gap = max(data.ranges[index_min:index_max])
    max_gap_center = data.ranges.index(max_gap)
    # Return the appropriate angle
    if max_gap > limit_threshold:
        angle = (max_gap_center * data.angle_increment + data.angle_min) * 180.0/np.pi
    else:
        deepest_direction_point = data.ranges.index(max(data.ranges))
        angle = (deepest_direction_point * data.angle_increment + data.angle_min) * 180.0/np.pi
    return angle

def cornerDetection(data):
    global car_width
    # Define the indexing range for up to -90 and up to 90 degrees
    index_min = int((np.radians(-100) - data.angle_min) / data.angle_increment)
    index_max = int((np.radians(100) - data.angle_min) / data.angle_increment)
    # if there is a value less than car_width in the range or inf or 0, return true
    for i in range(0, index_min):
        if data.ranges[i] < car_width / 2 and data.ranges[i] != 0:
            print("corner detected")
            return True
    for i in range(index_max, len(data.ranges)):
        if data.ranges[i] < car_width / 2 and data.ranges[i] != 0:
            print("corner detected")
            return True
    return False

# Function to calculate the number of LIDAR samples needed
def calculate_samples_needed(distance, angleInc):
    alpha = math.atan((car_width / 2 + tolerance) / distance)
    num_samples_needed = int(math.ceil(alpha / angleInc))
    return num_samples_needed

def bubble(data):
    proccessedRanges = list(data.ranges)
    # Find the closest point 
    index = 0
    close_distance = 9999
    for i in range(len(proccessedRanges)):
        if proccessedRanges[i] > 0.05 and proccessedRanges[i] < close_distance:
            index = i 
            close_distance = proccessedRanges[i]
    close_angle = math.atan((car_width / 2 + tolerance) / close_distance)
    
    # Set the bubble to 0
    bubble_samples_needed = calculate_samples_needed(close_distance, data.angle_increment)
    for i in range(int(max(0, index - bubble_samples_needed)), int(min(index + bubble_samples_needed, len(proccessedRanges)))):
        proccessedRanges[i] = 0.0

    return close_angle, proccessedRanges

def callback(data):

    global car_width
    global threshold 
    
    # Step 1 - Find disparities in the scan
    ranges = extendDisparities(data)
    new_scan = data
    new_scan.ranges = ranges
    # publish disparity scan for rviz
    disparity_pub.publish(new_scan)

    # Step 1.5 - bubble
    close_angle, ranges = bubble(new_scan)

    # Step 2 - Pick the largest gap in the scan, between -90 and 90 degrees
    angle = pickGap(new_scan)
    # angle = pickGap2(new_scan)
    # angle = pickGap3(new_scan)
    # angle = pickGap4(new_scan)
    # angle = pickGap5(data)
    
    # Step 3 - Corner detection
    #TODO: not working
    # angle = 0 if cornerDetection(new_scan) else angle

    # Step 4 - Vary the speed based on the angle
    speed = 40.0 - (abs(angle) / 90.0) * 40.0

    # Step 5 - Publish the command
    command = AckermannDrive()
    command.steering_angle = max(-100.0, min(100.0, angle))
    command.speed = max(0.0, min(40.0, speed)) 
    print("Angle: " + str(angle) + ", Speed: " + str(speed))
    command_pub.publish(command)

if __name__ == '__main__':

    car_width = 0.2
    threshold = 0.1
    limit_threshold = 2.0
    tolerance = 4

    rospy.init_node('ftg', anonymous = True)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
