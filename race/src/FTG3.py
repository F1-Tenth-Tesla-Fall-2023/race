#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan

# Field of view for the car (max 240)
global fov

# Car width
global car_width

# Disparity extender threshold
global threshold

command_pub = rospy.Publisher('/car_8/offboard/command', AckermannDrive, queue_size = 1)

#### NOTES ######
# Early in the scans list is RIGHT
# Negative is RIGHT in scans
# Positive is LEFT in scans 
# angle_min is on the RIGHT in scans

# Return the angle for an index 
def getAngleFromIndex(angle_inc, index, fov):
    return (fov / -2.0) + (float(index) * math.degrees(angle_inc))

# Return the ranges for a given FOV
def getRangesForFOV(scans, fov):
    lower_index = int(((fov / -2.0) - math.degrees(scans.angle_min)) / math.degrees(scans.angle_increment))
    higher_index = int((math.degrees(scans.angle_max) - math.degrees(scans.angle_min) - (math.degrees(scans.angle_max) - (fov / 2.0))) / math.degrees(scans.angle_increment))
    return [10.0 if math.isnan(d) else d for d in scans.ranges[lower_index : higher_index]]

# Disparity extender
def disparity_extend(ranges_list, angle_inc):
    ranges = [a for a in ranges_list]
    
    # Clean tiny values 
    val = 0
    if ranges[0] < 0.05:
        for i in range(len(ranges)):
            if ranges[i] > 0.05:
                val = ranges[i]
    for i in range(len(ranges)):
        if ranges[i] < 0.05:
            ranges[i] = val
        else:
            val = ranges[i]
            
    for i in range(len(ranges) - 1):
        diff = ranges[i] - ranges[i + 1]

        # lidar sees end of obstacle
        if diff < -threshold: 
            
            # Extend the obstacle by half the width of the car
            angle_offset = min(15.0, abs(math.degrees(math.atan((car_width / 2.0) / ranges[i]))))
            for j in range(1, min(len(ranges), 1 + int(angle_offset / math.degrees(angle_inc)))):
                ranges[j] = ranges[i]

        # lidar sees beginning of obstacle 
        elif diff > threshold: 

            # Extend the obstacle by half the width of the car
            angle_offset = math.degrees(abs(math.atan((car_width / 2.0) / ranges[i + 1])))
            for j in range(0, min(i, 1 + int(angle_offset / math.degrees(angle_inc)))):
                ranges[i - j] = ranges[i + 1]
    return ranges 

# Piece-wise speed function based on distance 
def getVelocityForDistance(dist):
    if dist <= 0.75: 
        return 10.0
    if dist <= 1.5: 
        return 20.0  
    return 25.0 

# Pick a gap to follow 
def findGap(scans):

    # Subset the scans to get data for only the desired FOV
    ranges = getRangesForFOV(scans, fov)

    # Extend obstacles to avoid collisions 
    ranges = disparity_extend(ranges, scans.angle_increment)
    
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

    # Dynamic velocity scaling 
    speed = getVelocityForDistance(close_distance)
    
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle * angle_mult, speed 

# Handle the scans from the lidar
def callback(data):
    global fov 
    global car_width
    global threshold 
    angle, speed = findGap(data)
    command = AckermannDrive()
    command.steering_angle = max(-100.0, min(100.0, angle))
    command.speed = max(0.0, min(40.0, speed)) 
    print("Angle: " + str(angle) + ", Speed: " + str(speed))
    command_pub.publish(command)

if __name__ == '__main__':
    fov = 180
    car_width = 0.5
    threshold = 0.1
    rospy.init_node('gap_finder', anonymous = True)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
