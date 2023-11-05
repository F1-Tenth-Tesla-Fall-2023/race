#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
import test_gap 

# Field of view for the car (max 240)
global fov

# Car width
global car_width

# Disparity extender threshold
global threshold

global driver 
global default_fov

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
    angle_extension = 20.0
    
    # Clean tiny values 
    val = 0
    if ranges[0] < 0.05:
        for i in range(len(ranges)):
            if ranges[i] > 0.05:
                val = ranges[i]
                break 
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
            angle_offset = min(angle_extension, abs(math.degrees(math.atan((car_width / 2.0) / ranges[i]))))
            for j in range(1, min(len(ranges), 1 + int(angle_offset / math.degrees(angle_inc)))):
                if ranges[j] > ranges[i]:
                    ranges[j] = ranges[i]

        # lidar sees beginning of obstacle 
        elif diff > threshold: 

            # Extend the obstacle by half the width of the car
            angle_offset = min(angle_extension, abs(math.degrees(abs(math.atan((car_width / 2.0) / ranges[i + 1])))))
            for j in range(0, min(i, 1 + int(angle_offset / math.degrees(angle_inc)))):
                if ranges[i - j] > ranges[i + 1]:
                    ranges[i - j] = ranges[i + 1]
    return ranges 

# Piece-wise speed function based on distance 
def getVelocityForDistance(angle, dist):
    if dist <= 2.0:
        return 15.0
    if angle <= 20.0:
        return 30.0
    return 15.0

def gapAveraging(scans):
    ahead = int((0 - math.degrees(scans.angle_min)) / math.degrees(scans.angle_increment))
    if scans.ranges[ahead] >= 2.0: 
        fov = 240
    else: fov = 210
    ranges = getRangesForFOV(scans, fov)
    ranges = disparity_extend(ranges, scans.angle_increment)
    left = ranges[:(len(ranges) / 2)]
    right = ranges[(len(ranges) / 2):]
    # for i in range(len(left)-15, len(left)): left[i] = 0
    # for i in range(15): right[i] = 0
    left_avg = sum(left) / len(left)
    right_avg = sum(right) / len(right)
    diff = left_avg - right_avg 
    angle = diff * -55.0
    speed = 30.0 
    if abs(angle) >= 40.0: speed = 20.0
    return angle, speed

# Pick a gap to follow 
def findGap(scans):

    # Subset the scans to get data for only the desired FOV
    ranges = getRangesForFOV(scans, fov)
    
    # Extend obstacles to avoid collisions 
    ranges = disparity_extend(ranges, scans.angle_increment)
    
    max_dist = max(ranges)
    max_index = ranges.index(max_dist)
    print("Target: " + str(max_dist) + str(max_index))
    gap_threshold = 2.0
    gaps = []
    counter = 0
    initial = -1
    for i in range(len(ranges)):
        if ranges[i] >= gap_threshold:
            if initial < 0: initial = i 
            counter += 1
        else:
            if initial >= 0:
                gaps.append((initial, counter))
            initial = -1
            counter = 0 
    best_gap = (0, 0)
    for gap in gaps:
        if gap[1] > best_gap[1]:
            best_gap = gap 
    gap_angle = getAngleFromIndex(scans.angle_increment, best_gap[0] + best_gap[1] / 2, fov)
    
    deep_angle = getAngleFromIndex(scans.angle_increment, max_index, fov)

    # Dynamic velocity scaling 
    target_angle = (gap_angle + deep_angle) / 2.0
    angle_mult = 1.0
    speed = getVelocityForDistance(target_angle * angle_mult, max_dist)
    
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle * angle_mult, speed 

# Handle the scans from the lidar
def callback(data):
    global fov 
    global default_fov 
    global car_width
    global threshold 
    global driver 
    # Hunter
    # angle, speed = findGap(data)
    
    # Average halves
    angle, speed = gapAveraging(data)
    
    # Paul's algorithm
    # speed, angle = driver.process_lidar(data.ranges)
    # angle = math.degrees(angle)
    
    command = AckermannDrive()
    command.steering_angle = max(-100.0, min(100.0, angle))
    command.speed = max(0.0, min(40.0, speed)) 
    print("Angle: " + str(angle) + ", Speed: " + str(speed))
    command_pub.publish(command)

if __name__ == '__main__':
    fov = 210
    default_fov = fov 
    car_width = 0.5
    threshold = 0.2
    driver = test_gap.GapFollower()
    rospy.init_node('gap_finder', anonymous = True)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
