#!/usr/bin/env python
import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
import numpy as np

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

def createBubble(scans):
    ranges = list(scans)
    n = len(ranges)
    index = 0
    close_distance = 9999
    for i in range(len(ranges)):
        if ranges[i] > 0.05 and ranges[i] < close_distance:
            index = i 
            close_distance = ranges[i]
    bubble_index_offset = int(math.degrees(math.atan(car_width / close_distance))) * (1.0 / math.degrees(scans.angle_increment))
    for i in range(int(max(0, index - bubble_index_offset)), int(min(index + bubble_index_offset, len(ranges)))):
        ranges[i] = 0.0 

def findBestGap(ranges, angle_inc):
    gap_threshold = 1.0
    #Two pointer solution to find all gaps within a threshold
    gaps = []
    l=0
    r=1
    while r<len(ranges):
        if ranges[l]<gap_threshold:
            l+=1
            r=l+1
        elif ranges[r]<gap_threshold:
            neededSamples =  int(abs(math.atan((car_width / 2.0) / ranges[l + 1]))/angle_inc)
            if r-l >= neededSamples:
                gaps.append(ranges[l:r])
            l=r+1
            r=l+1
        else:
            r+=1

    #returns the gap with the largest distance
    return(sorted(gaps, key=max, reverse=True)[0])

    #returns the widest gap
    #return(sorted(gaps, key=len, reverse=True)[0])

def pickGap(data):

    global limit_threshold
    global minGapDistance

    # Define the indexing range for -90 to 90 degrees
    index_min = int((np.radians(-90) - data.angle_min) / data.angle_increment)
    index_max = int((np.radians(90) - data.angle_min) / data.angle_increment)
    forwardRanges = data.ranges[index_min:index_max]

     # Calculate the max gap and its deepest point
    bestGap = findBestGap(forwardRanges, data.angle_increment)
    startOfBestGap = data.ranges.index(bestGap[0])
    indexInBestGap =  bestGap.index(max(bestGap))
    maxGapIndex = startOfBestGap+indexInBestGap
    max_gap = data.ranges[maxGapIndex]

    # Return the appropriate angle
    if max_gap > limit_threshold:
        angle = (maxGapIndex * data.angle_increment + data.angle_min) * 180.0/np.pi
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

def callback(data):

    global car_width
    global threshold 
    
    # Step 1 - Find disparities in the scan
    ranges = extendDisparities(data)
    new_scan = data
    new_scan.ranges = ranges
    postBubble = createBubble(new_scan)
    postBubbleScan = data
    postBubbleScan.ranges = postBubble
    # publish disparity scan for rviz
    disparity_pub.publish(postBubbleScan)


    # Step 2 - Pick the largest gap in the scan, between -90 and 90 degrees
    angle = pickGap(postBubbleScan)
    
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
    minGapDistance = 2.0

    rospy.init_node('ftg', anonymous = True)
    rospy.Subscriber("/car_8/scan", LaserScan, callback)
    rospy.spin()
