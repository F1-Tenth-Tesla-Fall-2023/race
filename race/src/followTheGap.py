import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# ROS publisher for controlling the car's movement
command_pub = rospy.Publisher('/car_8/ackermann_cmd', AckermannDrive, queue_size=1)

# Constants
threshold = 0.2  # Threshold for identifying disparities (adjust as needed)
car_width = 0.5  # Width of the car in meters
tolerance = 0.1  # Tolerance for gap calculation (adjust as needed)

# Function to calculate the number of LIDAR samples needed
def calculate_samples_needed(distance, angleInc):
    alpha = math.atan((car_width / 2 + tolerance) / distance)
    num_samples_needed = int(math.ceil(alpha / angleInc))
    return num_samples_needed

def process_lidar_scan(fullRange, angleInc, angleMin):
    # Identify and process disparities
    for i in range(1, len(fullRange)):
        # Calculate the difference between subsequent points
        disparity = abs(fullRange[i] - fullRange[i - 1])

        # Check if the disparity is larger than the threshold
        if disparity > threshold:
            # Determine the closer distance in the disparity
            desired_distance = min(fullRange[i], fullRange[i-1]) 
            # Calculate the number of LIDAR samples needed at closer distance
            num_samples_needed = calculate_samples_needed(desired_distance, angleInc)

            # Overwriting the needed number of samples with the smaller value
            if desired_distance == fullRange[i-1]:
                for j in range(i, i+num_samples_needed):
                    fullRange[j] =  min(fullRange[j],fullRange[i-1])
            else:
                for j in range(i-1, i-num_samples_needed-1, -1):
                    fullRange[j] =  min(fullRange[j],fullRange[i])

    # Find the index in the range coorelated to the direct left and right of the car
    leftAngle = math.radians(0) - (math.pi / 2)
    rightAngle = math.radians(180) - (math.pi / 2)
    leftIndex = int((leftAngle - angleMin) / angleInc)
    rightIndex = int((rightAngle - angleMin) / angleInc)
    filteredDistances = fullRange[leftIndex:rightIndex+1]
    #Return the filteredRanges after processing disparities and limiting the range
    return filteredDistances

def findGap(scan):

    proccessedRanges = process_lidar_scan(scan.ranges, scan.angle_increment, scan.angle_min)

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

    # TODO Dynamic velocity scaling
    speed = 25
    
    # Return an angle to turn towards (0 is straight ahead)
    return target_angle, speed 

def callback(data):
    angle, speed = findGap(data)
    command = AckermannDrive()
    command.steering_angle = max(-100.0, min(100.0, angle))
    command.speed = max(0.0, min(40.0, speed)) 
    print("Angle: " + str(angle) + ", Speed: " + str(speed))
    command_pub.publish(command)

def main():
    rospy.init_node('follow_the_gap', anonymous=True)
    rospy.Subscriber('/car_8/laser/scan', LaserScan, process_lidar_scan)
    rospy.spin()

if __name__ == '__main__':
    main()
