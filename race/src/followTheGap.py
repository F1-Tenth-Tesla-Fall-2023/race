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

def process_lidar_scan(data):
    # Extract LiDAR data
    ranges = data.ranges

    # Identify and process disparities
    for i in range(1, len(ranges)):
        # Calculate the difference between subsequent points
        disparity = abs(ranges[i] - ranges[i - 1])

        # Check if the disparity is larger than the threshold
        if disparity > threshold:
            # Determine the closer distance in the disparity
            desired_distance = min(ranges[i], ranges[i-1]) 
            # Calculate the number of LIDAR samples needed at closer distance
            num_samples_needed = calculate_samples_needed(desired_distance, ranges.angle_increment)

            # Overwriting the needed number of samples with the smaller value
            if desired_distance == ranges[i-1]:
                for j in range(i, i+num_samples_needed):
                    ranges[j] =  min(ranges[j],ranges[i-1])
            else:
                for j in range(i-1, i-num_samples_needed-1, -1):
                    ranges[j] =  min(ranges[j],ranges[i])




def main():
    rospy.init_node('follow_the_gap', anonymous=True)
    rospy.Subscriber('/car_8/laser/scan', LaserScan, process_lidar_scan)
    rospy.spin()

if __name__ == '__main__':
    main()
