#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import tf
import numpy as np
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path


# Topic for visualizing the path in Rviz
path_pub        = rospy.Publisher('/trajectory_builder/visualize_path', Path, queue_size = 1)
path            = Path()
visualize_resolution = 0.5

global prev_x
global prev_y

plan            = []
frame_id        = 'map'
seq             = 0
prev_x          = 0.0
prev_y          = 0.0
path_resolution = 0.1


# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
#car_name            = str(sys.argv[1])
car_name            = 'car_8'
#trajectory_name     = str(sys.argv[1])
trajectory_name     = 'raceline204'

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)

arrow_pub = rospy.Publisher("/arrow_marker123", Marker, queue_size = 2)
arrow_pub2 = rospy.Publisher("/alpha_marker", Marker, queue_size = 2)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq = 0
control_polygon = PolygonStamped()

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/nvidia/depend_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
        dx = plan[index][0] - plan[index-1][0]
        dy = plan[index][1] - plan[index-1][1]
        path_resolution.append(math.sqrt(dx*dx + dy*dy))

# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global seq
    global wp_seq
    global curr_polygon

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y

    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    min_dist = float('inf')
    min_index = 0
    for index, point in enumerate(plan):
        dx = odom_x - point[0]
        dy = odom_y - point[1]
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < min_dist:
            min_dist = distance
            min_index = index

    base_projection_index = min_index
    base_projection_point = plan[base_projection_index]

    # Calculate heading angle of the car (in radians)
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = 2.825  # Adjust this value as needed
    steering_multiplier = 7.15
    #steering_multiplier = 1

    # good was steering_multiplier = 3.3
    #          angle_abs > 70
    #          lookahead_distance = 1.6
    #          speed = [25, 55]

    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.
    #target_point_index = min(base_projection_index + int(lookahead_distance / sum(path_resolution)), len(plan) - 1)
    def distFinder(baseIndex):
        dx = odom_x - plan[baseIndex][0]
        dy = odom_y - plan[baseIndex][1]
        distance = math.sqrt(dx*dx + dy*dy)
        return distance
        

    # target_point_index = base_projection_index
    # while distFinder(target_point_index) < lookahead_distance:
    #     target_point_index+=1
    # target_point_index-=1

    target_point_index = base_projection_index
    total_distance = 0
    #while total_distance < lookahead_distance and len(plan) > target_point_index + 1:
    while total_distance < lookahead_distance:
        if target_point_index == len(plan)-1:
            target_point_index = 0
        dx = plan[target_point_index + 1][0] - plan[target_point_index][0]
        dy = plan[target_point_index + 1][1] - plan[target_point_index][1]
        total_distance += math.sqrt(dx*dx + dy*dy)
        target_point_index+=1
    target_point_index-=1

    #target_point_index = min(base_projection_index + int(lookahead_distance / (sum(path_resolution) / len(path_resolution))), len(plan) - 1)
    target_point = plan[target_point_index]

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here
    alpha = math.atan2(target_point[1] - odom_y, target_point[0] - odom_x) - heading
    #alpha = math.atan((target_point[1] - odom_y)/(target_point[0] - odom_x)) - heading
    # print("alpha1: ", math.degrees(alpha))
    # if alpha > np.pi / 2:
    #     print("alpha larger than 90, adding pi to alpha")
    #     alpha -= np.pi
    # elif alpha < -np.pi / 2:
    #     print("alpha smaller than 90, adding pi to alpha")
    #     alpha += np.pi
    # elif alpha > np.pi:
    #     print("alpha > 180, subtracting pi to alpha")
    #     alpha -= np.pi
    # elif alpha < 0:
    #     print("alpha < 0, adding pi to alpha")
    #     alpha += np.pi
    print("alpha: ", math.degrees(alpha))

    # alpha *= -1

    #alpha = math.asin(abs(target_point[1] - odom_y) / lookahead_distance)
    L = WHEELBASE_LEN
    #steering_angle = math.atan2(2.0 * L * math.sin(alpha), lookahead_distance)
    steering_angle =  math.atan((2.0 * L * math.sin(alpha)) / lookahead_distance) * steering_multiplier

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here    
    command.steering_angle = max(-STEERING_RANGE, min(steering_angle, STEERING_RANGE))

    #command.steering_angle = math.degrees(command.steering_angle) * 1.3
    # command.steering_angle = math.degrees(command.steering_angle)
    #command.steering_angle = math.degrees(alpha)

    arrow_marker2 = Marker()
    arrow_marker2.header.frame_id = "car_8_laser"
    arrow_marker2.type = 0
    arrow_marker2.header.stamp = rospy.Time.now()
    arrow_marker2.id = 1
    arrow_marker2.scale.x = 1.2
    arrow_marker2.scale.y = 0.1
    arrow_marker2.scale.z = 0.1
    quaternion = quaternion_from_euler(0, 0, alpha)
    arrow_marker2.pose.orientation.x = quaternion[0]
    arrow_marker2.pose.orientation.y = quaternion[1]
    arrow_marker2.pose.orientation.z = quaternion[2]
    arrow_marker2.pose.orientation.w = quaternion[3]
    arrow_marker2.color.r = 0.0
    arrow_marker2.color.g = 0.0
    arrow_marker2.color.b = 1.0
    arrow_marker2.color.a = 1.0
    arrow_pub2.publish(arrow_marker2)

    arrow_marker = Marker()
    arrow_marker.header.frame_id = "car_8_laser"
    arrow_marker.type = 0
    arrow_marker.header.stamp = rospy.Time.now()
    arrow_marker.id = 1
    arrow_marker.scale.x = 1.2
    arrow_marker.scale.y = 0.1
    arrow_marker.scale.z = 0.1
    quaternion = quaternion_from_euler(0, 0, command.steering_angle)
    arrow_marker.pose.orientation.x = quaternion[0]
    arrow_marker.pose.orientation.y = quaternion[1]
    arrow_marker.pose.orientation.z = quaternion[2]
    arrow_marker.pose.orientation.w = quaternion[3]
    arrow_marker.color.r = 1.0
    arrow_marker.color.g = 0.0
    arrow_marker.color.b = 1.0
    arrow_marker.color.a = 1.0
    arrow_pub.publish(arrow_marker)

    command.steering_angle = math.degrees(command.steering_angle)

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    curvature = math.tan(command.steering_angle) / WHEELBASE_LEN
    angle_abs = abs(command.steering_angle)
    max_speed = 100
    min_speed = 45
    
    if angle_abs > 70:
        # vel_input = min_speed
        vel_input = max_speed - ((max_speed - min_speed) * (angle_abs / 100.0))
        print("high corner")
        # command.steering_angle *= 1.2
        # command.steering_angle = 100
    else:
        vel_input = max_speed - ((max_speed - min_speed) * (angle_abs / 100.0))

    vel_input = max(min(vel_input, max_speed), min_speed)

    print("steering angle: ", (command.steering_angle))
    
    #normalized_speed = (command.steering_angle - (-100)) / (100 - (-100))
    #normalized_speed = normalized_speed * (max_speed-min_speed) + min_speed

    # # normalized_speed = (max_speed - min_speed) / ((100 - (-100)) * (command.steering_angle - 100) + max_speed)
    # vel_input = normalized_speed

    # vel_input = max_speed - (angle_abs * 0.3)
    # vel_input = max(min_speed, vel_input)
    # else:
        # max_speed = 40.0  # Adjust this value as needed
        # min_speed = 15.0   # Adjust this value as needed
        # a = 10
        # b = -5
        # vel_input = (max_speed - min_speed) / (1+math.exp(a* abs(command.steering_angle)+b)) + min_speed
        # vel_input = max(min(vel_input,100),0)
        

    command.speed = vel_input
    command_pub.publish(command)
    print("steering speed", vel_input)

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    path.header.seq = seq
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = frame_id

    # These are set to zero only so that the template code builds. 
    pose_x = base_projection_point[0]
    pose_y = base_projection_point[1]
    target_x = target_point[0]
    target_y = target_point[1]

    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)

    path_pub.publish(path)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous=True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

            # Populate the Path message with waypoints for visualization
        if len(plan) > 0:
            for index in range(0, len(plan)):
                waypoint = PoseStamped()
                waypoint.header.frame_id    = frame_id
                waypoint.pose.position.x    = plan[index][0]
                waypoint.pose.position.y    = plan[index][1]
                waypoint.pose.orientation.z = plan[index][2]
                waypoint.pose.orientation.w = plan[index][3]
                path.poses.append(waypoint)

        path_pub.publish(path)


        # This node subscribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
