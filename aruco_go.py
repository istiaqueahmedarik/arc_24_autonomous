#!/usr/bin/env python


# this is for aruco detection of zed camera

import math
import rospy
from detect_aruco.msg import Num
from aruco_go.msg import wheel
from std_msgs.msg import String
import time
from sensor_msgs.msg import LaserScan

def map_(x):
    return (abs(x) * 200) / 350
pub1 = rospy.Publisher('wifi', String, queue_size=10)
entrance = False
rotate = False
findOnce = False
# Callback function to process incoming messages
last_msg_time = None
tups = False
OBSTACLE_AVOIDANCE_FLAG = False

def obstacleAvoidance():
    OBSTACLE_AVOIDANCE_FLAG=True
    rospy.loginfo("obstacle avoid mode")
    neutral_speed = 1500
    forward_speed = 1750  
    backward_speed = 1250  
    front_distance = lidar_dist[0]
    left_distance =  lidar_dist[90]
    left_corner_distance = lidar_dist[45]
    for i in range(45):
        left_corner_distance = min(left_corner_distance,lidar_dist[i])
    right_distance = lidar_dist[-90]
    right_corner_distance = lidar_dist[-45]

    for i in range(45):
        right_corner_distance = min(right_corner_distance,lidar_dist[-1*i])
    
    if(left_distance == float('inf')):
        left_distance = 0.1
    if(right_distance == float('inf')):
        right_distance = 0.1
    if(left_corner_distance == float('inf')):
        left_corner_distance = 0.1
    if(right_corner_distance == float('inf')):
        right_corner_distance = 0.1
    l_speed = neutral_speed
    r_speed = neutral_speed
    if(lidar_dist[0]<1):
        if(left_distance>right_distance):
            publish_custom_data((1800,1100))
            return
        else:
            publish_custom_data((1100,1800))
            return
    if  left_corner_distance < 1.3 and right_corner_distance >1.3:
        l_speed = 1800
        r_speed = 1100
    elif  right_corner_distance < 1 and left_corner_distance >1:
        l_speed = 1100
        r_speed = 1800
    elif( left_corner_distance>1.8 and right_corner_distance>1.8):
        if left_corner_distance < right_corner_distance:
            speed_diff = int((right_corner_distance - left_corner_distance) * 500)  
            l_speed = forward_speed
            r_speed = max(neutral_speed, forward_speed - speed_diff)
        else:
            speed_diff = int((left_corner_distance - right_corner_distance) * 500)  
            l_speed = max(neutral_speed, forward_speed - speed_diff)
            r_speed = forward_speed
        
    elif left_distance < right_distance:
        speed_diff = int((right_distance - left_distance) * 500)  
        l_speed = forward_speed
        r_speed = max(neutral_speed, forward_speed - speed_diff)
    else:
        speed_diff = int((left_distance - right_distance) * 500)  
        l_speed = max(neutral_speed, forward_speed - speed_diff)
        r_speed = forward_speed
    
    publish_custom_data((l_speed,r_speed))

lidar_dist = {}
def lidar_callback(msg):
    global entrance
    global lidar_dist
    for i, distance in enumerate(msg.ranges):
        degree = math.degrees(msg.angle_min + i * msg.angle_increment)
        lidar_dist[round(degree)] = distance
    obstacleAvoidance()
    
 
    
 

def publish_custom_data(data):
    pub1.publish('connected')

    # Create a publisher object
    pub = rospy.Publisher('joystick', String, queue_size=10)

    # Create a custom message
    custom_msg = wheel()
    custom_msg.left = data[0]
    custom_msg.right = data[1]
    wh = "[0" + str(data[0]) + ",1" + str(data[1]) + "]"

    # Publish the custom message
    pub.publish(wh)


def listener():
    # Initialize the ROS node
    rospy.init_node('aruco_go', anonymous=True)
    pub1.publish('connected')
    
    # Create a subscriber object
    # rospy.Subscriber("aruco_info", Num, callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    
    # Create a timer to publish default values if no message is received
    # rospy.Timer(rospy.Duration(0.01), no_msg_timer)  # Adjust the timer period as needed

    # Spin and process incoming messages
    rospy.spin()

if __name__ == '__main__':
    listener()