#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

rospy.init_node('movement')

cmd_vel_pub_1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
cmd_vel_pub_2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=1)
cmd_vel_pub_3 = rospy.Publisher('/uav3/cmd_vel', Twist, queue_size=1)

drones = [cmd_vel_pub_1,cmd_vel_pub_2,cmd_vel_pub_3]

rate = rospy.Rate(1)

twist_msg = Twist()
twist_msg.linear.x=10

for i in range(4):    
    for drone in drones:
        drone.publish(twist_msg)
    rate.sleep()


twist_msg.linear.y=10
twist_msg.linear.x=0
for i in range(4):    
    for drone in drones:
        drone.publish(twist_msg)
    rate.sleep()

twist_msg.linear.y=0
for drone in drones:
    drone.publish(twist_msg)

rospy.spin()


