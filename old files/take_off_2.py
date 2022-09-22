#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from to_goal import go_to_goal
import numpy as np

#initialise the node
rospy.init_node('collision_avoider')

#define the message format for velocity
msg = Twist()
#rate of publishing messages
rate = rospy.Rate(1)

#define the velocity publishers for each drone
pub1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/uav2/cmd_vel', Twist, queue_size=1)


#takeoff function
def takeoff():
    #takeoff velocity
    msg.linear.z=1

    #publish takeoff message
    for i in range(5):
        pub1.publish(msg)
        pub2.publish(msg)
        rate.sleep()

    #tell it to stop now
    msg.linear.z=0
    pub1.publish(msg)
    pub2.publish(msg)



#takeoff
takeoff()

def move_1(pos,goal=(0,15)):
    pos = (pos.pose.position.x,pos.pose.position.y)
    velx, vely =  go_to_goal(pos,goal)
    msg.linear.x = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5)* velx
    msg.linear.y = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5)* vely

    pub1.publish(msg)

def move_2(pos,goal=(-15,0)):
    pos = (round(pos.pose.position.x,2),round(pos.pose.position.y,2))
    velx, vely = go_to_goal(pos,goal)
    msg.linear.x = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5)* velx
    msg.linear.y = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5)* vely

    pub2.publish(msg)

    



#define position subscribers for each drone
sub1 = rospy.Subscriber('/uav1/ground_truth_to_tf/pose', PoseStamped, move_1)
sub1 = rospy.Subscriber('/uav2/ground_truth_to_tf/pose', PoseStamped, move_2)










rospy.spin()











