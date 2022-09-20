from glob import glob
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
#from .test import poseCallback
from to_goal import angle_to_goal, go_to_goal
import numpy as np
import matplotlib.pyplot as plt
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler


node = rospy.init_node("deviationchecking")
x,y,yaw=0,0,0


pub = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)



def poseCallback(data):
    global x,y,yaw
    x = data.pose.position.x
    y = data.pose.position.y
    yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]

sub = rospy.Subscriber('/uav1/ground_truth_to_tf/pose', PoseStamped, poseCallback)


def go_to_goal(goal):
    global x,y,yaw    
    rate = rospy.Rate(10)
    msg = Twist()
    while True:
        note_pose()
        dist = np.linalg.norm((goal[0]-x,goal[1]-y))
        angle = np.arctan2(goal[1]-y,goal[0]-x) - yaw

        msg.linear.x = min(dist,5) * np.cos(angle)
        msg.linear.y = min(dist,5) * np.sin(angle)

        if dist<0.1:
            msg.linear.x=0
            msg.linear.y=0
            pub.publish(msg)
            break
        pub.publish(msg)
        rate.sleep()

def takeoff():
    #takeoff velocity
    msg = Twist()
    msg.linear.z=1
    rate = rospy.Rate(10)
    for i in range(15):
        pub.publish(msg)
        rate.sleep()

    #tell it to stop now
    msg.linear.z=0
    pub.publish(msg)
    time.sleep(3)

takeoff()

file = open('pose.csv','w')
def note_pose():
    global x,y,file
    line = str(x)+","+str(y)+"\n"
    file.write(line)

def stop():
    msg = Twist()
    msg.linear.x=0
    msg.linear.y=0
    pub.publish(msg)

goals = [(0,15),(0,0)]
for i in range(10):
    goal = goals[i%2]
    go_to_goal(goal)
    #note_pose()
stop()
file.close()


    

