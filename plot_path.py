import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from to_goal import angle_to_goal, go_to_goal
import numpy as np
import matplotlib.pyplot as plt


node = rospy.init_node("straight_flying")

msg = Twist()
pub1 = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)


def takeoff():
    #takeoff velocity
    msg.linear.z=1

    #publish takeoff message
    for i in range(15):
        pub1.publish(msg)
        rate.sleep()

    #tell it to stop now
    msg.linear.z=0
    pub1.publish(msg)

#takeoff
takeoff()


goals = [(0,15),(0,-15)]

def move_1(pos,goal):
    pos = (pos.pose.position.x,pos.pose.position.y)
    velx, vely =  go_to_goal(pos,goal)
    angle = angle_to_goal(pos, goal)
    msg.linear.x = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5) * velx
    
    msg.linear.y = min(np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])),5) * vely
    #msg.angular.z=angle

    pub1.publish(msg)
    print("position: ",pos, "goal: ",goal)
    print("distance: ",np.linalg.norm((goal[0]-pos[0],goal[1]-pos[1])), "angle: ",angle)
    print("velocity",round(msg.linear.x,2),round(msg.linear.y,2))
    #rate.sleep()


dir = 0
def switch(data):
    global dir
    goal = goals[dir]
    #if distance between position and goal <0.1 switch direction
    dist = np.linalg.norm((goal[0] - data.pose.position.x, goal[1] - data.pose.position.y))
    if dist<0.1:
        dir = (dir+ 1)%2
    move_1(data,goal)


sub1 = rospy.Subscriber('/uav1/ground_truth_to_tf/pose', PoseStamped, switch)

rospy.spin()