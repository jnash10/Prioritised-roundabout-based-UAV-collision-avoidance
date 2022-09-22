
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from to_goal import angle_to_goal, go_to_goal
import numpy as np
import matplotlib.pyplot as plt
import time
from tf.transformations import euler_from_quaternion


node = rospy.init_node("avoid_cllision")
rate = rospy.Rate(10)
class Uav:
    
    def __init__(self,name,v,goal):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = v
        self.name=name
        self.pub = rospy.Publisher(str('/'+name+'/cmd_vel'), Twist, queue_size=10)
        self.sub = rospy.Subscriber(str('/'+name+'/ground_truth_to_tf/pose'), PoseStamped, self.poseCallback)
        self.msg = Twist()
        self.goal = goal
        self.file = open(name+'.csv','w')

    def poseCallback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.yaw = euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])[2]

    def go_to_goal(self, goal): #return velocity vector for goal
        dist = np.linalg.norm((goal[0] - self.x, goal[1]-self.y))
        if dist<1:
            return np.array([0,0])
        else:
            return self.v*np.array([goal[0]-self.x,goal[1]-self.y])/np.linalg.norm((goal[0]-self.x,goal[1]-self.y))
            #return self.v*np.array([goal[0]-self.x,goal[1]-self.y])
        #can remove distance minimum here since its being done later too

    def avoid_collision(self, drones): 
        #return velocity vector to avoid collision
        vec = np.array([0,0],dtype='float64')
        for drone in drones:
            if drone.name != self.name:
                if np.linalg.norm((drone.x-self.x,drone.y-self.y))<5:
                    curvec = drone.v*np.array([drone.x-self.x,drone.y-self.y])/(np.linalg.norm((drone.x-self.x,drone.y-self.y)))**2
                    tangential_vec = ((drone.v)**2)*np.array([curvec[1],-curvec[0]])
                    vec += (tangential_vec + curvec)/(self.v)**2
                    #vec += ((drone.v)**2)*(tangential_vec)/self.v
                    
                    #vec += curvec

        return vec

    def note_pos(self):
        self.file.write(str(self.x)+","+str(self.y)+"\n")

    

    def move(self, drones): 
        goal = self.goal
        #add the results obtained from avoid_collision and go_to_goal and publish them
        #also correct for yaw
        vec = self.go_to_goal(goal) - self.avoid_collision(drones)
        print(self.name,self.go_to_goal(goal) , self.avoid_collision(drones))
        
        angle = np.arctan2(vec[1],vec[0]) - self.yaw
        
        #print(np.linalg.norm((vec[0],vec[1])))
        self.msg.linear.x = min(abs(vec[0]),self.v)*np.cos(angle)
        self.msg.linear.y = min(abs(vec[1]),self.v)*np.sin(angle)
        self.pub.publish(self.msg)
        rate.sleep()

def take_off(drones):
    msg = Twist()
    msg.linear.z=1
    rate = rospy.Rate(10)
    for i in range(15):
        for drone in drones:
            drone.pub.publish(msg)
        rate.sleep()

    #tell it to stop now
    msg.linear.z=0
    for drone in drones:
        drone.pub.publish(msg)
    time.sleep(3)


drones = [Uav('uav1',3,(0,15)), Uav('uav2',2,(-15,0)), Uav('uav3',2,(15,0))]
#drones = [Uav('uav2',2,(-15,0)), Uav('uav3',2,(15,0))]
#drones = [Uav('uav1',3,(0,15)), Uav('uav2',2,(-15,0))]

if __name__ == '__main__':
    take_off(drones)
    while True:
        for drone in drones:
            drone.move(drones)
            drone.note_pos()
    
    rate.sleep()



        





