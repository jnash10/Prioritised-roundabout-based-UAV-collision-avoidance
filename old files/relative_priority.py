
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from to_goal import angle_to_goal, go_to_goal
import numpy as np
import matplotlib.pyplot as plt
import time
from tf.transformations import euler_from_quaternion


timefile = open('time.csv','w')
timefile.write('name,time,priority\n')
node = rospy.init_node("avoid_cllision")
rate = rospy.Rate(100)
detection_distance = 100
safe_distance = 2
maneuver_distance = 5
class Uav:
    def __init__(self,name,v,priority,goal):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = v
        self.priority=priority
        self.name=name
        self.pub = rospy.Publisher(str('/'+name+'/cmd_vel'), Twist, queue_size=10)
        self.sub = rospy.Subscriber(str('/'+name+'/ground_truth_to_tf/pose'), PoseStamped, self.poseCallback)
        self.msg = Twist()
        self.goal = goal
        self.file = open(name+'.csv','w')
        self.time = 0
        self.check=True
        self.vreal = 0
        

    def poseCallback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.yaw = euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])[2]

    def go_to_goal(self, goal): #return velocity vector for goal
        dist = np.linalg.norm((goal[0] - self.x, goal[1]-self.y))
        if dist<1:
            return np.array([0,0])
        else:
            angle = np.arctan2(goal[1]-self.y,goal[0]-self.x) 
            return min(self.v,dist)*np.array([np.cos(angle),np.sin(angle)])

    def avoid_collision(self, drones): 
        vec = np.array([0,0],dtype='float64')
        for drone in drones:
            if drone.name != self.name:
                if np.linalg.norm((drone.x-self.x,drone.y-self.y))<detection_distance:
                    if self.zem(drone) < safe_distance:
                        angle = np.arctan2(drone.y-self.y,drone.x-self.x) 
                        curvec = ((drone.priority/self.priority)**2)*np.array([np.cos(angle),np.sin(angle)])/np.linalg.norm((drone.x-self.x,drone.y-self.y))**2
                        tangential_vec = ((drone.priority/self.priority)**2)*np.array([np.sin(angle),-np.cos(angle)])
                        vec += self.v*(tangential_vec + curvec)/(self.priority/drone.priority)**2
        return vec

    def t_go(self,drone):
        v1 = self.v*np.array([np.cos(self.yaw),np.sin(self.yaw)])
        v2 = drone.v*np.array([np.cos(drone.yaw),np.sin(drone.yaw)])

        p1 = np.array([self.x,self.y])
        p2 = np.array([drone.x,drone.y])

        t = -np.dot((v1-v2),(p1-p2))/np.dot((v1-v2),(v1-v2))
        return t

    def zem(self, drone):
        print(self.name, self.yaw)
        v1 = self.vreal
        v2 = drone.vreal


        p1 = np.array([self.x,self.y])
        p2 = np.array([drone.x,drone.y])

        t = -np.dot((v1-v2),(p1-p2))/np.dot((v1-v2),(v1-v2))
        
        s = (p1-p2) + t*(v1-v2)

        return np.linalg.norm(s)


    def note_pos(self):
        self.file.write(str(self.x)+","+str(self.y)+"\n")

    def move(self, drones): 
        goal = self.goal
        #add the results obtained from avoid_collision and go_to_goal and publish them
        #also correct for yaw
        vec = self.go_to_goal(goal) - self.avoid_collision(drones)
        
        angle = np.arctan2(vec[1],vec[0]) - self.yaw
        
        norm = np.linalg.norm(vec)
        self.msg.linear.x = min(norm, self.v)*np.cos(angle)
        self.msg.linear.y = min(norm, self.v)*np.sin(angle)
        self.vreal=np.array([self.msg.linear.x,self.msg.linear.y])
        print(self.name,self.msg.linear.x,self.msg.linear.y)
        self.pub.publish(self.msg)

    def check_end(self):
        if self.msg.linear.x==0 and self.msg.linear.y==0 and self.check==True:
            self.time = (rospy.Time.now()-self.time).secs
            timefile.write(str(self.name+','+str(self.time)+','+str(self.priority)+'\n'))
            self.check=False
        

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

def create_points():
    radius = 100
    n = int(input("how many drones you want? : "))
    
    goals = []
    step = 2*np.pi/n
    for i in range(n):
        theta = i*step
        #goals.append((-radius*np.cos(theta),-radius*np.sin(theta)))
        noise = np.random.normal(loc=(0),scale=np.pi/3)
        noise = 0
        goals.append((radius*-np.cos(theta+noise), radius*-np.sin(theta+noise)))
    return goals
        
def create_drones(goals):
    drones = []
    for i in range(len(goals)):
        drones.append(Uav(f'uav{i+1}', 10, 1, goals[i]))
    return drones
    

if __name__ == '__main__':
    goals = create_points()

    drones = create_drones(goals)
    drones[0].priority=3


    take_off(drones)


    for drone in drones:
        drone.time = rospy.Time.now()

    while True:
        for drone in drones:
            drone.move(drones)
            drone.note_pos()
            drone.check_end()
        rate.sleep()   

