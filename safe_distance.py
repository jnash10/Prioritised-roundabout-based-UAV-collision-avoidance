
from ast import Del
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.srv import DeleteModel, SpawnModel

from to_goal import angle_to_goal, go_to_goal
import numpy as np
import matplotlib.pyplot as plt
import time
from tf.transformations import euler_from_quaternion
import os



fname = input("filename: ")
os.mkdir('outputs/'+fname)
timefile = open('time.csv','w')
timefile.write('name,time,priority\n')
node = rospy.init_node("avoid_cllision")
rate = rospy.Rate(100)
detection_distance = 100
safe_distance = 2
maneuver_distance = 40
threshold=0.2
k=2*10**4
class Uav:
    def __init__(self,name,v,priority,goal):
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = v
        self.priority=priority
        self.name=name
        self.pub = rospy.Publisher(str('/'+name+'/cmd_vel'), Twist, queue_size=100)
        self.sub = rospy.Subscriber(str('/'+name+'/ground_truth_to_tf/pose'), PoseStamped, self.poseCallback)
        self.msg = Twist()
        self.goal = goal
        self.file = open('outputs/'+fname+'/'+name+'.csv','w')
        self.time = 0
        self.check=True
        self.vreal = np.array([0,0])
        self.ideal = 0
        

    def poseCallback(self, data):
        self.x = data.pose.position.x
        self.y = data.pose.position.y
        self.yaw = euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])[2]

    def go_to_goal(self, goal): #return velocity vector for goal
        dist = np.linalg.norm((goal[0] - self.x, goal[1]-self.y))
        if dist<1:
            return np.array([0,0])
        else:
            f_v = self.v*np.array([self.goal[0]-self.x, self.goal[1]-self.y])
            return f_v

    def avoid_collision(self, drones): 
        vec = self.vreal
        vec = np.array([0,0])
        for drone in drones:
            if drone.name != self.name:
                if np.linalg.norm((drone.x-self.x,drone.y-self.y))<detection_distance:
                    if self.zem(drone) < safe_distance and self.t_go(drone)>=0:
                        angle = np.arctan2(drone.y-self.y,drone.x-self.x) 

                        dist = np.linalg.norm((drone.x-self.x,drone.y-self.y))
                        f_v = (k/dist**3)*(1/(dist-safe_distance) - 1/detection_distance)*np.array([drone.x-self.x, drone.y-self.y])
                        theta = np.pi/2
                        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                        f_v_ = np.dot(rot, f_v)
                        del_v = ((drone.priority/self.priority)**2)*(f_v + f_v_)

                        vec = vec  + self.v*del_v
        return vec

    def t_go(self,drone):
        v1 = self.v*np.array([np.cos(self.yaw),np.sin(self.yaw)])
        v2 = drone.v*np.array([np.cos(drone.yaw),np.sin(drone.yaw)])

        p1 = np.array([self.x,self.y])
        p2 = np.array([drone.x,drone.y])

        t = -np.dot((v1-v2),(p1-p2))/np.dot((v1-v2),(v1-v2))
        return t

    def zem(self, drone):
        #print(self.name, self.yaw)
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
        if self.avoid_collision(drones)[0]<threshold and self.avoid_collision(drones)[1]<threshold:
            vec = self.go_to_goal(goal)
        else:
            vec = -self.avoid_collision(drones)

        
        angle = np.arctan2(vec[1],vec[0]) - self.yaw
        
        norm = np.linalg.norm(vec)
        self.msg.linear.x = min(norm, self.v)*np.cos(angle)
        self.msg.linear.y = min(norm, self.v)*np.sin(angle)
        self.vreal=np.array([self.msg.linear.x,self.msg.linear.y])
        #print(self.name,self.msg.linear.x,self.msg.linear.y)
        self.pub.publish(self.msg)




    def check_end(self, drones):
        if (np.linalg.norm(np.array([self.goal[1]-self.y,self.goal[0]-self.x]))<=2) and self.check==True:
            #self.time = (rospy.Time.now()-self.time).secs
            self.time = (rospy.Time.now()-self.time).to_sec()
            print(self.time)
            timefile.write(str(self.name+','+str(self.time)+','+str(self.priority)+'\n'))
            self.check=False
            drones.remove(self)
            delete_model_prox(self.name) 

        
delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)



#spawn_model_prox = rospy.ServiceProxy(str((str(np.random))+'uav'), '/media/storage/agam/hector_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_gazebo/launch/spawn_quadrotor.launch',0,0,"world" )
def take_off(drones):
    # msg = Twist()
    
    # #rate = rospy.Rate(10)
    
    # for drone in drones:
    #     msg.linear.z=20
    #     for i in range(15):
    #         drone.pub.publish(msg)
    #         rate.sleep()
    #     msg.linear.z=0
    #     drone.pub.publish(msg)
    msg = Twist()
    msg.linear.z=1
    #rate = rospy.Rate(10)
    for i in range(300):
        for drone in drones:
            drone.pub.publish(msg)
        rate.sleep()

    #tell it to stop now
    msg.linear.z=0
    for drone in drones:
        drone.pub.publish(msg)
    #time.sleep(3)
    


def create_points():
    radius = 100
    n = int(input("how many drones you want? : "))
    
    goals = []
    step = 2*np.pi/n
    for i in range(n):

        #goals.append((-radius*np.cos(theta),-radius*np.sin(theta)))
        theta = np.random.uniform(low=-np.pi, high = np.pi)
        #noise = 0
        goals.append((radius*-np.cos(theta), radius*-np.sin(theta)))
    return goals

np.random.seed(42)
def create_drones(goals):
    drones = []
    for i in range(len(goals)):
        drones.append(Uav(f'uav{i+1}', 5, np.random.normal(loc=(3.5), scale=1.2), goals[i]))
        #drones.append(Uav(f'uav{i+1}', 5, 3, goals[i]))
        #drones.append(Uav(f'uav{i+1}', 5, (i%5)+1, goals[i]))
        rate.sleep()
    return drones
    

if __name__ == '__main__':
    goals = create_points()

    drones = create_drones(goals)
    take_off(drones)


    for drone in drones:
        drone.time = rospy.Time.now()

    while True:
        if drones == []:
            print("simulation complete")
            break
        for drone in drones:
            drone.move(drones)
            print(drone.name, drone.vreal)
            drone.note_pos()
            drone.check_end(drones)
        rate.sleep()
        

