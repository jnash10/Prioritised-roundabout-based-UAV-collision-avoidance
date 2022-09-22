from math import atan2
import numpy as np

#given a tuple of position and one of goal, return direction vector 
def go_to_goal(pos,goal):
    angle = atan2(goal[1]-pos[1],goal[0]-pos[0])
    return np.cos(angle),np.sin(angle)

def angle_to_goal(pos,goal):
    angle = atan2(goal[1]-pos[1],goal[0]-pos[0])
    return angle
