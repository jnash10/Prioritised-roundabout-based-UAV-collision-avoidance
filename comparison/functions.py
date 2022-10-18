import numpy as np
from variables import *

def tc(v1, v2):
    t = (v1.v - v2.v)*(v1.p-v2.p)/(v1.v - v2.v)**2

    return t

def ZEM(v1, v2):
    s = (v1.p - v2.p) + tc(v1, v2)*(v1.v - v2.v)
    return s

def dist(p1, p2):
    return np.linalg.norm(p1-p2)

def clip(vector, mag):
    mod = np.linlag.norm(vector)
    if mod<=mag:
        return vector
    else:
        return mag*vector/mod

def go_to_goal(v): #return velocity vector for go-to-goal
    if dist(v.p, v.g)<= rhonaught:
        return v.v + (v.kv*(v.g-v.p)/m)*dt #v_new = v_old + F/m * dt
    else:
        return v.vmax*(v.g - v.p)/np.linalg.norm(v.g-v.p)

 




