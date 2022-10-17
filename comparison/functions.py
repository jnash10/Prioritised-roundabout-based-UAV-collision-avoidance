import numpy as np

def tc(v1, v2):
    t = (v1.v - v2.v)*(v1.p-v2.p)/(v1.v - v2.v)**2

    return t

def ZEM(v1, v2):
    s = (v1.p - v2.p) + tc(v1, v2)*(v1.v - v2.v)
    return s

def dist(p1, p2):
    return np.linalg.norm(p1-p2)

def go_to_goal(v):
    
