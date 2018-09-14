from bvh import Bvh, BvhNode
import numpy as np
import math
   
def degree_to_rad(value): # deprecated
    return np.deg2rad(value)

#def degree_to_rad(value):  #provo a velocizzare
#   return math.radians(value);

def print_status(actual, total):  # actual : total = % : 100
    print("############################## " + str(int(round(actual * 100 / total))) + "% ##########")
    
def calculate_Rzyx(RX, RY, RZ):
    #return np.matmul(np.matmul(RZ, RY), RX)
    rotmat = np.eye(4)
    rotmat = np.matmul(RX, rotmat)
    rotmat = np.matmul(RY, rotmat)
    rotmat = np.matmul(RZ, rotmat)
    return rotmat
    #return np.matmul(np.matmul(RZ, RY), RX)
    
def deg2rad(x):
    return x/180*math.pi

def rad2deg(x):
    return x/math.pi*180
