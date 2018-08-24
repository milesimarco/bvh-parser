from bvh import Bvh, BvhNode
import numpy as np
#import math
   
def degree_to_rad(value):
    return np.deg2rad(value)

#def degree_to_rad(value):  #provo a velocizzare
#   return math.radians(value);

def print_status(actual, total):  # actual : total = % : 100
    print("############################## " + str(int(round(actual * 100 / total))) + "% ##########")
    
def calculate_Rzyx(RX, RY, RZ):
    return np.dot(RZ, RY, RX)