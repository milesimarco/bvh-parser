from bvh import *
from bvh_extend import *
import datetime
import os.path
import numpy as np
from functions import *
from transforms3d.euler import euler2mat
from transforms3d.affines import compose
from array import array
from estensione import *

np.set_printoptions(suppress=True)

f=open(os.path.relpath('2017-12-22_16-22-35.bvh', start=os.curdir))
#f=open(os.path.relpath('default_ipi_rig.bvh', start=os.curdir))
data = BvhCalculator(f.read())
#np.set_printoptions(suppress=True)

frame_index = 0
Xr = data.frame_joint_channel( frame_index, "Hip", "Xrotation")
Yr = data.frame_joint_channel( frame_index, "Hip", "Yrotation")
Zr = data.frame_joint_channel( frame_index, "Hip", "Zrotation")

hip_rotation = euler_matrix( deg2rad(Xr), deg2rad(Yr), deg2rad(Zr), "rxyz" )
identity = np.eye(4)

trasl_hip = np.identity(4)
trasl_hip[:, 3] = np.append( data.get_hip_traslation( 0 ), [1] )

M_HIP = np.matmul( trasl_hip, hip_rotation )
print( str( M_HIP ) )
print("#########")

Xr = data.frame_joint_channel( frame_index, "LowerSpine", "Xrotation")
Yr = data.frame_joint_channel( frame_index, "LowerSpine", "Yrotation")
Zr = data.frame_joint_channel( frame_index, "LowerSpine", "Zrotation")
LowerSpine_rotation = euler_matrix( deg2rad(Xr), deg2rad(Yr), deg2rad(Zr), "rxyz" )

trasl_lowerspine = np.identity(4)
trasl_lowerspine[:, 3] = [0, 9.3027, -5.0341, 1]

M_LOWERSPINE = np.matmul( M_HIP, np.matmul( trasl_lowerspine, LowerSpine_rotation) )
print( str( M_LOWERSPINE ) )
print("#########")

Xr = data.frame_joint_channel( frame_index, "MiddleSpine", "Xrotation")
Yr = data.frame_joint_channel( frame_index, "MiddleSpine", "Yrotation")
Zr = data.frame_joint_channel( frame_index, "MiddleSpine", "Zrotation")
MiddleSpine_rotation = euler_matrix( deg2rad(Xr), deg2rad(Yr), deg2rad(Zr), "rxyz" )

trasl_middlespine = np.identity(4)
trasl_middlespine[:, 3] = [0.0000, 8.4894, 0.4435, 1]

M_MIDDLESPINE = np.matmul( M_LOWERSPINE, np.matmul( trasl_middlespine, MiddleSpine_rotation ) )
print( str( M_MIDDLESPINE ) )
print("#########")
