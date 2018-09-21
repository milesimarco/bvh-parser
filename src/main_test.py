from bvh import *
from bvh_extend import *
import datetime
import os.path
import numpy as np
from functions import *
from transforms3d.euler import euler2mat
from transforms3d.affines import compose
from array import array

f=open(os.path.relpath('2017-12-22_16-22-35.bvh', start=os.curdir))
#f=open(os.path.relpath('default_ipi_rig.bvh', start=os.curdir))
data = BvhCalculator(f.read())
np.set_printoptions(suppress=True)

frame_index = 0

# matrice di traslazione hip
trasl_hip = np.identity(4)
trasl_hip[:, 3] = np.append( data.get_hip_traslation( 0 ), [1] )
print( "Hip trasl")
print ( str(trasl_hip))

# matrice di rotazione hip
Xr = data.frame_joint_channel( frame_index, "Hip", "Xrotation")
Yr = data.frame_joint_channel( frame_index, "Hip", "Yrotation")
Zr = data.frame_joint_channel( frame_index, "Hip", "Zrotation")
rot_hip = data.get_rotation_matrix2( Xr, Yr, Zr)
print( "Hip rot" )
print( str(rot_hip) )

# rototrasl HIP
print( "##### Hip rototrasl")
rt_hip = np.dot( trasl_hip, rot_hip)
print( rt_hip )


# traslazione lowerspine
trasl_lowerspine = np.identity(4)
trasl_vector = np.array( [0, 9.3027, -5.0341] ) + np.array( [0.2181, 43.4415, 2.8116] )
trasl_lowerspine[:, 3] = np.append( trasl_vector, [1] )
trasl_lowerspine[:, 3] = [0, 9.3027, -5.0341, 1]
print( "LowerSpine trasl")
print ( str( trasl_lowerspine ))
#trasl_matrix = np.dot( trasl_matrix, hip_roto)

# matrice di rotazione lowerspine
Xr = data.frame_joint_channel( frame_index, "LowerSpine", "Xrotation")
Yr = data.frame_joint_channel( frame_index, "LowerSpine", "Yrotation")
Zr = data.frame_joint_channel( frame_index, "LowerSpine", "Zrotation")
rot_lowerspine = data.get_rotation_matrix2( Xr, Yr, Zr)
print( "LowerSpine rot" )
print( str(rot_lowerspine) )

# rototrasl
print( "LowerSpine rototrasl")
rt_lowerspine = np.dot( trasl_lowerspine, rot_lowerspine)
print( str( rt_lowerspine ) )

# rototrasl hip * rototrasl lowerspine
print( "roto hip * roto lowerspine")
print( np.dot( rt_hip, rt_lowerspine ) )

print( "START")
rot_identity = np.eye(4)
print( str(rot_identity) )
print( str(rot_hip))
M = np.dot( trasl_hip, rot_identity, rot_hip)
print( "### MAGICO GRUPPO ###" )
print( "Hip RT" )
print( str(M) )
print( "LowerSpine RT")
N = np.dot( trasl_lowerspine, rot_identity, rot_lowerspine)
print( str(trasl_lowerspine))
print( str(rot_identity))
print( str(rot_lowerspine))
print( str( np.dot( M, N)))

raise SystemError
passo_1 = np.dot( rot_lowerspine, rot_hip )
print( str(passo_1))
# aggiunto position channel a offset
passo_2 = np.array( [0, 9.3027, -5.0341] ) + np.array( [0.2181, 43.4415, 2.8116] )
passo_2 = np.array( [0, 9.3027, -5.0341] ) 
passo_2 = np.append( passo_2, [1] )
print( str(passo_2 ))
passo_3 = np.dot( passo_2, rot_hip)
print( str(passo_3))
passo_4 = passo_3  + np.array( [0.2181, 43.4415, 2.8116, 1] )
print( passo_4 )

# aggiunto al position channel il calore dell'offset
raise SystemError

joint_name = "LowerSpine"
Xr = data.frame_joint_channel( frame_index, joint_name, "Xrotation")
Yr = data.frame_joint_channel( frame_index, joint_name, "Yrotation")
Zr = data.frame_joint_channel( frame_index, joint_name, "Zrotation")
RR = data.get_rotation_matrix( Xr, Yr, Zr)
R = np.dot(R, RR)
print( "Hip * LowerSpine rot")
print( str(R) )

# xposition e lo sommo all'offset

Xr = data.frame_joint_channel( frame_index, joint_name, "Xposition")
Yr = data.frame_joint_channel( frame_index, joint_name, "Yrotation")
Zr = data.frame_joint_channel( frame_index, joint_name, "Zrotation")



matrix = np.identity(4)
#matrix[:, 3] = np.append( data.joint_offset( joint_name ), [1] )
matrix[:, 3] = np.append( data.get_hip_traslation( frame_index ), [1] )
print(str(matrix))
print( str( np.dot(R, matrix)))