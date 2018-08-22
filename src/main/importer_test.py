from bvh import Bvh, BvhNode
from functions import *

f =  open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\bvh_carrozzina.bvh')
#f =  open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\test_freebvh.bvh')
mocap = Bvh(f.read())
print( mocap.get_joints_names() )

joint_names = mocap.get_joints_names();
print( len(joint_names) );
print( "#########" )

i = 0
while i < len( joint_names ):
    
    joint_name = joint_names[i]
    joint_index = mocap.get_joint_index( joint_name ) # Indice del joint
    joint_channels = mocap.joint_channels( joint_name ) # Array con i nomi dei joint
    frames_joint_channels = mocap.frames_joint_channels( joint_name, joint_channels );
    joint_offset = mocap.joint_offset(joint_names[i]);
    
    print( "\n" + joint_name + " - " + str( joint_index ) )
    print( joint_channels )
    #print (mocap.frames_joint_channels( joint_name, joint_channels ) )
    #print( mocap.frame_joint_channels(0, joint_name, joint_channels ) )
    
    eulero_angles( 0, joint_channels, frames_joint_channels )
    print( "Offset: " + str( joint_offset ) )
    i += 1