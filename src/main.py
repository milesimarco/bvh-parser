from bvh import *
from bvh_extend import *


f = open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\bvh_carrozzina.bvh')
data = BvhCalculator(f.read())

joint_names = data.get_joints_names();
print( "Nomi " + str( joint_names ) )

# VALUES TO CHANGE
joint_name = "MiddleSpine"
frame_index = 3

print ( "\n##### Calculating " + joint_name + " on index " + str(frame_index))
joint_channels = data.joint_channels(joint_name)  # Array con i nomi dei joint
print("Joint_name " + str(joint_name))
print("Joint channels" + str(joint_channels))
frame_joint_channels = data.frame_joint_channels(frame_index, joint_name, joint_channels);

print( "\nRotazione RZ * RY * RX")
eulero_angles = data.get_eulero_angles( frame_index, joint_channels, frame_joint_channels )
print( calculate_Rzyx( eulero_angles[0], eulero_angles[1], eulero_angles[2] ) )
print( "\nOffset assoluto")
print( data.get_offset_assoluto(frame_index, joint_name))

print( "\nHIP Traslation")
print( data.get_hip_traslation(frame_index) )

print( "\nRototraslation" )
rotation = data.get_rotation(frame_index, joint_channels, frame_joint_channels)
print( data.get_rototraslation(joint_name, frame_index, rotation) )


#raise SystemExit

print( "\n ##### Calculating rototraslations, all")

frame_index = 0
while frame_index < data.nframes:
    
    j = 0
    while j < len( joint_names ):
        joint_name = joint_names[j]
        joint_channels = data.joint_channels(joint_name)
    
        frame_joint_channels = data.frame_joint_channels(frame_index, joint_name, joint_channels)
        rotation = data.get_rotation(frame_index, joint_channels, frame_joint_channels)
        #print( data.get_rototraslation(joint_name, instant, rotation) )
        j += 1
        
    #raise SystemExit

    print( "Instant " + str(frame_index+1) + " ok")
    frame_index += 1
