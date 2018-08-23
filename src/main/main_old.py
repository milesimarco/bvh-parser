from bvh import *
from bvh_extend import *
from functions import *

f = open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\bvh_carrozzina.bvh')
# f =  open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\test_freebvh.bvh')
mocap = BvhCalculator(f.read())
print(mocap.get_joints_names())

# print( "TRASL " + str(eulero_angles( 0, joint_channels, frames_joint_channels )))
# mocap.set_offset_assoluto_all()
# print("Fine")
# raise SystemExit

joint_names = mocap.get_joints_names();
print(len(joint_names));
# print( "TRASL " + str(mocap.get_hip_traslation(1 )))

# mocap.calculate_joints_rotations_all_frames()
print("#########")

i = 0
while i < len(joint_names):
    
    joint_name = joint_names[i]
    joint_index = mocap.get_joint_index(joint_name)  # Indice del joint
    joint_channels = mocap.joint_channels(joint_name)  # Array con i nomi dei joint
    frames_joint_channels = mocap.frames_joint_channels(joint_name, joint_channels);
    joint_offset = mocap.joint_offset(joint_name);

    print("\n" + joint_name + " # " + str(joint_index))
    print(joint_channels)
    # print (mocap.frames_joint_channels( joint_name, joint_channels ) )
    # print( mocap.frame_joint_channels(0, joint_name, joint_channels ) )
    # print( "Offset assoluto di " + str(i) + ": " + str( mocap.offset_assoluto( joint_name, 1 ) ) )
    
    i = 0
    while i < mocap.nframes:
        eulero_angles(i, joint_channels, frames_joint_channels)
        
    print("Offset relativo di " + str(i) + ": " + str(joint_offset))
    i += 1
