from bvh import *
from bvh_extend import *
import datetime
import os.path

#f = open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\bvh_carrozzina.bvh')
#f = open(r'C:\Users\Gianb\Documents\GitHub\prova_bvh\src\tests\bvh_carrozzina.bvh') #lo esegue gianby


#f=open(os.path.relpath('bvh_carrozzina.bvh', start=os.curdir))
f=open(os.path.relpath('2017-12-22_16-22-35.bvh', start=os.curdir))
data = BvhCalculator(f.read())
print( data.nframes )
start = datetime.datetime.now()

# Calcola tutte le rototraslazioni
data.calculate_rototranslations()

end = datetime.datetime.now()
elapsed = end - start

print("Tempo impiegato: " + str(elapsed.seconds) + ":" + str(elapsed.microseconds)) 

#joint_names = data.get_joints_names()
#j = 0
#while j < len( joint_names ):
#    joint_name = joint_names[j]
#    x = data.get_joint(joint_name).rototranslation
#    print( joint_name + " " + str( len(x) ) + " " )
#    j += 1
        
# Test
print( data.get_joint("Hip").get_rototranslation(1) )
print( data.get_joint("Hip").get_position(1) )
print( data.get_joint("Hip").get_position(2) )
print( data.get_joint("Hip").get_position(3) )
print( data.get_joint("LowerSpine").get_position(3) )
print( data.get_joint("MiddleSpine").get_position(4) )

print( "last frame tests")
print( data.get_joint("Hip").get_position( data.nframes -1 ) )
print( data.get_joint("LowerSpine").get_position( data.nframes -1) )
print( data.get_joint("MiddleSpine").get_position( data.nframes -1) )
print( data.get_joint("Chest").get_position( data.nframes -1) )
print( data.get_joint("LToe").get_position( data.nframes -1) )


raise SystemError
start = datetime.datetime.now()

#data.set_tpos_all()
#data.set_offset_assoluto_all_new()
#data.set_rototrasl_all()
#data.set_rototraslation_2()

data.set_offset_assoluto_all_new()
data.set_rototraslation_new()
raise SystemError
end = datetime.datetime.now()
elapsed = end - start

print("Tempo impiegato: " + str(elapsed.seconds) + ":" + str(elapsed.microseconds)) 

raise SystemExit

joint_names = data.get_joints_names();
print( "Nomi " + str( joint_names ) )

# VALUES TO CHANGE
joint_name = "LShoulder"
frame_index = 1

print ( "\n##### Calculating " + joint_name + " on frame_index " + str(frame_index))
joint_channels = data.joint_channels(joint_name)  # Array con i nomi dei joint
print("Joint_name " + str(joint_name))
print("Joint channels" + str(joint_channels))
frame_joint_channels = data.frame_joint_channels(frame_index, joint_name, joint_channels);

print( "\nRotazione RZ * RY * RX")
eulero_angles = data.get_eulero_angles( frame_index, joint_channels, frame_joint_channels )

print( calculate_Rzyx( eulero_angles[0], eulero_angles[1], eulero_angles[2] ) )
print( "\nEuler")
print( data.get_rotation( frame_index, joint_channels, frame_joint_channels ) )
print( "\nOffset assoluto")
print( data.get_offset_assoluto(frame_index, joint_name))

print( "\nHIP Traslation")
print( data.get_hip_traslation(frame_index) )

print( "\nRototraslation" )
rotation = data.get_rotation(frame_index, joint_channels, frame_joint_channels)
print( data.get_rototraslation(joint_name, frame_index, rotation) )

#raise SystemExit

print( "\n ##### Calculating rototraslations, all")

start = datetime.datetime.now()

frame_index = 0
k = 0
while frame_index < data.nframes:
    
    j = 0
    while j < len( joint_names ):
        joint_name = joint_names[j]
        joint_channels = data.joint_channels(joint_name)
        frame_joint_channels = data.frame_joint_channels(frame_index, joint_name, joint_channels)
        rotation = data.get_rotation(frame_index, joint_channels, frame_joint_channels)
        #data.get_rotation(frame_index, joint_channels, frame_joint_channels);
        #print("    Joint Name " + joint_name)
        data.get_rototraslation(joint_name, frame_index, rotation)
        #print( data.get_rototraslation(joint_name, frame_index, rotation) )
        j += 1
        k += 1

    print( "Frame_Index " + str(frame_index) + " ok")
    frame_index += 1
    
end = datetime.datetime.now()
elapsed = end - start
print("Tempo impiegato: " + str(elapsed.seconds) + ":" + str(elapsed.microseconds)) 
print( "Iterazioni rototraslazioni: " + str(k) )