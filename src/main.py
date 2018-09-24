from bvh import *
from bvh_extend import *
from bvh_functions import *
import datetime
import os.path

f=open(os.path.relpath('2017-12-22_16-22-35.bvh', start=os.curdir))
data = BvhCalculator(f.read())
print( "Frame: " + str(data.nframes) + " - Joints: " + str( len(data.get_joints_names() )) )

start = datetime.datetime.now()
data.calculate_rototranslations() # Calcola tutte le rototraslazioni
end = datetime.datetime.now()
time_print(start, end, "Rototraslazioni, tutte")

#joint_names = data.get_joints_names()
#j = 0
#while j < len( joint_names ):
#    joint_name = joint_names[j]
#    x = data.get_joint(joint_name).rototranslation
#    print( joint_name + " " + str( len(x) ) + " " )
#    j += 1
      
if 0 == 1: # Test
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


print( "###########" )

start = datetime.datetime.now()
data.calculate_rototranslations_relative() # Calcola tutte le rototraslazioni
end = datetime.datetime.now()
time_print(start, end, "Rototraslazioni Relative, tutte")

if 1 == 1: # Test
    print( data.get_joint("Hip").get_rototranslation_relative(1) )
    print( data.get_joint("Hip").get_position_relative(1) )
    print( data.get_joint("Hip").get_position_relative(2) )
    print( data.get_joint("Hip").get_position_relative(3) )
    print( data.get_joint("LowerSpine").get_position_relative(3) )
    print( data.get_joint("MiddleSpine").get_position_relative(4) )
    
    print( "last frame tests")
    print( data.get_joint("Hip").get_position_relative( data.nframes -1 ) )
    print( data.get_joint("LowerSpine").get_position_relative( data.nframes -1) )
    print( data.get_joint("MiddleSpine").get_position_relative( data.nframes -1) )
    print( data.get_joint("Chest").get_position_relative( data.nframes -1) )
    print( data.get_joint("LToe").get_position_relative( data.nframes -1) )

raise SystemError

MHip = data.get_joint( "Hip").rototranslation[0]
MLowerSpine = data.get_joint( "LowerSpine").rototranslation[0]

T = MLowerSpine - MHip
print( str(T) )

MLowerSpine = data.get_joint( "LowerSpine").rototranslation[0]
MMiddleSpine = data.get_joint( "MiddleSpine").rototranslation[0]

T = MMiddleSpine - MLowerSpine
print( str(T) )