from bvh import *
from bvh_extend import *
import datetime
import os.path

#apertura del file con path relativo
#f=open(os.path.relpath('carrozzina.bvh', start=os.curdir))
f=open(os.path.relpath('2017-11-09_15-23-34.bvh', start=os.curdir))
data = BvhCalculator(f.read()) #lettura del file e salvataggio in data
print( "Frame: " + str(data.nframes) + " - Joints: " + str( len(data.get_joints_names() )) )


start = datetime.datetime.now()
data.calculate_tpos() # Calcola tutte le rototraslazioni
end = datetime.datetime.now()
time_print(start, end, "Tpos, tutti frame")

if 0 == 1: # Test
    print( data.get_joint("Hip").get_tpos_vector() )
    print( data.get_joint("LowerSpine").get_tpos_vector() )
    print( data.get_joint("MiddleSpine").get_tpos_vector() )
    print( data.get_joint("LClavicle").get_tpos_vector() )
    print( data.get_joint("Chest").get_tpos_vector() )
    print( data.get_joint("LToe").get_tpos_vector() )
    
start = datetime.datetime.now()
data.calculate_rototranslations() # Calcola tutte le rototraslazioni
end = datetime.datetime.now()
time_print(start, end, "Rototraslazioni, tutti frame")
      
if 0 == 1: # Test
    print( data.get_joint("Hip").get_rototranslation_matrix(1) )
    print( data.get_joint("Hip").get_position_vector(1) )
    print( data.get_joint("Hip").get_position_vector(2) )
    print( data.get_joint("Hip").get_position_vector(3) )
    print( data.get_joint("LowerSpine").get_position_vector(3) )
    print( data.get_joint("MiddleSpine").get_position_vector(4) )
    
    print( "last frame tests")
    print( data.get_joint("Hip").get_position_vector( data.nframes -1 ) )
    print( data.get_joint("LowerSpine").get_position_vector( data.nframes -1) )
    print( data.get_joint("MiddleSpine").get_position_vector( data.nframes -1) )
    print( data.get_joint("Chest").get_position_vector( data.nframes -1) )
    print( data.get_joint("LToe").get_position_vector( data.nframes -1) )

start = datetime.datetime.now()
data.calculate_rototranslations_relative() # Calcola tutte le rototraslazioni
end = datetime.datetime.now()
time_print(start, end, "Rototraslazioni Relative, tutte")

if 1 == 1: # Test
    print( data.get_joint("Hip").get_rototranslation_relative_matrix(1) )
    print( data.get_joint("Hip").get_position_relative_vector(1) )
    print( data.get_joint("Hip").get_position_relative_vector(2) )
    print( data.get_joint("Hip").get_position_relative_vector(3) )
    print( data.get_joint("LowerSpine").get_position_relative_vector(3) )
    print( data.get_joint("MiddleSpine").get_position_relative_vector(4) )
    
    print( "last frame tests")
    print( data.get_joint("Hip").get_position_relative_vector( data.nframes -1 ) )
    print( data.get_joint("LowerSpine").get_position_relative_vector( data.nframes -1) )
    print( data.get_joint("MiddleSpine").get_position_relative_vector( data.nframes -1) )
    print( data.get_joint("Chest").get_position_relative_vector( data.nframes -1) )
    print( data.get_joint("LToe").get_position_relative_vector( data.nframes -1) )
    
print( "valori cercati nel main:")

print(  data.get_joint( "RHand").rototranslation[0] )
print(  data.get_joint( "RHand").get_euler_vector_all_frames_xyz()[2] )

raise SystemError