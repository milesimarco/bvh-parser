
from bvh import Bvh, BvhNode


print( "START" )

f =  open(r'C:\Users\miles\eclipse-workspace\prova_bvh\src\tests\bvh_carrozzina.bvh')
#f =  open(r'C:\Users\miles\eclipse-workspace\Prova_2\src\tests\test_freebvh.bvh')
mocap = Bvh(f.read())
print( mocap.get_joints_names() )
print( "#########" )

colors = mocap.get_joints_names();   
    
i = 0
while i < len(colors):
    print("\n" + colors[i])
    print(mocap.joint_channels(colors[i]))
    print(mocap.frames_joint_channels(colors[i], mocap.joint_channels(colors[i]) ))
    print(mocap.joint_offset(colors[i]))
    i += 1