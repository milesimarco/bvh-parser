import bvh
import numpy as np

class BvhCalculator(bvh.Bvh):
    def offset_assoluto(self, joint_name, istant = 0 ):
        
        result = np.matrix( self.joint_offset( joint_name ) )
        
        while self.joint_parent( joint_name ) != None:
            print( "scanning " + str(joint_name) + ", adding " + str( self.joint_offset( joint_name ) ) )
            #print( "Parent " + str(self.joint_parent( joint_name) ) )
            
            joint_parent_name = self.get_joints_names()[self.get_joint_index(joint_name) -1]
            result += np.matrix( self.joint_offset( joint_parent_name ) )
            joint_name = joint_parent_name
            #joint_name = self.joint_parent( joint_name )
            
        return result + np.matrix( self.get_hip_traslation( istant ) )
    
    def get_hip_traslation(self, instant = 0 ):
        
        hip_name = self.get_joints_names()[0];
        joint_channels = self.joint_channels( hip_name )
        frames_joint_channels = self.frames_joint_channels( hip_name, joint_channels );
        
        return [
            frames_joint_channels[ instant ][ joint_channels.index("Xposition" ) ],
            frames_joint_channels[ instant ][ joint_channels.index("Yposition" ) ],
            frames_joint_channels[ instant ][ joint_channels.index("Zposition" ) ]
            ]