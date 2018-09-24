import bvh
from bvh_functions import *
import datetime
#from transforms3d.euler import euler2mat
#from transforms3d.affines import compose

class BvhCalculator(bvh.Bvh):

    # Marco Milesi 20180921
    def calculate_rototranslations(self):
        
        np.set_printoptions(suppress=True)
        
        joint_names = self.get_joints_names()
        identity = np.identity(4)
        
        # HIP
        frame_index = 0
        joint_name = "Hip"
        joint = self.get_joint(joint_name)
        while frame_index < self.nframes:
            
            Xr = self.frame_joint_channel( frame_index, joint_name, "Xrotation")
            Yr = self.frame_joint_channel( frame_index, joint_name, "Yrotation")
            Zr = self.frame_joint_channel( frame_index, joint_name, "Zrotation")
            rotation = euler_matrix( deg2rad(Xr), deg2rad(Yr), deg2rad(Zr), "rxyz" )
            
            trasl = identity
            
            #Hip Translation
            Xt = self.frame_joint_channel( frame_index, joint_name, "Xposition")
            Yt = self.frame_joint_channel( frame_index, joint_name, "Yposition")
            Zt = self.frame_joint_channel( frame_index, joint_name, "Zposition")
            trasl[:, 3] = np.append( [Xt, Yt, Zt], [1] )
            M = np.dot( trasl, rotation )
            joint.rototranslation.append(M)
           
            frame_index+=1
            
        
        j = 1
        while j < len( joint_names ):
            
            joint_name = joint_names[j]
            trasl = identity
            trasl[:, 3] = np.append( self.joint_offset( joint_name), [1] )
            
            joint = self.get_joint(joint_name)
            
            frame_index = 0
            while frame_index < self.nframes:
                
                Xr = self.frame_joint_channel( frame_index, joint_name, "Xrotation")
                Yr = self.frame_joint_channel( frame_index, joint_name, "Yrotation")
                Zr = self.frame_joint_channel( frame_index, joint_name, "Zrotation")
                
                rotation = euler_matrix( deg2rad(Xr), deg2rad(Yr), deg2rad(Zr), "rxyz" )
                
                M = np.dot( self.joint_parent( joint_name ).rototranslation[frame_index], np.dot(trasl, rotation ) )
                
                joint.rototranslation.append(M)
                
                frame_index+=1
            j+=1
    
    # Marco Milesi 20180924
    def calculate_rototranslations_relative(self):
        
        np.set_printoptions(suppress=True)
        
        joint_names = self.get_joints_names()
                
                
        # Assunzione: Hip sempre zero
        frame_index = 0
        main_joint = self.get_joint( joint_names[0] )
        while frame_index < self.nframes:
            main_joint.rototranslation_relative.append( np.zeros( ( 4, 4), dtype=int ) )
            frame_index+=1
            
        start = datetime.datetime.now()
        
        frame_index = 0
        j = 1
        while j < len( joint_names ):
            
            joint_name = joint_names[j]            
            joint = self.get_joint(joint_name)
            
            current_rotos = joint.rototranslation
            parent_rotos = self.joint_parent( joint_name ).rototranslation
            
            frame_index = 0
            while frame_index < self.nframes:
                joint.rototranslation_relative.append( current_rotos[frame_index] - parent_rotos[frame_index])
                
                frame_index+=1
            j+=1            
            
        end = datetime.datetime.now()
        time_print(start, end, "tutti i giunti tranne hip, su tutti i frame")
        
    def get_hip_traslation(self, instant=0):
        
        hip_name = self.get_joints_names()[0];
        joint_channels = self.joint_channels(hip_name)
        frames_joint_channels = self.frames_joint_channels(hip_name, joint_channels);
        
        return [
            frames_joint_channels[ instant ][ joint_channels.index("Xposition") ],
            frames_joint_channels[ instant ][ joint_channels.index("Yposition") ],
            frames_joint_channels[ instant ][ joint_channels.index("Zposition") ]
        ]
    
    def calcuate_tpos(self):
        
        joint_names = self.get_joints_names();
        
        j = 0
        while j < len(joint_names):
            matrix = np.identity(4)
            x = np.append( self.joint_offset( joint_names[j] ), [1] )
            matrix[:, 3] = np.append( self.joint_offset( joint_names[j] ), [1] )
            self.get_joint( joint_names[j] ).TPos = matrix
            j+=1

class BvhNodeExtend(bvh.BvhNode):

    def test(self):
        return 0
    
    
