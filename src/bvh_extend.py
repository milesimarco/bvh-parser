import bvh
import numpy as np
from functions import *
#from estensione import *
from transforms3d.euler import euler2mat

class BvhCalculator(bvh.Bvh):

    def get_eulero_angles(self, instant, joint_channels, frame_joint_channels): #deprecated
        
        angles = [
            frame_joint_channels[ joint_channels.index("Xrotation") ],
            frame_joint_channels[ joint_channels.index("Yrotation") ],
            frame_joint_channels[ joint_channels.index("Zrotation") ]
        ]
        
        x_angle = deg2rad(angles[0]);
        y_angle = deg2rad(angles[1]);
        z_angle = deg2rad(angles[2]);
        
        x_cos = math.cos(x_angle)
        x_sin = math.sin(x_angle)
        y_cos = math.cos(y_angle)
        y_sin = math.sin(y_angle)
        z_cos = math.cos(z_angle)
        z_sin = math.sin(z_angle)
        
        RX = np.asarray(
            [
                [1, 0, 0, 0],
                [0, x_cos, x_sin, 0 ],
                [0, -x_sin, x_cos, 0 ],
                [0, 0, 0, 1]
            ]
            )
        RY = np.asarray(
            [
                [y_cos, 0, -y_sin, 0],
                [0, 1, 0, 0 ],
                [y_sin, 0, y_cos, 0 ],
                [0, 0, 0, 1]
            ]
            )
        RZ = np.asarray(
            [
                [z_cos, z_sin, 0, 0],
                [-z_sin, z_cos, 0, 0 ],
                [0, 0, 1, 0 ],
                [0, 0, 0, 1]
            ]
            )

        return [ RX, RY, RZ]
    
    def get_matrix(self, instant, joint_channels, frame_joint_channels):  # calcolo una matrice sola
                                                                        # invece che 3 per poi moltiplicarle
                                                                        # magari sveltisce, quando sistemiamo gli offset
        
        angles = [
            frame_joint_channels[ joint_channels.index("Xrotation") ],
            frame_joint_channels[ joint_channels.index("Yrotation") ],
            frame_joint_channels[ joint_channels.index("Zrotation") ]
        ]
        
        x_angle = deg2rad(angles[0]);
        y_angle = deg2rad(angles[1]);
        z_angle = deg2rad(angles[2]);
        
        x_cos = np.cos(x_angle)
        x_sin = np.sin(x_angle)
        y_cos = np.cos(y_angle)
        y_sin = np.sin(y_angle)
        z_cos = np.cos(z_angle)
        z_sin = np.sin(z_angle)
        
        R = np.matrix(
            [
                [z_cos * y_cos, z_cos * y_sin * x_sin - z_sin * x_cos, z_cos * y_sin * x_cos + z_sin * x_sin, 0],
                [z_sin * y_cos, z_sin * y_sin * x_sin + z_cos * x_cos, z_sin * y_sin * x_cos - z_cos * x_sin, 0],
                [-y_sin, y_cos * x_sin, y_cos * x_cos, 0],
                [0, 0, 0, 1]
            
            ]
            )
        
        return R
    
    def get_matrix3d(self, instant, joint_channels, frame_joint_channels):  #usa il metodo euler2mat della libreria transform3d
        
        angles = [
            frame_joint_channels[ joint_channels.index("Xrotation") ],
            frame_joint_channels[ joint_channels.index("Yrotation") ],
            frame_joint_channels[ joint_channels.index("Zrotation") ]
        ]
        
        x_angle = deg2rad(angles[0]);
        y_angle = deg2rad(angles[1]);
        z_angle = deg2rad(angles[2]);
        
        #return euler_matrix(x_angle, y_angle, z_angle, 'sxyz');
        return euler2mat(x_angle,y_angle,z_angle,'sxyz');  #con utilizzo di libreria 3d transform #tempisticamente identico al mio
                                                            # estensione


    def get_offset_assoluto(self, instant, joint_name):
        
        result = np.matrix(self.joint_offset(joint_name))
        
        while self.joint_parent(joint_name) != None:
            joint_parent_name = self.get_joints_names()[self.get_joint_index(joint_name) - 1]
            result += np.matrix(self.joint_offset(joint_parent_name))
            joint_name = joint_parent_name
            
        return result + np.matrix(self.get_hip_traslation(instant))

    
    def get_hip_traslation(self, instant=0):
        
        hip_name = self.get_joints_names()[0];
        joint_channels = self.joint_channels(hip_name)
        frames_joint_channels = self.frames_joint_channels(hip_name, joint_channels);
        
        return [
            frames_joint_channels[ instant ][ joint_channels.index("Xposition") ],
            frames_joint_channels[ instant ][ joint_channels.index("Yposition") ],
            frames_joint_channels[ instant ][ joint_channels.index("Zposition") ]
        ]

        
    def get_rotation(self, instant, joint_channels, frames_joint_channels):
        k = self.get_matrix(instant, joint_channels, frames_joint_channels)
        return k;
        #return calculate_Rzyx(k[0], k[1], k[2])  # Alias: rotation

        
    def get_rototraslation(self, joint_name, instant, rotation):
        abs_pos_now = self.get_offset_assoluto(instant, joint_name)
        abs_pos_before = self.get_offset_assoluto(instant - 1, joint_name)
        abs_diff = np.subtract(abs_pos_now, abs_pos_before)
        # abs_diff = np.append( abs_diff, np.matrix( [[1]] ))
        abs_diff = abs_diff.reshape((-1, 1))
        c = np.array([[1]])
        abs_diff = np.concatenate((abs_diff, c), axis=0)
        rotation[:, 3] = abs_diff
        return rotation
        
    
    def set_offset_assoluto_all(self):
        joint_names = self.get_joints_names();
        i = 0
        while i < self.nframes - 1:
            j = 0
            print_status(i, self.nframes)
            while j < len(joint_names) - 1:
                # print( joint_names[j] + " frame " + str(i))
                self.get_joint(joint_names[j]).var_offset_assoluto.append(self.get_offset_assoluto(joint_names[j], i))
                j += 1
            i += 1

        
    def calculate_joints_rotations_all_frames(self):
        joint_names = self.get_joints_names();
        
        i = 0
        while i < self.nframes:
            
            print("Scanning " + str(i))

            j = 0
            while j < len(joint_names):
                node = BvhNode()
                node = self.get_joint(joint_names[j])
                node.get_hip_traslation(i)

                
    

class BvhNodeExtend(bvh.BvhNode):

    def test(self):
        return 0
    
    
