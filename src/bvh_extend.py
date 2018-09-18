import bvh
import numpy as np
from functions import *
#from estensione import *
from transforms3d.euler import euler2mat
from transforms3d.affines import compose
from array import array

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
    
    def get_rotation(self, instant, joint_channels, frame_joint_channels):  #usa il metodo euler2mat della libreria transform3d
        
        angles = [
            frame_joint_channels[ joint_channels.index("Xrotation") ],
            frame_joint_channels[ joint_channels.index("Yrotation") ],
            frame_joint_channels[ joint_channels.index("Zrotation") ]
        ]
        
        x_angle = deg2rad(angles[0]);
        y_angle = deg2rad(angles[1]);
        z_angle = deg2rad(angles[2]);
        
        #return euler_matrix(x_angle, y_angle, z_angle, 'sxyz');
        a = compose( np.zeros(3), euler2mat(x_angle,y_angle,z_angle,'sxyz'), [1.0, 1.0, 1.0] )
        return a  #con utilizzo di libreria 3d transform #tempisticamente identico al mio
                                                            # estensione


    def get_offset_assoluto(self, instant, joint_name):
        
        result = np.matrix(self.joint_offset(joint_name))
        
        while self.joint_parent(joint_name) != None:
            joint_parent_name = self.get_joints_names()[ self.joint_parent_index(joint_name) ]
            #print( joint_parent_name + " is parent of " + joint_name)
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
        
    def get_rototraslation(self, joint_name, instant, rotation):
        
        abs_pos_now = self.get_offset_assoluto(instant, joint_name)
        abs_pos_before = self.get_offset_assoluto(instant - 1, joint_name)
        abs_diff = np.subtract(abs_pos_now, abs_pos_before)
        #abs_diff = abs_diff.reshape((-1, 1))
        c = np.array([[1]])
        abs_diff = np.concatenate((abs_diff, c), axis=1)
        #print( abs_diff )
        #print( str(rotation))
        rotation[:, 3] = abs_diff
        return rotation
        
    
    def set_offset_assoluto_all_new(self):
        joint_names = self.get_joints_names();
        
        actual_node = self.get_joint( "Hip" ) #start
        strat_hip_translation = self.get_hip_traslation(0)
        print( "HIP TRASLATION " + str( strat_hip_translation ) )
        
        figli_hip = actual_node.get_childs()
        nodi_da_fare = []
        print ( "a " + str(figli_hip))
        for child_name in figli_hip:
            #print( child_name )
            for child_child_name in self.get_joint(child_name).get_childs():
                print( "child of " + child_name + " @ " + child_child_name )
                nodi_da_fare.append( child_child_name )

            offset = np.add( strat_hip_translation,  self.joint_offset( child_name ) )
            self.get_joint( child_name ).offsets.append(offset)
            
        #self.print_offsets();
                
        while len( nodi_da_fare ) != 0:
            print( "ToDo: " + str( nodi_da_fare ) )
            for child_name in nodi_da_fare:
                print( child_name)
                
                joint = self.get_joint( child_name )
                parent = self.joint_parent( child_name )
                #print( child_name + str( self.joint_offset( child_name ) ) )
                print( "@ " + str( parent.offsets ))
                offset = np.add( parent.offsets[0],  self.joint_offset( child_name ) )
                joint.offsets.append(offset)
                
                # Scan nodi figli e aggiunta alla lista
                c = joint.get_childs()
                if ( len( c ) ):
                    nodi_da_fare = nodi_da_fare + c
                    
                nodi_da_fare.remove( child_name )
                print( "# " + str(nodi_da_fare))
                #print( str(self.get_joint( child_name).offsets) )
            
        self.print_offsets(0)
        
    def print_offsets(self, frame_index = 0):
        joint_names = self.get_joints_names();
        
        j = 0
        while j < len(joint_names) - 1:
            joint = self.get_joint( joint_names[j] )
            print( joint_names[j] + " // " + str( joint.offsets ) )
            j += 1
        
        print ( "Total: " + str(j))
        
    def set_offset_assoluto_all(self):
        joint_names = self.get_joints_names();
        frame_index = 0
        while frame_index < self.nframes - 1:
            j = 0
            print_status( frame_index, self.nframes)
            while j < len(joint_names) - 1:
                joint_name = joint_names[j]
                joint_parent_name = joint_names[ self.joint_parent_index(joint_name) ]
                
                try:
                    parent_offset = self.get_joint(joint_parent_name).offsets[frame_index]
                    self.get_joint(joint_parent_name).offsets.append( self.get_offset_assoluto(frame_index, joint_name) )
                except (IndexError, ValueError):
                    self.get_joint(joint_parent_name).offsets.append( self.get_offset_assoluto(frame_index, joint_name) )
    
                # print( joint_names[j] + " frame " + str(i))
                #self.get_joint(joint_names[j]).offsets.append( [0, 0, 0] )
                j += 1
                #print( str(self.get_joint(joint_names[j]).offsets) )
            frame_index += 1

        
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
    
    
