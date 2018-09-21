import bvh
import numpy as np
from functions import *
#from estensione import *
from transforms3d.euler import euler2mat
from transforms3d.affines import compose
from array import array
from estensione import *

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
            trasl[:, 3] = np.append( self.get_hip_traslation( frame_index ), [1] )
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
                
                #print( joint_name + " @ " + str(frame_index) )
                #print( str(M) )
                joint.rototranslation.append(M)
                
                frame_index+=1
            j+=1
                
            
        
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
    
    def get_rotation(self, frame_index, joint_channels, frame_joint_channels):  #usa il metodo euler2mat della libreria transform3d
        
        angles = [
            frame_joint_channels[ joint_channels.index("Xrotation") ],
            frame_joint_channels[ joint_channels.index("Yrotation") ],
            frame_joint_channels[ joint_channels.index("Zrotation") ]
        ]
        
        x_angle = deg2rad(angles[0]);
        y_angle = deg2rad(angles[1]);
        z_angle = deg2rad(angles[2]);
        
        a = compose( np.zeros(3), euler2mat(x_angle,y_angle,z_angle,'sxyz'), [1.0, 1.0, 1.0] )
        return a

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
        
    
    def set_rototraslation_new(self):
        joint_names = self.get_joints_names()
        
        frame_index = 0
        while frame_index < self.nframes:
            
            #print( "rototrasl frame_index " + str(frame_index) )
            j = 0
            while j < len( joint_names ):
                joint_name = joint_names[j]                
                joint_channels = self.joint_channels(joint_name)
                frame_joint_channels = self.frame_joint_channels(frame_index, joint_name, joint_channels)
                rotation = self.get_rotation( frame_index, joint_channels, frame_joint_channels)
                
                #print( "DEBUG " + joint_name + " index " + str(frame_index))
                abs_pos_now = self.get_joint(joint_name).offsets[frame_index]
                abs_pos_before = self.get_joint( joint_name ).offsets[frame_index-1]
                abs_diff = np.subtract(abs_pos_now, abs_pos_before)
                abs_diff = np.append( abs_diff, [1] )
                rotation[:, 3] = abs_diff
                print( "Rototraslazione " + joint_name + " #" + str(frame_index) + " @ " + str(rotation))
                j+=1
                
            frame_index+=1
            
    def set_rototraslation_2(self):
        joint_names = self.get_joints_names()
        
        frame_index = 0
        while frame_index < self.nframes:
            
            #print( "rototrasl frame_index " + str(frame_index) )
            j = 0
            while j < len( joint_names ):
                joint_name = joint_names[j]                
                joint_channels = self.joint_channels(joint_name)
                frame_joint_channels = self.frame_joint_channels(frame_index, joint_name, joint_channels)
                rotation = self.get_rotation( frame_index, joint_channels, frame_joint_channels)
                
                #print( "DEBUG " + joint_name + " index " + str(frame_index))
                abs_pos_now = self.get_joint(joint_name).offsets[frame_index]
                abs_pos_before = self.get_joint( joint_name ).offsets[frame_index-1]
                abs_diff = np.subtract(abs_pos_now, abs_pos_before)
                abs_diff = np.append( abs_diff, [1] )
                rotation[:, 3] = abs_diff
                rotation = np.matmul( rotation, )
                print( "Rototraslazione " + joint_name + " #" + str(frame_index) + " @ " + str(rotation))
                j+=1
                
            frame_index+=1
            
    def set_offset_assoluto_all_new(self):
        joint_names = self.get_joints_names();
        
        actual_node = self.get_joint( "Hip" ) #start
        strat_hip_translation = self.get_hip_traslation(0)
        #print( "HIP TRASLATION " + str( strat_hip_translation ) )
        
        figli_hip = actual_node.get_childs()
        nodi_da_fare = []
        #print ( "a " + str(figli_hip))
        for child_name in figli_hip:
            #print( child_name )
            for child_child_name in self.get_joint(child_name).get_childs():
                #print( "child of " + child_name + " @ " + child_child_name )
                nodi_da_fare.append( child_child_name )

            offset = np.add( strat_hip_translation,  self.joint_offset( child_name ) )
            self.get_joint( child_name ).offsets.append(offset)
            
        #self.print_offsets();
                
        while len( nodi_da_fare ) != 0:
            #print( "ToDo: " + str( nodi_da_fare ) )
            for child_name in nodi_da_fare:
                #print( child_name)
                
                joint = self.get_joint( child_name )
                parent = self.joint_parent( child_name )
                #print( child_name + str( self.joint_offset( child_name ) ) )
                #print( "@ " + str( parent.offsets ))
                offset = np.add( parent.offsets[0],  self.joint_offset( child_name ) )
                joint.offsets.append(offset)
                
                # Scan nodi figli e aggiunta alla lista
                c = joint.get_childs()
                if ( len( c ) ):
                    nodi_da_fare = nodi_da_fare + c
                    
                nodi_da_fare.remove( child_name )
                #print( "# " + str(nodi_da_fare))
                #print( str(self.get_joint( child_name).offsets) )
        
        self.get_joint( joint_names[0] ).offsets.append( self.get_hip_traslation( 0 ) )
        # Istanti > 0
        frame_index = 1
        while frame_index < self.nframes:
            self.get_joint( joint_names[0] ).offsets.append( self.get_hip_traslation( frame_index ) )
            hip_translation = np.subtract( self.get_hip_traslation( frame_index ), self.get_hip_traslation( frame_index - 1 ) )
            
            j = 1
            while j < len(joint_names):
                joint_name = joint_names[j]
                joint = self.get_joint( joint_name )
                #print( "ID " + str( frame_index ) + " " + joint_name + " @ " + str(joint.offsets))
                offset = joint.offsets[ frame_index-1 ] +  hip_translation
                joint.offsets.append(offset)
                
                j+=1
            
            frame_index += 1
            
        self.print_offsets()
        
    def print_offsets(self ):
        joint_names = self.get_joints_names();
        
        print( "###### OFFSETS ######")
        j = 0
        while j < len(joint_names) - 1:
            joint = self.get_joint( joint_names[j] )
            print( joint_names[j] + "        " + str( joint.offsets ) )
            j += 1
        
        print ( "Total: " + str(j+1) + "\n#########")
        
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
                
                
                
                
    def set_tpos_all(self):
        
        joint_names = self.get_joints_names();
        
        j = 0
        while j < len(joint_names) - 1:
            matrix = np.identity(4)
            
            #abs_pos_now = self.get_joint(joint_name).offsets[frame_index]
            #abs_pos_before = self.get_joint( joint_name ).offsets[frame_index-1]
            #abs_diff = np.subtract(abs_pos_now, abs_pos_before)
            x = np.append( self.joint_offset( joint_names[j] ), [1] )
            matrix[:, 3] = np.append( self.joint_offset( joint_names[j] ), [1] )
            self.get_joint( joint_names[j] ).TPos = matrix
            #print( str(matrix))
            #raise SystemError
            j+=1
            
    def set_rototrasl_all(self):
        
        joint_names = self.get_joints_names();
         
        frame_index = 0
        while frame_index < self.nframes - 1:
            j = 0
            while j < len(joint_names) - 1:
        
                Xr = self.frame_joint_channel( frame_index, joint_names[j], "Xrotation")
                Yr = self.frame_joint_channel( frame_index, joint_names[j], "Yrotation")
                Zr = self.frame_joint_channel( frame_index, joint_names[j], "Zrotation")
        
                R = self.get_rotation_matrix(Xr, Yr, Zr)
                
                print( str( R ) )
                print( "RESULT HIP: ")
                print( np.matmul( self.get_joint( "Hip").TPos, R ) )
    
    def get_magicEnglish(self, Xr, Yr, Zr):  # calcolo una matrice sola
                                                                        # invece che 3 per poi moltiplicarle
                                                                        # magari sveltisce, quando sistemiamo gli offset
        
        x_angle = deg2rad( Xr );
        y_angle = deg2rad( Yr );
        z_angle = deg2rad( Zr);
        
        #return euler_matrix(x_angle, y_angle, z_angle, 'sxyz');
        return euler2mat(x_angle,y_angle,z_angle,'sxyz');  #con utilizzo di libreria 3d transform #tempisticamente identico al mio
                                                            # estensione
        
    def get_magic(self, Xr, Yr, Zr):  # calcolo una matrice sola
                                                                        # invece che 3 per poi moltiplicarle
                                                                        # magari sveltisce, quando sistemiamo gli offset
        
        x_angle = deg2rad( Xr );
        y_angle = deg2rad( Yr );
        z_angle = deg2rad( Zr );
    
        x_cos = math.cos(x_angle)
        x_sin = math.sin(x_angle)
        y_cos = math.cos(y_angle)
        y_sin = math.sin(y_angle)
        z_cos = math.cos(z_angle)
        z_sin = math.sin(z_angle)
        
        R = np.matrix(
            [
                [z_cos * y_cos, z_cos * y_sin * x_sin - z_sin * x_cos, z_cos * y_sin * x_cos + z_sin * x_sin, 0],
                [z_sin * y_cos, z_sin * y_sin * x_sin + z_cos * x_cos, z_sin * y_sin * x_cos - z_cos * x_sin, 0],
                [-y_sin, y_cos * x_sin, y_cos * x_cos, 0],
                [0, 0, 0, 1]
            
            ]
            )
        
        return R
    
    def get_rotation_matrix(self, Xr, Yr, Zr):
        
        x_angle = deg2rad( Xr )
        y_angle = deg2rad( Yr )
        z_angle = deg2rad( Zr )
        
        return compose( np.zeros(3), euler2mat(x_angle,y_angle,z_angle,'rxyz'), [1, 1, 1] )

    def get_rotation_matrix2(self, Xr, Yr, Zr):
        
        x_angle = deg2rad( Xr )
        y_angle = deg2rad( Yr )
        z_angle = deg2rad( Zr )
        
        x_cos = math.cos(x_angle)
        x_sin = math.sin(x_angle)
        y_cos = math.cos(y_angle)
        y_sin = math.sin(y_angle)
        z_cos = math.cos(z_angle)
        z_sin = math.sin(z_angle)
        
        RX = np.asarray(
            [
                [1, 0, 0, 0],
                [0, x_cos, -x_sin, 0 ],
                [0, x_sin, x_cos, 0 ],
                [0, 0, 0, 1]
            ]
            )
        RY = np.asarray(
            [
                [y_cos, 0, y_sin, 0],
                [0, 1, 0, 0 ],
                [-y_sin, 0, y_cos, 0 ],
                [0, 0, 0, 1]
            ]
            )
        RZ = np.asarray(
            [
                [z_cos, -z_sin, 0, 0],
                [z_sin, z_cos, 0, 0 ],
                [0, 0, 1, 0 ],
                [0, 0, 0, 1]
            ]
            )

        return np.dot( np.dot( RZ, RY), RX)


class BvhNodeExtend(bvh.BvhNode):

    def test(self):
        return 0
    
    
