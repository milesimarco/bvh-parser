from bvh import Bvh, BvhNode
import numpy as np

debug = 1

def eulero_angles( instant, joint_channels, frames_joint_channels ): # In entrata [ x, y, z] in gradi
    angles = [
        frames_joint_channels[ instant ][ joint_channels.index("Xrotation" ) ],
        frames_joint_channels[ instant ][ joint_channels.index("Yrotation" ) ],
        frames_joint_channels[ instant ][ joint_channels.index("Zrotation" ) ]
    ]
    print( "Angoli: " + str( angles ) )
    
    x_angle = degree_to_rad( angles[0] );
    y_angle = degree_to_rad( angles[1] );
    z_angle = degree_to_rad( angles[2] );
    
    x_cos = np.cos( x_angle )
    x_sin = np.sin( x_angle )
    y_cos = np.cos( y_angle )
    y_sin = np.sin( y_angle )
    z_cos = np.cos( z_angle )
    z_sin = np.sin( z_angle )
    
    RX = np.matrix(
        [
            [1, 0, 0, 0],
            [0, x_cos, x_sin, 0 ],
            [0, -x_sin, x_cos, 0 ],
            [0, 0, 0, 1]
        ]
        )
    RY = np.matrix(
        [
            [y_cos, 0, -y_sin, 0],
            [0, 1, 0, 0 ],
            [y_sin, 0, y_cos, 0 ],
            [0, 0, 0, 1]
        ]
        )
    RZ = np.matrix(
        [
            [z_cos, z_sin, 0, 0],
            [-z_sin, z_cos, 0, 0 ],
            [0, 0, 1, 0 ],
            [0, 0, 0, 1]
        ]
        )
    
    if debug == 1:
        print( "RX: " + str(RX) )
        print( "RY: " + str(RY) )
        print( "RZ: " + str(RZ) )
   
def degree_to_rad( value ):
    return np.deg2rad( value )
    