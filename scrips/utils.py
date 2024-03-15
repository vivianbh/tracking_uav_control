#!/usr/bin/env python3
import numpy as np
import math as m
from tf.transformations import quaternion_matrix

def isInvertible(a):
    return a.shape[0] == a.shape[1] and np.linalg.matrix_rank(a) == a.shape[0]

def Rx(theta):
    return np.matrix([[ 1,            0,             0],
                        [ 0, m.cos(theta), -m.sin(theta)],
                        [ 0, m.sin(theta),  m.cos(theta)]])

def Ry(theta):
    return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                        [ 0           , 1,            0],
                        [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
    return np.matrix([[m.cos(theta), -m.sin(theta),  0],
                        [m.sin(theta),  m.cos(theta),  0],
                        [           0,              0, 1]]) 

def Qua2Euler(q):
    roll = m.atan2( 2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y) )
    pitch = 0
    sinp = 2*( q.w*q.y-q.z*q.x )
    if( abs(sinp) >= 1 ):
        pitch = m.copysign( m.pi/2, sinp)
    else:
        pitch = m.asin(sinp)
    yaw = m.atan2( 2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z) )
    euler = [roll, pitch, yaw]

    return euler

def euler2RotMatrix(euler):
    R = np.dot( Rx(euler[0]), Ry(euler[1]), Rz(euler[2]))
    return R

def Qua2RotMatrix(q):
    R = euler2RotMatrix( Qua2Euler(q) )
    return R

# local frame convert to UAV body frame: (ENU -> FLU)
def R_g2l(q):
    #rot_enu2flu = quaternion_matrix([q.w, q.x, q.y, q.z])
    #R = rot_enu2flu[:3, :3]

    R = Qua2RotMatrix(q)
    return R

# UAV gazebo frame to UAV body frame: (FLU -> FRD)
def R_l2b():
    R = np.array([[1,  0,  0],
                  [0, -1,  0],
                  [0,  0, -1]])
    return R

# local frame convert to UAV body frame: (ENU -> FRD)
def R_g2b(q):
    R = np.dot(R_l2b(), R_g2l(q))
    return R

# UAV body frame convert to gimbal pan frame
def R_b2p(ang):
    # [units]: radian
    R = Rz(ang)
    return R

# gimbal tilt frame convert to camera frame
def R_p2c(ang):
    # [units]: radian
    rot_pan2tilt = Ry(ang)
    rot_tilt2cam = np.dot( Rx(m.pi/2), Rz(m.pi/2) )
    R = np.dot(rot_tilt2cam, rot_pan2tilt)
    return R

def R_g2c(tilt, pan, qua):
    return np.dot( R_p2c(tilt), R_b2p(pan), R_g2b(qua) )

def invR_g2c(tilt, pan, qua):
    return np.linalg.inv( R_g2c(tilt, pan, qua) )
