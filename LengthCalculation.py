# -*- coding: utf-8 -*-
"""
Created on Fri Mar 31 11:29:54 2017

@author: guest-q0qkzd
"""
import math

def matmul(a, b):
    zip_b = zip(*b)
    zip_b = list(zip_b)
    return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b)) for col_b in zip_b] for row_a in a]

def RotX(rads):
    return [[1,0,0],[0,math.cos(rads),-math.sin(rads)],[0,math.sin(rads),math.cos(rads)]]

def RotY(rads):
    return [[math.cos(rads),0,math.sin(rads)],[0,1,0],[-math.sin(rads),0,math.cos(rads)]] 

def RotZ(rads):
    return [[math.cos(rads),-math.sin(rads),0],[math.sin(rads),math.cos(rads),0],[0,0,1]]

def CalcLengths(O_az, O_alt, O_rot):	
    '''
    Function which calculates the correct lengths of the three linear actuators in order to
    achieve the specified angles (in radians)

    Test values: O_alt = .7854, O_az = .7854, O_rot = 0
    Result in: L_p1 = 21.6256, L_p2 = 25.8043, L_p3 = 9.3052
    '''        
    # Define Base Positions
    # All subjective directions (left/right) based on an observer operating the telescope
    P0_b = [[0],[0],[0]]    # Origin with respect to the base
    P1_b = [[-12],[-2.7],[16]]    # Right Vertical with respect to the base
    P2_b = [[12],[-2.7],[16]]    # Left Vertical with respect to the base
    P3_b = [[-12],[-2.7],[5]]    # Horizontal with respect to the base
    OA_b = P0_b;            # Optical Axis base                 

    # Define Home Positions
    P1_h = [[-13.75],[6.75],[10.6875]]
    P2_h = [[6],[6.4375],[16.6875]]   
    P3_h = [[-1.2],[-3.75],[4]]  
    OA_h = [[-6],[0],[19.875]]

    #Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
    phi_az = -math.atan2(OA_h[0][0], OA_h[2][0]);
    phi_alt = -math.atan2(OA_h[1][0], OA_h[2][0]);

    # Assemble Translation Matrix from Base (b) reference frame to Telescope (s)
    # Rotations Azimuth -> Altitude -> Image Rotations

    sTb = matmul(RotZ(O_rot),matmul(RotX(-O_alt),matmul(RotX(phi_alt),matmul(RotY(-O_az),RotY(phi_az)))))

    # Calculate Rotated Positions
    P1_r = matmul(sTb,P1_h)
    P2_r = matmul(sTb,P2_h)
    P3_r = matmul(sTb,P3_h)
    OA_r = matmul(sTb,OA_h)
    # Calculate Length
    L_p1 = pow(pow((P1_r[0][0]-P1_b[0][0]),2)+pow((P1_r[1][0]-P1_b[1][0]),2)+pow((P1_r[2][0]-P1_b[2][0]),2),.5)
    L_p2 = pow(pow((P2_r[0][0]-P2_b[0][0]),2)+pow((P2_r[1][0]-P2_b[1][0]),2)+pow((P2_r[2][0]-P2_b[2][0]),2),.5)
    L_p3 = pow(pow((P3_r[0][0]-P3_b[0][0]),2)+pow((P3_r[1][0]-P3_b[1][0]),2)+pow((P3_r[2][0]-P3_b[2][0]),2),.5)
    print(L_p1)
    print(L_p2)
    print(L_p3)
    return([L_p1], [L_p2], [L_p3])
    
def StepsReq(delLength):
    '''
    Function which calculates the number of steps required to change the length of an actuators
    delLength and steps should be signed.
    '''
    motor_resolution = 400  # steps / revolution of motor
    rod_resolution = .0625  # inches / revolution of rod
    steps = motor_resolution/rod_resolution*delLength
    return steps