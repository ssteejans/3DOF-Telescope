# -*- coding: utf-8 -*-
"""
@file TelescopeDriver.py
    This file contains a driver for a Three Parallel actuator telescope
    
    @author Samuel S. Artho-Bentz
"""
import math

class Telescope(object):
    """This class creates a telescope object controlled by 3 parallel actuators. Each actuator is driven by a stepper motor
    with an L6470 stepper driver."""
    
    def __init__ (self):
        # I think all of the geometry information from GoToAngles can move up to here. Need to figure that out.
        # Define Base Positions
        # All subjective directions (left/right) based on an observer operating the telescope
        self.P0_b = [[0],[0],[0]]    # Origin with respect to the base
        self.P1_b = [[-12],[-2.7],[16]]    # Right Vertical with respect to the base
        self.P2_b = [[12],[-2.7],[16]]    # Left Vertical with respect to the base
        self.P3_b = [[-12],[-2.7],[5]]    # Horizontal with respect to the base
        self.OA_b = self.P0_b;            # Optical Axis base                 

        # Define Home Positions
        self.P1_h = [[-13.75],[6.75],[10.6875]]
        self.P2_h = [[6],[6.4375],[16.6875]]   
        self.P3_h = [[-1.2],[-3.75],[4]]  
        self.OA_h = [[-6],[0],[19.875]]
        
        # Calculate Minimum Lengths
        self.L_p1min = rootsumsquare(self.P1_h, self.P1_b)
        #self.L_p1min = pow(pow((self.P1_h[0][0]-self.P1_b[0][0]),2)+pow((self.P1_h[1][0]-self.P1_b[1][0]),2)+pow((self.P1_h[2][0]-self.P1_b[2][0]),2),.5)
        self.L_p2min = pow(pow((self.P2_h[0][0]-self.P2_b[0][0]),2)+pow((self.P2_h[1][0]-self.P2_b[1][0]),2)+pow((self.P2_h[2][0]-self.P2_b[2][0]),2),.5)
        self.L_p3min = pow(pow((self.P3_h[0][0]-self.P3_b[0][0]),2)+pow((self.P3_h[1][0]-self.P3_b[1][0]),2)+pow((self.P3_h[2][0]-self.P3_b[2][0]),2),.5)
        self.L_p1p2  = pow(pow((self.P1_h[0][0]-self.P2_h[0][0]),2)+pow((self.P1_h[1][0]-self.P2_h[1][0]),2)+pow((self.P1_h[2][0]-self.P2_h[2][0]),2),.5)
        #Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
        self.phi_az = -math.atan2(self.OA_h[0][0], self.OA_h[2][0]);
        self.phi_alt = -math.atan2(self.OA_h[1][0], self.OA_h[2][0]);
        self.phi_rot = math.atan2(self.P1_h[1][0]-self.P2_h[1][0], self.L_p1p2)
        
        
    def GoToAngles(self, O_az, O_alt, O_rot):	
        '''
        Function which calculates the correct lengths of the three linear actuators in order to
        achieve the specified angles (in radians) then sends the scope to that location

        Test values: O_alt = .7854, O_az = .7854, O_rot = 0
        Result in: L_p1 = 21.6256, L_p2 = 25.8043, L_p3 = 9.3052
        '''        
        # Assemble Translation Matrix from Base (b) reference frame to Telescope (s)
        # Rotations Azimuth -> Altitude -> Image Rotations
        sTb = matmul(RotZ(self.phi_rot),matmul(RotZ(O_rot),matmul(RotX(-O_alt),matmul(RotX(self.phi_alt),matmul(RotY(-O_az),RotY(self.phi_az))))))

        # Calculate Rotated Positions
        P1_r = matmul(sTb,self.P1_h)
        P2_r = matmul(sTb,self.P2_h)
        P3_r = matmul(sTb,self.P3_h)
        OA_r = matmul(sTb,self.OA_h)
        # Calculate Length
        L_p1 = pow(pow((P1_r[0][0]-self.P1_b[0][0]),2)+pow((P1_r[1][0]-self.P1_b[1][0]),2)+pow((P1_r[2][0]-self.P1_b[2][0]),2),.5)
        L_p2 = pow(pow((P2_r[0][0]-self.P2_b[0][0]),2)+pow((P2_r[1][0]-self.P2_b[1][0]),2)+pow((P2_r[2][0]-self.P2_b[2][0]),2),.5)
        L_p3 = pow(pow((P3_r[0][0]-self.P3_b[0][0]),2)+pow((P3_r[1][0]-self.P3_b[1][0]),2)+pow((P3_r[2][0]-self.P3_b[2][0]),2),.5)

        # Need to check that the lengths are legitimate
        
        # Need to move actuators
        
        return([L_p1], [L_p2], [L_p3]) 
    
    def StepsReq(self,delLength):
        '''
        Function which calculates the number of steps required to change the length of an actuators
        delLength and steps should be signed.
        '''
        motor_resolution = 400  # steps / revolution of motor
        rod_resolution = .0625  # inches / revolution of rod
        steps = motor_resolution/rod_resolution*delLength
        return steps
    
    def FindHome(self):

class Actuator(object):
    ''' 
    @details Object which represents one linear actuator for a 3 DOF parallel actuator telescope mount
    @param stepperDriver stepper driver object which powers the actuator
    @param stepperResolution Resolution of stepper motor in steps / revolution
    @param rodResolution Resolution of actuator rod in inches / revolution
    '''
    def __init__ (self, stepperDriver, motorNumber, stepperResolution, rodResolution):
        self.stepperDriver = stepperDriver
        self.motorNumber = motorNumber
        self.stepperResolution = stepperResolution
        self.rodResolution = rodResolution
        self.curLength = 0
    def updateCurLength(self):
        self.curLength = self.stepperDriver.get_positions(self.motorNumber)*self.rodResolution/self.stepperResolution
    def goToLength(newLength):
        newSteps = newLength*self.stepperResolution/self.rodResolution
        self.stepperDriver.go_to_position(self.motorNumber, newSteps)
    def findHome(self):
        # not working yet. Trying to determine if stall detection will work 
        steps_per_sec = 100
        self.stepperDriver.run(self.motorNumber, steps_per_sec)
        
        

def rootsumsquare(a, b):
    rss = pow(pow((a[0][0]-b[0][0]),2)+pow((a[1][0]-b[1][0]),2)+pow((a[2][0]-b[2][0]),2),.5)
    return rss
    
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
