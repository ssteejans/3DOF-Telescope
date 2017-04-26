# -*- coding: utf-8 -*-
"""
@file TelescopeDriver.py
    This file contains a driver for a Three Parallel actuator telescope
    
    @author Samuel S. Artho-Bentz
"""
import math
import time

class Telescope(object):
    """This class creates a telescope object controlled by 3 parallel actuators. Each actuator is driven by a stepper motor
    with an L6470 stepper driver."""
    
    def __init__ (self, actuatorOne, actuatorTwo, actuatorThree):
        self.actuatorOne = actuatorOne
        self.actuatorTwo = actuatorTwo
        self.actuatorThree = actuatorThree

        # Define Base Positions
        # All subjective directions (left/right) based on an observer operating the telescope
        self.P0_b = [[0],[0],[0]]    # Origin with respect to the base
        self.P1_b = [[-12],[-2.7],[16]]    # Right Vertical with respect to the base
        self.P2_b = [[12],[-2.7],[16]]    # Left Vertical with respect to the base
        self.P3_b = [[-12],[-2.7],[5]]    # Horizontal with respect to the base
        self.OA_b = self.P0_b;            # Optical Axis base                 

        # Define Home Positions
        self.P1_h = [[-13.375],[8.625],[10.935]]
        self.P2_h = [[7.25],[9.0],[15.5313]]   
        self.P3_h = [[-1.25],[-2.125],[5.6875]]  
        self.OA_h = [[-4],[4],[15.625]]
        
        # Calculate Minimum Lengths
        
        #self.L_p1min = pow(pow((self.P1_h[0][0]-self.P1_b[0][0]),2)+pow((self.P1_h[1][0]-self.P1_b[1][0]),2)+pow((self.P1_h[2][0]-self.P1_b[2][0]),2),.5)
        #self.L_p2min = pow(pow((self.P2_h[0][0]-self.P2_b[0][0]),2)+pow((self.P2_h[1][0]-self.P2_b[1][0]),2)+pow((self.P2_h[2][0]-self.P2_b[2][0]),2),.5)
        #self.L_p3min = pow(pow((self.P3_h[0][0]-self.P3_b[0][0]),2)+pow((self.P3_h[1][0]-self.P3_b[1][0]),2)+pow((self.P3_h[2][0]-self.P3_b[2][0]),2),.5)
        #self.L_p1p2  = pow(pow((self.P1_h[0][0]-self.P2_h[0][0]),2)+pow((self.P1_h[1][0]-self.P2_h[1][0]),2)+pow((self.P1_h[2][0]-self.P2_h[2][0]),2),.5)
        self.L_p1min = rootsumsquare(self.P1_h, self.P1_b)
        self.L_p2min = rootsumsquare(self.P2_h, self.P2_b)
        self.L_p3min = rootsumsquare(self.P3_h, self.P3_b)
        self.L_p1p2  = rootsumsquare(self.P1_h, self.P2_h)
		
		#Calculate Correction Angles \phi to Rotate from home position to alt = 0, az = 0 
        self.phi_az = -math.atan2(self.OA_h[0][0], self.OA_h[2][0]);
        self.phi_alt = math.atan2(self.OA_h[1][0], self.OA_h[2][0]);
        self.phi_rot = math.atan2(self.P1_h[1][0]-self.P2_h[1][0], self.L_p1p2)
        
        
    def GoToAngles(self, O_az, O_alt, O_rot, move=False):	
        '''
        Function which calculates the correct lengths of the three linear actuators in order to
        achieve the specified angles (in degrees) then sends the scope to that location

       
        '''    
        
        # Convert Degrees to Radians
        O_alt = O_alt*math.pi/180
        O_az = O_az*math.pi/180
        O_rot = O_rot*math.pi/180
        # Assemble Translation Matrix from Base (b) reference frame to Telescope (s)
        # Rotations Azimuth -> Altitude -> Image Rotations
        sTb = matmul(RotZ(self.phi_rot),matmul(RotZ(O_rot),matmul(RotX(-O_alt),matmul(RotX(self.phi_alt),matmul(RotY(-O_az),RotY(self.phi_az))))))

        # Calculate Rotated Positions
        P1_r = matmul(sTb,self.P1_h)
        P2_r = matmul(sTb,self.P2_h)
        P3_r = matmul(sTb,self.P3_h)
        OA_r = matmul(sTb,self.OA_h)
        
        # Calculate Length
        #L_p1 = pow(pow((P1_r[0][0]-self.P1_b[0][0]),2)+pow((P1_r[1][0]-self.P1_b[1][0]),2)+pow((P1_r[2][0]-self.P1_b[2][0]),2),.5)
        #L_p2 = pow(pow((P2_r[0][0]-self.P2_b[0][0]),2)+pow((P2_r[1][0]-self.P2_b[1][0]),2)+pow((P2_r[2][0]-self.P2_b[2][0]),2),.5)
        #L_p3 = pow(pow((P3_r[0][0]-self.P3_b[0][0]),2)+pow((P3_r[1][0]-self.P3_b[1][0]),2)+pow((P3_r[2][0]-self.P3_b[2][0]),2),.5)
        L_p1 = rootsumsquare(P1_r, self.P1_b)
        L_p2 = rootsumsquare(P2_r, self.P2_b)
        L_p3 = rootsumsquare(P3_r, self.P3_b)
		
        # Need to check that the lengths are legitimate
        if abs(L_p1<self.L_p1min) or abs(L_p2<self.L_p2min) or abs(L_p3<self.L_p3min):
            print('This position is beyond the minimum lengths')
        # Need to move actuators
        elif move == True:
            self.goToLengths(L_p1-self.L_p1min, L_p2-self.L_p2min, L_p3-self.L_p3min)
            
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
    
    def findHome(self):
        isOneHome = False
        isTwoHome = False
        isThreeHome = False
        self.actuatorOne.findHome()
        self.actuatorTwo.findHome()
        self.actuatorThree.findHome()
        
        while not(isOneHome and isTwoHome and isThreeHome):
            if self.actuatorOne.isHome():
                print('A1 home')
                isOneHome = True
            if self.actuatorTwo.isHome():
                isTwoHome = True
                print('A2 home')
            if self.actuatorThree.isHome():
                isThreeHome = True
                print('A3 home')


            time.sleep(.01)

        print("There's no place like home")

    def resetPositions(self):
        self.actuatorOne.resetPosition()
        self.actuatorTwo.resetPosition()
        self.actuatorThree.resetPosition()
        
    def goToLengths(self, lengthOne, lengthTwo, lengthThree):
        self.actuatorOne.commandLength(lengthOne)
        self.actuatorTwo.commandLength(lengthTwo)
        self.actuatorThree.commandLength(lengthThree)
        

        
        

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
