# -*- coding: utf-8 -*-
"""
Created on Sun Apr 16 11:07:58 2017

@author: Artho-Bentz, Samuel
"""

class Actuator(object):
    ''' 
    @details Object which represents one linear actuator for a 3 DOF parallel actuator telescope mount
    @param stepperDriver stepper driver object which powers the actuator
    @param stepperResolution Resolution of stepper motor in steps / revolution
    @param rodResolution Resolution of actuator rod in inches / revolution
    '''
    
    def __init__(self, stepPerLength, motorController, motorNumber):
        self.stepPerLength = stepPerLength
        self.controller = motorController
        self.motorNumber = motorNumber
        self.currentLength = 0


    def commandLength(self, length):
        steps = self.__calcSteps(length)
        self.__commandStepMotion(steps)        
        
    def __calcSteps(self, length):
        steps = length*self.stepPerLength
        return steps
        
    def __commandStepMotion(self,steps):
        self.controller.GoTo(self.motorNumber, steps)
    
    def updateCurrentLength(self):
        self.currentLength = self.controller.getPositions(self.motorNumber)*self.stepPerLength
        
    def findHome(self):
        self.controller.Run(self.motorNumber, -10)
        
    def isHome(self):
        stalled = self.controller.isStalled(self.motorNumber, True)
        if stalled:
            self.controller.HardHiZ(self.motorNumber)
            return True
        else:
            return False
        
        