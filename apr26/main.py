# -*- coding: utf-8 -*-
"""
Sam Artho-Benz


Stepper Motor Test File
"""
import pyb
import l6470nucleo
import time
import ActuatorDriver
import TelescopeDriver

## Stepper Resolution [steps/rev]
STEPPER_RESOLUTION = const (400)
STEPPER_RESOLUTION2 = const (200)
## Leadscrew Resolution [rev/in]
SCREW_RESOLUTION = const(16)

stby_rst_pin1 = pyb.Pin.cpu.B5
stby_rst_pin2 = pyb.Pin.cpu.B3
nCS1 = pyb.Pin.cpu.A4
nCS2 = pyb.Pin.cpu.A10
spi_number = 1
print('pins initialized')
spi_object = pyb.SPI (spi_number, mode=pyb.SPI.MASTER, 
                            baudrate=2000000, polarity=1, phase=1, 
                            bits=8, firstbit=pyb.SPI.MSB)
                            
Driver1 = l6470nucleo.Dual6470(spi_object,nCS1, stby_rst_pin1)
Driver2 = l6470nucleo.Dual6470(spi_object,nCS2, stby_rst_pin2)

Driver1._get_params(0xD0,2)


actuatorOne = ActuatorDriver.Actuator(STEPPER_RESOLUTION*SCREW_RESOLUTION, Driver1, 1)
actuatorTwo = ActuatorDriver.Actuator(STEPPER_RESOLUTION*SCREW_RESOLUTION, Driver1, 2)
actuatorThree = ActuatorDriver.Actuator(STEPPER_RESOLUTION2*SCREW_RESOLUTION, Driver2, 1)
Telescope = TelescopeDriver.Telescope(actuatorOne, actuatorTwo, actuatorThree)

#Driver1.go_to_position(2,128*200)

def estop():
    Driver1.HardHiZ(1)
    Driver1.HardHiZ(2)
    Driver2.HardHiZ(1)
def Go(speed):
    Driver1.Run(1, speed)
    Driver1.Run(2, speed)
    Driver2.Run(1, speed)
def getPos():
    print(['Actuator One is at: ' + str(actuatorOne.updateCurrentLength()+Telescope.L_p1min)])
    print(['Actuator Two is at: ' + str(actuatorTwo.updateCurrentLength()+Telescope.L_p2min)])
    print(['Actuator Three is at: ' + str(actuatorThree.updateCurrentLength()+Telescope.L_p3min)])
def getStatus(motornumber):
    if motornumber ==1:
        actuatorOne.controller.GetStatus(1, 1)
    elif motornumber ==2:
        actuatorTwo.controller.GetStatus(2, 1)
    elif motornumber ==3:
        actuatorThree.controller.GetStatus(1, 1)
    
