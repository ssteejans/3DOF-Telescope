# -*- coding: utf-8 -*-
"""
Sam Artho-Benz


Stepper Motor Test File
"""
import pyb
import l6470nucleo
import time
#import TelescopeDriver

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
data_1 = 0
data_2 = 0
Driver1.HardHiZ(1)
Driver1.HardHiZ(2)
Driver2.HardHiZ(1)
Driver1._get_params(0xD0,2)
print(data_1)
print(data_2)
#Driver1.go_to_position(2,128*200)

def estop():
    Driver1.HardHiZ(1)
    Driver1.HardHiZ(2)
    Driver2.HardHiZ(1)
def Go(speed):
    Driver1.Run(1, speed)
    Driver1.Run(2, speed)
    Driver2.Run(1, speed)
    
