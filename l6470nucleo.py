# -*- coding: utf-8 -*-
#
## @file l6470nucleo.py
#  This file contains a driver for a dual L6470 stepper driver board which 
#  is part of the Nucleo package from ST Micro. It can control each of the
#  two L5470 chips which are on the board. 
#
#  In order to use a stepping motor, the constants @c MAX_SPEED, @c ACCEL, 
#  @e etc. need to be tuned for that motor using the methods shown at
#  @c http://www.st.com/resource/en/application_note/dm00061093.pdf. 
#
#  @author JR Ridgely


import pyb
import time


## Maximum speed for the motor; default 65, or ~992 steps/s
MAX_SPEED = const (65)

## Acceleration for the motor; default 138, or 2008 steps/s^2
ACCEL = const (12)

## Deceleration for the motor; default 138, or 2008 steps/s^2
DECEL = const (12)

## The K_val constant for registers 0x09, 0x0A, 0x0B, 0x0C; default 0x29 = 41
K_VAL = const (253)   # Calculated 200??

## Intersection speed for register 0x0D; default 0x0408 = 1032
INT_SPEED = const (6408)

## Startup slope for register 0x0E; default 0x19 = 25
ST_SLP = const (45)

## Final slope for registers 0x0F, 0x10; default 0x29 = 41
FN_SLP = const (215)

## Number of Microsteps, num which are powers of 2 up to 128 are acceptable.
STEP_SEL = const (1)

## Define SYNC_ENable bitmask. 0x80 for High, 0x00 for Low
SYNC_EN = const(0x00)   

## SYNC_SEL modes. Datasheet pg 46 for full information
SYNC_SEL = const(0x10)


## This class implements a driver for the dual L6470 stepping motor driver 
#  chips on a Nucleo IHM02A1 board. The two driver chips are connected in 
#  SPI daisy-chain mode, which makes communication a bit convoluted. 
#
#  NOTE: One solder bridge needs to be moved for the IHM02A1 to work 
#  with unmodified MicroPython. Bridge SB34 must be disconnected, and
#  bridge SB12 must be connected instead. This is because the SCK signal
#  for which the board is shipped is not the one which MicroPython uses
#  by default.

class Dual6470:

    ## Initialize the Dual L6470 driver. The modes of the @c CS and
    #  @c STBY/RST pins are set and the SPI port is set up correctly. 
    #  @param spi_object A SPI object already initialized. used to talk to the 
    #      driver chips, either 1 or 2 for most Nucleos
    #  @param cs_pin The pin which is connected to the driver chips' SPI
    #      chip select (or 'slave select') inputs, in a pyb.Pin object 
    #  @param stby_rst_pin The pin which is connected to the driver 
    #      chips' STBY/RST inputs, in a pyb.Pin object

    def __init__ (self, spi_object, cs_pin, stby_rst_pin):

        ## The CPU pin connected to the Chip Select (AKA 'Slave Select') pin
        #    of both the L6470 drivers. 
        self.cs_pin = cs_pin

        ## The CPU pin connected to the STBY/RST pin of the 6470's.
        self.stby_rst_pin = stby_rst_pin

        ## The SPI object, configured with parameters that work for L6470's.
        # pyb.SPI (spi_number, mode=pyb.SPI.MASTER, 
        #                    baudrate=2000000, polarity=1, phase=1, 
        #                    bits=8, firstbit=pyb.SPI.MSB)
        self.spi = spi_object

        # Make sure the CS and STBY/RST pins are configured correctly
        self.cs_pin.init (pyb.Pin.OUT_PP, pull=pyb.Pin.PULL_NONE)
        self.cs_pin.high ()
        self.stby_rst_pin.init (pyb.Pin.OUT_OD, pull=pyb.Pin.PULL_NONE)

        # Reset the L6470's
        stby_rst_pin.low ()
        time.sleep (0.01)
        stby_rst_pin.high ()

        # Set the registers which need to be modified for the motor to go
        # This value affects how hard the motor is being pushed
        self._set_par_1b (0x09, K_VAL)
        self._set_par_1b (0x0A, K_VAL)
        self._set_par_1b (0x0B, K_VAL)
        self._set_par_1b (0x0C, K_VAL)

        # Speed at which we transition from slow to fast V_B compensation
        self._set_par_2b (0x0D, INT_SPEED)

        # Acceleration and deceleration back EMF compensation slopes
        self._set_par_1b (0x0E, ST_SLP)
        self._set_par_1b (0x0F, ST_SLP)
        self._set_par_1b (0x10, ST_SLP)

        # Set the maximum speed at which motor will run
        self._set_par_2b (0x07, MAX_SPEED)

        # Set the maximum acceleration and deceleration of motor
        self._set_par_2b (0x05, ACCEL)
        self._set_par_2b (0x06, DECEL)
        
        # Set the number of Microsteps to use
        self._set_MicroSteps (SYNC_EN, SYNC_SEL, STEP_SEL)


    ## Set the number of Microsteps to use, the SYNC output frequency, and the
    # SYNC ENABLE bit.
    # @param SYNC_Enable A 1-bit integer which determines the behavior of the 
    # BUSY/SYNC output.
    # @param SYNC_Select A 3-bit integer which determines SYNC output frequency
    # @param num_STEP the integer number of microsteps, numbers which are 
    # powers of 2 up to 128 are acceptable.
    
    
    def _set_MicroSteps(self, SYNC_Enable, SYNC_Select, num_STEP):
                
        for stepval in range(0,8): #convert num_STEPS to 3-bit power of 2
            if num_STEP ==1:
                break
            num_STEP = num_STEP >>1
        if SYNC_Enable == 1:
            SYNC_EN_Mask = 0x80
        else:
            SYNC_EN_Mask = 0x00
        self._set_par_1b(0x16, SYNC_EN_Mask|stepval|SYNC_Select)
        
    ## Read a set of arguments which have been sent by the L6470's in
    #  response to a read-register command which has already been
    #  transmitted to the L6470's. The arguments are shifted into the
    #  integers supplied as parameters to this function.
    #  @param num_bytes The number of bytes to be read from each L6470
    #  @param data_1 A 32-bit integer to hold data from driver 1
    #  @param data_1 A 32-bit integer to hold data from driver 2        
        
    def _read_bytes (self, num_bytes, data_1, data_2):

        data_1 = 0
        data_2 = 0

        # Each byte which comes in is put into the integer as the least 
        # significant byte so far and will be shifted left to make room for
        # the next byte
        for index in range (num_bytes):
            self.cs_pin.low ()
            data_1 <<= 8
            data_1 |= (self.spi.recv (1))[0]
            data_2 <<= 8
            data_2 |= (self.spi.recv (1))[0]
            self.cs_pin.high ()
        print("in read bytes motor 1 is " + str(bin(data_1)))
        print("in read bytes motor 2 is " + str(bin(data_2)))
        return ([data_1, data_2])
    ## Send one byte to each L6470 as a command and receive two bytes
    #  from each driver in response. 
    #  @param command_byte The byte which is sent to both L6470's 
    #  @param recv_bytes The number of bytes to receive: 1, 2, or 3
    #  @param data_1 An integer to hold data from driver 1
    #  @param data_2 An integer to hold data from driver 2

    def _get_params (self, command_byte, recv_bytes, data_1, data_2):

        # Send the command byte, probably a read-something command
        self._sndbs (command_byte, command_byte)

        # Receive the bytes from the driver chips
        [data_1, data_2] = self._read_bytes (recv_bytes, data_1, data_2)

        
        return([data_1, data_2])
    ## Set a parameter to both L6740 drivers, in a register which needs 
    #  two bytes of data.0
    #  @param reg The register to be set
    #  @param num The two-byte number to be put in that register

    def _set_par_2b (self, reg, num):

        self._sndbs (reg, reg)
        highb = (num >> 8) & 0x03
        self._sndbs (highb, highb)
        lowb = num & 0xFF
        self._sndbs (lowb, lowb)


    ## Set a parameter to both L6740 drivers, in a register which needs 
    #  one byte of data.
    #  @param reg The register to be set
    #  @param num The one-byte number to be put in that register

    def _set_par_1b (self, reg, num):

        self._sndbs (reg, reg)
        self._sndbs (num, num)


    ## Send one command byte to each L6470. No response is expected.
    #  @param byte_1 The byte sent first; it goes to the second chip
    #  @param byte_2 The byte sent second, to go to the first chip

    def _sndbs (self, byte_1, byte_2):

        self.cs_pin.low ()
        self.spi.send (byte_1)
        self.spi.send (byte_2)
        self.cs_pin.high ()


    ## Send a command byte only to one motor. The other motor is sent a NOP
    #  command (all zeros).
    #  @param motor The number, 1 or 2, of the motor to receive the command

    def _cmd_1b (self, motor, command):

        if motor == 1:
            self._sndbs (command, 0x00)
        elif motor == 2:
            self._sndbs (0x00, command)
        else:
            raise ValueError ('Invalid L6470 motor number; must be 1 or 2')


    ## Send a command byte plus three bytes of associated data to one of
    #  the motors. 
    #  @param motor The motor to receive the command, either 1 or 2
    #  @param command The one-byte command to be sent to one motor
    #  @param data The data to be sent after the command, in one 32-bit
    #    integer

    def _cmd_3b (self, motor, command, data):

        # Break the integer containing the data into three bytes
        byte_2 = (data >> 16) & 0x0F
        byte_1 = (data >> 8) & 0xFF
        byte_0 = data & 0xFF

        # Send commands to one motor, NOP (zero) bytes to the other
        if motor == 1:
            self._sndbs (command, 0x00)
            self._sndbs (byte_2, 0x00)
            self._sndbs (byte_1, 0x00)
            self._sndbs (byte_0, 0x00)
        elif motor == 2:
            self._sndbs (0x00, command)
            self._sndbs (0x00, byte_2)
            self._sndbs (0x00, byte_1)
            self._sndbs (0x00, byte_0)
        else:
            raise ValueError ('Invalid L6470 motor number; must be 1 or 2')


    ## Get the positions stored in the drivers for the selected motor. Each
    #  driver stores its motor's position in a 22-bit register. If only
    #  one position is needed, it's efficient to get both because the
    #  drivers are daisy-chained on the SPI bus, so we have to send two
    #  commands and read a bunch of bytes of data anyway. 
    #  @return The current positions of the selected motor

    def get_positions (self, motor):

        data_1 = 0                     # The motor positions to be returned
        data_2 = 0

        # Read (command 0x20) register 0x01, the current position
        [data_1, data_2] = self._get_params (0x21, 3, data_1, data_2)
        print("motor 1 is " + str(bin(data_1)))
        print("motor 2 is " + str(bin(data_2)))
        # Sign-extend the signed absolute position numbers to 32 bits
        if data_1 & 0x00400000:
            data_1 |= 0xFF800000
        else:
            data_1 &= 0x001FFFFF
        if data_2 & 0x00400000:
            data_2 |= 0xFF800000
        else:
            data_2 &= 0x001FFFFF
        if motor == 1:
            data = data_1
        else:
            data = data_2
        return (data)


    ## Tell motor driver @c motor to move @c num_steps in the direction
    #  @c direction. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param steps How many steps to move, in a 20 bit number 
    #  @param direc The direction in which to move, either 0 for one
    #      way or nonzero for the other; if unspecified, the sign of the
    #      number of steps will be used, positive meaning direction 0

    def move (self, motor, steps, direc = None):

        # Figure out the intended direction
        if direc == None:
            if steps <= 0:
                direc = 1
                steps = -steps
            else:
                direc = 0
        else:
            if direc != 0:
                direc = 1

        # Call the _cmd_3b() method to do most of the work
        self._cmd_3b (motor, 0x40 | direc, steps & 0x003FFFFF)


    ## Tell the motor to run at the given speed. The speed may be 
    #  positive, causing the motor to run in direction 0, or negative,
    #  causing the motor to run in direction 1. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param steps_per_sec The number of steps per second at which to
    #      go, up to the maximum allowable set in @c MAX_SPEED

    def run (self, motor, steps_per_sec):

        # Figure out the direction from the sign of steps_per_sec
        if steps_per_sec < 0:
            direc = 0
            steps_per_sec = -steps_per_sec
        else:
            direc = 1

        # Convert to speed register value: multiply by ~(250ns)(2^28)
        steps_per_sec *= 67.108864
        steps_per_sec = int (steps_per_sec)

        # Have the _cmd_3b() method write the command to a driver chip
        self._cmd_3b (motor, 0x50 | direc, 
                            steps_per_sec & 0x000FFFFF)
    

    ## This command asks the specified motor to move to the specified 
    #  position. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param pos_steps The position to which to move, in absolute steps

    def go_to_position (self, motor, pos_steps):

        # Call the _cmd_3b() method to do most of the work
        self._cmd_3b (motor, 0x60, pos_steps & 0x003FFFFF)


#    # ------------------------------------------------------------------------
#
#    def go_until (self, motor, action, steps_per_sec, direction):
#        pass


    ## This command sets the given motor driver's absolute position register
    #  to zero.
    #  @param motor The motor whose position is to be zeroed, either 1 or 2

    def reset_position (self, motor):

        self._cmd_1b (motor, 0b11011000)


    ## Tell a motor driver to decelerate its motor and stop wherever it ends
    #  up after the deceleration.
    #  @param motor Which motor is to halt, 1 or 2

    def soft_stop (self, motor):

        self._cmd_1b (motor, 0b10110000)


    ## Tell the specified motor to stop immediately, not even doing the usual
    #  smooth deceleration. This command should only be used when the compost
    #  is really hitting the fan because it asks for nearly infinite 
    #  acceleration of the motor, and this will probably cause the motor to
    #  miss some steps and have an inaccurate position count. 
    #  @param motor Which motor is to halt, 1 or 2

    def hard_stop (self, motor):

        self._cmd_1b (motor, 0b10111000)


    ## Tell the specified motor to decelerate smoothly from its motion, then
    #  put the power bridges in high-impedance mode, turning off power to the
    #  motor. 
    #  @param motor Which motor is to be turned off, 1 or 2

    def soft_high_z (self, motor):

        self._cmd_1b (motor, 0b10100000)


    ## Tell the specified motor to stop and go into high-impedance mode (no
    #  current is applied to the motor coils) immediately, not even doing the 
    #  usual smooth deceleration. This command should only be used when the 
    #  compost is really hitting the fan because the motor is put into a
    #  freewheeling mode with no control of position except for the small 
    #  holding torque from the magnets in a PM hybrid stepper, and this will 
    #  probably cause the motor to miss some steps and have an inaccurate 
    #  position count. 
    #  @param motor Which motor is to be turned off, 1 or 2

    def hard_high_z (self, motor):

        self._cmd_1b (motor, 0b10101000)



#setLoSpdOpt(boolean enable);
#configSyncPin(byte pinFunc, byte syncSteps);
#configStepMode(byte stepMode);
#setMaxSpeed(float stepsPerSecond);
#setMinSpeed(float stepsPerSecond);
#setFullSpeed(float stepsPerSecond);
#setAcc(float stepsPerSecondPerSecond);
#setDec(float stepsPerSecondPerSecond);
#setOCThreshold(byte threshold);
#setPWMFreq(int divisor, int multiplier);
#setSlewRate(int slewRate);
#setOCShutdown(int OCShutdown);
#setVoltageComp(int vsCompMode);
#setSwitchMode(int switchMode);
#setOscMode(int oscillatorMode);
#setAccKVAL(byte kvalInput);
#setDecKVAL(byte kvalInput);
#setRunKVAL(byte kvalInput);
#setHoldKVAL(byte kvalInput);
