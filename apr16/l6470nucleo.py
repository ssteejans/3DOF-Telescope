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
import BitUtilities
from math import ceil as math_ceil


## Maximum speed for the motor; default 65, or ~992 steps/s
MAX_SPEED = const (65)

## Acceleration for the motor; default 138, or 2008 steps/s^2
ACCEL = const (12)

## Deceleration for the motor; default 138, or 2008 steps/s^2
DECEL = const (12)

## The K_val constant for registers 0x09, 0x0A, 0x0B, 0x0C; default 0x29 = 41
K_VAL = const (200)   # Calculated 200??

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

## STALL_TH. Default of 2.03A
# pg 47 of L6470 Programming Manual for details
STALL_TH = const(7)


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
    
    # === DICTIONARIES ===
    """ Dictionary of available registers and their addresses.
    """
    REGISTER_DICT = {} #        ADDR | LEN |  DESCRIPTION     | xRESET | Write
    REGISTER_DICT['ABS_POS'   ]=[0x01, 22] # current pos      | 000000 |   S
    REGISTER_DICT['EL_POS'    ]=[0x02,  9] # Electrical pos   |    000 |   S
    REGISTER_DICT['MARK'      ]=[0x03, 22] # mark position    | 000000 |   W
    REGISTER_DICT['SPEED'     ]=[0x04, 20] # current speed    |  00000 |   R
    REGISTER_DICT['ACC'       ]=[0x05, 12] # accel limit      |    08A |   W
    REGISTER_DICT['DEC'       ]=[0x06, 12] # decel limit      |    08A |   W
    REGISTER_DICT['MAX_SPEED' ]=[0x07, 10] # maximum speed    |    041 |   W
    REGISTER_DICT['MIN_SPEED' ]=[0x08, 13] # minimum speed    |      0 |   S
    REGISTER_DICT['FS_SPD'    ]=[0x15, 10] # full-step speed  |    027 |   W
    REGISTER_DICT['KVAL_HOLD' ]=[0x09,  8] # holding Kval     |     29 |   W
    REGISTER_DICT['KVAL_RUN'  ]=[0x0A,  8] # const speed Kval |     29 |   W
    REGISTER_DICT['KVAL_ACC'  ]=[0x0B,  8] # accel start Kval |     29 |   W
    REGISTER_DICT['KVAL_DEC'  ]=[0x0C,  8] # decel start Kval |     29 |   W
    REGISTER_DICT['INT_SPEED' ]=[0x0D, 14] # intersect speed  |   0408 |   H
    REGISTER_DICT['ST_SLP'    ]=[0x0E,  8] # start slope      |     19 |   H
    REGISTER_DICT['FN_SLP_ACC']=[0x0F,  8] # accel end slope  |     29 |   H
    REGISTER_DICT['FN_SLP_DEC']=[0x10,  8] # decel end slope  |     29 |   H
    REGISTER_DICT['K_THERM'   ]=[0x11,  4] # therm comp factr |      0 |   H
    REGISTER_DICT['ADC_OUT'   ]=[0x12,  5] # ADC output       |     XX |
    REGISTER_DICT['OCD_TH'    ]=[0x13,  4] # OCD threshold    |      8 |   W
    REGISTER_DICT['STALL_TH'  ]=[0x14,  7] # STALL threshold  |     40 |   W
    REGISTER_DICT['STEP_MODE' ]=[0x16,  8] # Step mode        |      7 |   H
    REGISTER_DICT['ALARM_EN'  ]=[0x17,  8] # Alarm enable     |     FF |   S
    REGISTER_DICT['CONFIG'    ]=[0x18, 16] # IC configuration |   2E88 |   H
    REGISTER_DICT['STATUS'    ]=[0x19, 16] # Status           |   XXXX |
    REGISTER_DICT['RESERVED A']=[0x1A,  0] # RESERVED         |        |   X
    REGISTER_DICT['RESERVED B']=[0x1B,  0] # RESERVED         |        |   X
    # Write: X = unreadable, W = Writable (always), 
    #        S = Writable (when stopped), H = Writable (when Hi-Z)
    
    """ Dictionary for the STATUS register. Contains all error flags,
            as well as basic motor state information.
    """
    STATUS_DICT = {} # [    NAME    | OK/DEFAULT VALUE ]
    STATUS_DICT[14] = ['STEP_LOSS_B',1] # stall detection on bridge B
    STATUS_DICT[13] = ['STEP_LOSS_A',1] # stall detection on bridge A
    STATUS_DICT[12] = ['OVERCURRENT',1] # OCD, overcurrent detection
    STATUS_DICT[11] = ['HEAT_SHUTDN',1] # TH_SD, thermal shutdown
    STATUS_DICT[10] = ['HEAT_WARN  ',1] # TH_WN, thermal warning
    STATUS_DICT[ 9] = ['UNDERVOLT  ',1] # UVLO, low drive supply voltage
    STATUS_DICT[ 8] = ['WRONG_CMD  ',0] # Unknown command
    STATUS_DICT[ 7] = ['NOTPERF_CMD',0] # Command can't be performed
    
    STATUS_DICT[ 3] = ['SWITCH_EDGE',0] # SW_EVN, signals switch falling edge
    STATUS_DICT[ 2] = ['SWITCH_FLAG',0] # switch state. 0=open, 1=grounded
    
    STATUS_DICT[15] = ['STEPCK_MODE',0] # 1=step-clock mode, 0=normal
    STATUS_DICT[ 4] = ['DIRECTION'  ,1] # 1=forward, 0=reverse
    STATUS_DICT[ 6] = ['MOTOR_STAT' ,0] # two bits: 00=stopped, 01=accel
                                            #           10=decel,   11=const spd
    STATUS_DICT[ 1] = ['BUSY'       ,1] # low during movement commands
    STATUS_DICT[ 0] = ['Hi-Z'       ,1] # 1=hi-Z, 0=motor active
    
    """ Dictionary for Application Commands. See L6470 Programming manual Pg 56
            for information on usage. These commands must be OR'd with the
            values for use.
    
    """
    COMMAND_DICT = {} # [    NAME    | Command Hex Code |  Action ]
    COMMAND_DICT['SetParam'   ] = 0x00 # Writes VALUE in PARAM register 
    COMMAND_DICT['GetParam'   ] = 0x20 # Returns the stored value in PARAM register
    COMMAND_DICT['Run'        ] = 0x50 # Sets the target speed and direction
    COMMAND_DICT['StepClock'  ] = 0x58 # Puts the device in Step-clock mode and imposes direction
    COMMAND_DICT['Move'       ] = 0x40 # Moves specified number of steps in direction
    COMMAND_DICT['GoTo'       ] = 0x60 # Goes to specified ABS_POS (min path)
    COMMAND_DICT['GoTo_DIR'   ] = 0x68 # Goes to specified ABS_POS (forced direction)
    COMMAND_DICT['GoUntil'    ] = 0x82 # 
    COMMAND_DICT['ReleaseSW'  ] = 0x92 # 
    COMMAND_DICT['GoHome'     ] = 0x70 # 
    COMMAND_DICT['GoMark'     ] = 0x78 # 
    COMMAND_DICT['ResetPos'   ] = 0xD8 # 
    COMMAND_DICT['ResetDevice'] = 0xC0 # 
    COMMAND_DICT['SoftStop'   ] = 0xB0 # 
    COMMAND_DICT['HardStop'   ] = 0xB8 # 
    COMMAND_DICT['SoftHiZ'    ] = 0xA0 # 
    COMMAND_DICT['HardHiZ'    ] = 0xA8 # 
    COMMAND_DICT['GetStatus'  ] = 0xD0 # 
    
    

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
        self._set_par_1b ('KVAL_HOLD', K_VAL)
        self._set_par_1b ('KVAL_RUN', K_VAL)
        self._set_par_1b ('KVAL_ACC', K_VAL)
        self._set_par_1b ('KVAL_DEC', K_VAL)

        # Speed at which we transition from slow to fast V_B compensation
        self._set_par_2b ('INT_SPEED', INT_SPEED)

        # Acceleration and deceleration back EMF compensation slopes
        self._set_par_1b ('ST_SLP', ST_SLP)
        self._set_par_1b ('FN_SLP_ACC', ST_SLP)
        self._set_par_1b ('FN_SLP_DEC', ST_SLP)

        # Set the maximum speed at which motor will run
        self._set_par_2b ('MAX_SPEED', MAX_SPEED)

        # Set the maximum acceleration and deceleration of motor
        self._set_par_2b ('ACC', ACCEL)
        self._set_par_2b ('DEC', DECEL)
        
        # Set the number of Microsteps to use
        self._set_MicroSteps (SYNC_EN, SYNC_SEL, STEP_SEL)
        
        # Set the Stall Threshold
        self._setStallThreshold(STALL_TH)
        
        # Set Minimum Speed = 400 steps/s and turn on Low Speed Opt (pg 35)
        
        self._set_par_2b('MIN_SPEED', 0x168E)
        
        # Set motors in high impedence mode
        self.SoftHiZ(1)
        self.SoftHiZ(2)
        
    def _setStallThreshold(self, value):
        self._set_par_1b('STALL_TH', value)
        
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
        self._set_par_1b('STEP_MODE', SYNC_EN_Mask|stepval|SYNC_Select)
        
    ## Read a set of arguments which have been sent by the L6470's in
    #  response to a read-register command which has already been
    #  transmitted to the L6470's. The arguments are shifted into the
    #  integers supplied as parameters to this function.
    #  @param num_bytes The number of bytes to be read from each L6470
    
        
    def _read_bytes (self, num_bytes):

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
        #print("in read bytes motor 1 is " + str(bin(data_1)))
        #print("in read bytes motor 2 is " + str(bin(data_2)))
        return ([data_1, data_2])
    ## Send one byte to each L6470 as a command and receive two bytes
    #  from each driver in response. 
    #  @param command_byte The byte which is sent to both L6470's 
    #  @param recv_bytes The number of bytes to receive: 1, 2, or 3


    def _get_params (self, command_byte, recv_bytes):

        # Send the command byte, probably a read-something command
        self._sndbs (command_byte, command_byte)

        # Receive the bytes from the driver chips
        [data_1, data_2] = self._read_bytes (recv_bytes)

        
        return([data_1, data_2])
    ## Set a parameter to both L6740 drivers, in a register which needs 
    #  two bytes of data.0
    #  @param reg_name (string) The register to be set
    #  @param num The two-byte number to be put in that register

    def _set_par_2b (self, reg_name, num):
        reg = self.REGISTER_DICT[reg_name][0]
        self._sndbs (reg, reg)
        highb = (num >> 8) & 0x03
        self._sndbs (highb, highb)
        lowb = num & 0xFF
        self._sndbs (lowb, lowb)


    ## Set a parameter to both L6740 drivers, in a register which needs 
    #  one byte of data.
    #  @param reg_name (string) The register to be set
    #  @param num The one-byte number to be put in that register

    def _set_par_1b (self, reg_name, num):
        reg = self.REGISTER_DICT[reg_name][0]
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


#   ---------------- L6470 Built-in Functions------------------


    '''-------------------------------------------------------'''  
    
    ## Tell the motor to run at the given speed. The speed may be 
    #  positive, causing the motor to run in direction 0, or negative,
    #  causing the motor to run in direction 1. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param steps_per_sec The number of steps per second at which to
    #      go, up to the maximum allowable set in @c MAX_SPEED

    def Run (self, motor, steps_per_sec):

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
        self._cmd_3b (motor, self.COMMAND_DICT['Run'] | direc, 
                            steps_per_sec & 0x000FFFFF)
    
    '''-------------------------------------------------------'''
    
    def StepClock(self, direc = None):
        '''Needs Writing'''

    '''-------------------------------------------------------'''

    ## Tell motor driver @c motor to move @c num_steps in the direction
    #  @c direction. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param steps How many steps to move, in a 20 bit number 
    #  @param direc The direction in which to move, either 0 for one
    #      way or nonzero for the other; if unspecified, the sign of the
    #      number of steps will be used, positive meaning direction 0

    def Move (self, motor, steps, direc = None):

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
        self._cmd_3b (motor, self.COMMAND_DICT['Move'] | direc, steps & 0x003FFFFF)


    '''-------------------------------------------------------'''


    ## This command asks the specified motor to move to the specified 
    #  position. 
    #  @param motor Which motor to move, either 1 or 2
    #  @param pos_steps The position to which to move, in absolute steps

    def GoTo (self, motor, pos_steps):

        # Call the _cmd_3b() method to do most of the work
        self._cmd_3b (motor, self.COMMAND_DICT['GoTo'], pos_steps & 0x003FFFFF)


    '''-------------------------------------------------------'''

    def GoTo_DIR(self, motor, pos_steps, direc):
        '''Needs Writing'''
        
    '''-------------------------------------------------------'''

    def GoUntil (self, motor, action, steps_per_sec, direction):
        '''Needs Writing'''


    '''-------------------------------------------------------'''
    
    def ReleaseSW(self, motor, action, direc):
        '''Needs Writing'''
        
    '''-------------------------------------------------------'''
    
    def GoHome(self, motor):
        '''Needs Writing'''
        
    '''-------------------------------------------------------'''
    
    def GoMark(self, motor):
        '''Needs Writing'''
        
    '''-------------------------------------------------------'''

    ## This command sets the given motor driver's absolute position register
    #  to zero.
    #  @param motor The motor whose position is to be zeroed, either 1 or 2


    def ResetPos (self, motor):

        self._cmd_1b (motor, self.COMMAND_DICT['ResetPos'])

    '''-------------------------------------------------------'''
    
    def ResetDevice(self, motor):
        '''Needs Writing'''
        
    '''-------------------------------------------------------'''

    ## Tell a motor driver to decelerate its motor and stop wherever it ends
    #  up after the deceleration.
    #  @param motor Which motor is to halt, 1 or 2

    def SoftStop (self, motor):

        self._cmd_1b (motor, self.COMMAND_DICT['SoftStop'])

    '''-------------------------------------------------------'''

    ## Tell the specified motor to stop immediately, not even doing the usual
    #  smooth deceleration. This command should only be used when the compost
    #  is really hitting the fan because it asks for nearly infinite 
    #  acceleration of the motor, and this will probably cause the motor to
    #  miss some steps and have an inaccurate position count. 
    #  @param motor Which motor is to halt, 1 or 2

    def HardStop (self, motor):

        self._cmd_1b (motor, self.COMMAND_DICT['HardStop'])

    '''-------------------------------------------------------'''
    
    ## Tell the specified motor to decelerate smoothly from its motion, then
    #  put the power bridges in high-impedance mode, turning off power to the
    #  motor. 
    #  @param motor Which motor is to be turned off, 1 or 2

    def SoftHiZ (self, motor):

        self._cmd_1b (motor, self.COMMAND_DICT['SoftHiZ'])

    '''-------------------------------------------------------'''
    
    ## Tell the specified motor to stop and go into high-impedance mode (no
    #  current is applied to the motor coils) immediately, not even doing the 
    #  usual smooth deceleration. This command should only be used when the 
    #  compost is really hitting the fan because the motor is put into a
    #  freewheeling mode with no control of position except for the small 
    #  holding torque from the magnets in a PM hybrid stepper, and this will 
    #  probably cause the motor to miss some steps and have an inaccurate 
    #  position count. 
    #  @param motor Which motor is to be turned off, 1 or 2

    def HardHiZ (self, motor):

        self._cmd_1b (motor, self.COMMAND_DICT['HardHiZ'])

    '''-------------------------------------------------------'''
    
    def GetStatus (self, motor, verbose=0):
        status = self._get_params(self.COMMAND_DICT['GetStatus'], 2)
        if verbose:
            self.Print_Status(motor, status)
        return status
    '''-------------------------------------------------------'''
    
    ## Get the positions stored in the drivers for the selected motor. Each
    #  driver stores its motor's position in a 22-bit register. If only
    #  one position is needed, it's efficient to get both because the
    #  drivers are daisy-chained on the SPI bus, so we have to send two
    #  commands and read a bunch of bytes of data anyway. 
    #  @return The current positions of the selected motor
    
#   -----------------Secondary Functions-------------------

    '''-------------------------------------------------------'''
    
    def getPositions (self, motor):

        # Read (command 0x20) register 0x01, the current position
        [data_1, data_2] = self._get_params (self.COMMAND_DICT['GetParam']|self.REGISTER_DICT['ABS_POS'][0], 3)
        print("motor 1 is " + str(bin(data_1)))
        print("motor 2 is " + str(bin(data_2)))
        

        # Sign-extend the signed absolute position numbers to 32 bits
        '''  We Don't need to sign extend the number      
        if data_1 & 0x00400000:
            data_1 |= 0xFF800000
        else:
            data_1 &= 0x001FFFFF
        if data_2 & 0x00400000:
            data_2 |= 0xFF800000
        else:
            data_2 &= 0x001FFFFF
        '''
        if motor == 1:
            data = data_1
        else:
            data = data_2
        return (BitUtilities.GetTwosComplement(data, 22))
    '''-------------------------------------------------------'''
    def isStalled(self, motor, homing=False):
        status = self.GetStatus(1)
        if ((status[motor-1]&(1<<13) == 0) or (status[motor-1]&(1<<14) == 0)):
            if homing:
                self.ResetPos(motor)
            return True
        else:
            return False
        
    '''-------------------------------------------------------'''
    def Print_Status(self, motor, status):
        """ Formatted printing of status codes for the driver.

            @arg @c motor  (int): the motor which the status is representing.
            @arg @c status (int): the code returned by a GetStatus call.
        """
        # check error flags
        print ('Driver ', str(motor), ' Status: ') #, bin(status))
        print(status)
        for bit_addr in range(7,15):
            print('  Flag ', self.STATUS_DICT[bit_addr][0], ': ', end='')
            # we shift a 1 to the bit address, then shift the result down again
            if ((status[motor-1] & 1<<bit_addr)>>bit_addr)==self.STATUS_DICT[bit_addr][1]:
                # the result should either be a 1 or 0. Which is 'ok' depends.
                print("ok")
            else:
                print("Alert!")
        
        # check SCK_MOD
        if status[motor-1] & (1<<15):
            print('  Step-clock mode is on.')
        else:
            print("  Step-clock mode is off.")
        
        # check MOT_STATUS
        if status[motor-1] & (1<<6):
            if status[motor-1] & (1<<5):
                print("  Motor is at constant speed.")
            else:
                print("  Motor is decelerating.")
        else:
            if status[motor-1] & (1<<5):
                print("  Motor is accelerating.")
            else:
                print("  Motor is stopped.")
                
        # check DIR
        if status[motor-1] & (1<<4):
            print("  Motor direction is set to forward.")
        else:
            print("  Motor direction is set to reverse.")
            
        # check BUSY
        if not (status[motor-1] & (1<<1)):
            print("  Motor is busy with a movement command.")
        else:
            print("  Motor is ready to recieve movement commands.")
            
        # check HiZ
        if status[motor-1] & 1:
            print("  Bridges are in high-impedance mode (disabled).")
        else:
            print("  Bridges are in low-impedance mode (active).")
        
        # check SW_EVEN flag
        if status[motor-1] & (1<<3):
            print("  External switch has been clicked since last check.")
        else:
            print("  External switch has no activity to report.")
        # check SW_F
        if status[motor-1] & (1<<2):
            print("  External switch is closed (grounded).")
        else:
            print("  External switch is open.")
            

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
