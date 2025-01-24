#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available STServo model on this example : All models using Protocol STS
# This example is tested with a STServo and an URT
#

import sys
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *                      # Uses STServo SDK library

# Default setting
STS_ID                      = 1                 # STServo ID : 1
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = "/dev/ttyAMA0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

STS_MINIMUM_POSITION_VALUE  = 0           # STServo will rotate between this value
STS_MAXIMUM_POSITION_VALUE  = 4095
STS_MOVING_SPEED            = 2400        # STServo moving speed
STS_MOVING_ACC              = 50          # STServo moving acc

SCS_MINIMUM_POSITION_VALUE  = 0           # SC Servo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE  = 165        # SC09 = 1023 max position
SCS_MOVING_SPEED            = 500        # SC09 = 3073? max moving speed
SCS_MOVING_TIME             = 0

index = 0
sts_goal_position = [STS_MINIMUM_POSITION_VALUE, STS_MAXIMUM_POSITION_VALUE] # ST goal position
scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE] # SC goal position

ST_NUM_SERVOS = 5
SC_NUM_SERVOS = 4

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
sts = sts(portHandler)
scs = scscl(portHandler)

    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # Move ST servos
    for servo_id in range(1,ST_NUM_SERVOS+1):
        # Write STServo goal position/moving speed/moving acc
        sts_comm_result, sts_error = sts.WritePosEx(servo_id, sts_goal_position[index], STS_MOVING_SPEED, STS_MOVING_ACC)
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % sts.getTxRxResult(sts_comm_result))
        elif sts_error != 0:
            print("%s" % sts.getRxPacketError(sts_error))

        while 1:
            # Read STServo present position
            sts_present_position, sts_present_speed, sts_comm_result, sts_error = sts.ReadPosSpeed(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(sts.getTxRxResult(sts_comm_result))
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (servo_id, sts_goal_position[index], sts_present_position, sts_present_speed))
            if sts_error != 0:
                print(sts.getRxPacketError(sts_error))

            # Read STServo moving status
            moving, sts_comm_result, sts_error = sts.ReadMoving(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(sts.getTxRxResult(sts_comm_result))

            #time.sleep(0.754)

            if moving==0:
                break

    # Move SC servos
    for servo_id in range(ST_NUM_SERVOS + 1, ST_NUM_SERVOS + SC_NUM_SERVOS + 1):
        # Write SC Servo goal position/moving speed/moving time
        scs_comm_result, scs_error = scs.WritePos(servo_id, scs_goal_position[index], SCS_MOVING_TIME, SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % scs.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % scs.getRxPacketError(scs_error))

        while 1:
            # Read SC Servo present position
            scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(servo_id)
            if scs_comm_result != COMM_SUCCESS:
                print(scs.getTxRxResult(scs_comm_result))
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (servo_id, scs_goal_position[index], scs_present_position, scs_present_speed))
            if scs_error != 0:
                print(scs.getRxPacketError(scs_error))

            # Read SC servo moving status
            moving, scs_comm_result, scs_error = scs.ReadMoving(servo_id)
            if scs_comm_result != COMM_SUCCESS:
                print(scs.getTxRxResult(scs_comm_result))

            time.sleep(0.754)

            if moving==0:
                break

        

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Close port
portHandler.closePort()
