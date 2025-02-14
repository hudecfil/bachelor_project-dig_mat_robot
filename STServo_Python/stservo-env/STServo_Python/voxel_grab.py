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
STS_ID                      = 8                 # STServo ID : 1
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = "/dev/ttyAMA0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
STS_MINIMUM_POSITION_VALUE  = 0        # STServo will rotate between this value
STS_MAXIMUM_POSITION_VALUE  = 458        # ST/SC09 = 1023/4095 max position
SCS_MOVING_SPEED            = 500        # SC09 = 3073? max moving speed
SCS_MOVING_TIME             = 0

index = 0
sts_goal_position = [STS_MINIMUM_POSITION_VALUE, STS_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = scscl(portHandler)
    
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


# Write STServo goal position/moving speed/moving acc
sts_comm_result, sts_error = packetHandler.WritePos(STS_ID, STS_MINIMUM_POSITION_VALUE, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if sts_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(sts_comm_result))
elif sts_error != 0:
    print("%s" % packetHandler.getRxPacketError(sts_error))

time.sleep(5)

sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(STS_ID)
if sts_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(sts_comm_result))
else:
    print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (STS_ID, STS_MINIMUM_POSITION_VALUE, sts_present_position, sts_present_speed))
if sts_error != 0:
    print(packetHandler.getRxPacketError(sts_error))

# Read STServo moving status
moving, sts_comm_result, sts_error = packetHandler.ReadMoving(STS_ID)
if sts_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(sts_comm_result))

# Write STServo goal position/moving speed/moving acc
sts_comm_result, sts_error = packetHandler.WritePos(STS_ID, STS_MAXIMUM_POSITION_VALUE, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if sts_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(sts_comm_result))
elif sts_error != 0:
    print("%s" % packetHandler.getRxPacketError(sts_error))


sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(STS_ID)
if sts_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(sts_comm_result))
else:
    print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (STS_ID, STS_MAXIMUM_POSITION_VALUE, sts_present_position, sts_present_speed))
if sts_error != 0:
    print(packetHandler.getRxPacketError(sts_error))

# Read STServo moving status
moving, sts_comm_result, sts_error = packetHandler.ReadMoving(STS_ID)
if sts_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(sts_comm_result))


# Close port
portHandler.closePort()
