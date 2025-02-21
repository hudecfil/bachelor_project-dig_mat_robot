import sys
import os
from time import sleep

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
ANCHOR_ID                   = 9                 # STServo ID : 1
MANIP_ID                    = 8
BAUDRATE                    = 1000000           # STServo default baudrate : 1000000
DEVICENAME                  = "/dev/ttyAMA0"    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MOVING_SPEED            = 500        # SC09 = 3073? max moving speed
SCS_MOVING_TIME             = 0

LOCK_POS = 35
UNLOCK_POS = 180
MANIP_DOWN = 50
MANIP_UP = 555



# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
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

# Write SC Servo goal position/moving speed/moving time
scs_comm_result, scs_error = scs.WritePos(MANIP_ID, MANIP_DOWN, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (MANIP_ID,  MANIP_DOWN, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break
sleep(5)

scs_comm_result, scs_error = scs.WritePos(ANCHOR_ID, LOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(ANCHOR_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (ANCHOR_ID, LOCK_POS, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(ANCHOR_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break
sleep(3)

scs_comm_result, scs_error = scs.WritePos(MANIP_ID, MANIP_UP, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (MANIP_ID, MANIP_UP, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break
sleep(5)

scs_comm_result, scs_error = scs.WritePos(MANIP_ID, MANIP_DOWN, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (MANIP_ID, MANIP_DOWN, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break
sleep(5)

scs_comm_result, scs_error = scs.WritePos(ANCHOR_ID, UNLOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(ANCHOR_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (ANCHOR_ID, UNLOCK_POS, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(ANCHOR_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break
sleep(3)

scs_comm_result, scs_error = scs.WritePos(MANIP_ID, MANIP_UP, SCS_MOVING_TIME, SCS_MOVING_SPEED)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % scs.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print("%s" % scs.getRxPacketError(scs_error))

while 1:
    # Read SC Servo present position
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = scs.ReadPosSpeed(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (MANIP_ID, MANIP_UP, scs_present_position, scs_present_speed))
    if scs_error != 0:
        print(scs.getRxPacketError(scs_error))

    # Read SC servo moving status
    moving, scs_comm_result, scs_error = scs.ReadMoving(MANIP_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(scs.getTxRxResult(scs_comm_result))

    if moving==0:
        break


# Close port
portHandler.closePort()
