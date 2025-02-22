import sys
import os
from time import sleep

from STservo_sdk import *

SCS_MOVING_TIME = 0
SCS_MOVING_SPEED = 500

LOCK_POS = 35
UNLOCK_POS = 180

class Robot:
    def __init__(self, baudrate = 1000000, deviceName = "/dev/ttyAMA0"):
        self.baudrate = baudrate
        self.device = deviceName
        self.portHandler = PortHandler(self.deviceName)
        self.sts = sts(self.portHandler)
        self.scs = scscl(self.portHandler)
        self.sts_IDs = [1,2,3,4,5]
        self.scs_anchor_IDs = [6,7,9]
        self.scs_manip_ID = [8]

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

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def lock_anchor(self, servo_id, lock=False):
        assert servo_id == 6 or servo_id == 7 or servo_id == 9

        if lock == True:
            scs_comm_result, scs_error = packetHandler.WritePos(servo_id, LOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
        else:
        scs_comm_result, scs_error = packetHandler.WritePos(servo_id, UNLOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
        sleep(2)

    def move_to_q(self, angle):
        

      
