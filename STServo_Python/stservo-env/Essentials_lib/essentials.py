import sys
import os
import time

from STservo_sdk import *

SCS_MOVING_TIME = 0
SCS_MOVING_SPEED = 500

LOCK_POS = 35
UNLOCK_POS = 180

def lock_anchor(servo_id, lock=True):
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