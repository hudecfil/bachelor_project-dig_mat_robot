import sys
import numpy as np
from time import sleep

sys.path.append("..")
from STservo_sdk import *

STS_MOVING_SPEED = 2400
STS_MOVING_ACC = 50
SCS_MOVING_TIME = 0
SCS_MOVING_SPEED = 500

LOCK_POS = 35
UNLOCK_POS = 180
MANIP_DOWN = 45
MANIP_UP = 555

# Servos zero position [steps]
STS1_ZERO = 2100
STS2_ZERO = 2050
STS3_ZERO = 2100
STS4_ZERO = 2175
STS5_ZERO = 2050


class Robot:
    def __init__(self, baudrate = 1000000, deviceName = "/dev/ttyAMA0"):
        self.baudrate = baudrate
        self.deviceName = deviceName
        self.portHandler = PortHandler(self.deviceName)
        self.sts = sts(self.portHandler)
        self.scs = scscl(self.portHandler)
        self.sts_IDs = [1,2,3,4,5]
        self.scs_anchor_IDs = [6,7,9]
        self.scs_manip_ID = 8
        self.num_sts = len(self.sts_IDs)
        self.sts_max_limit = []
        self.sts_zero_points = [STS1_ZERO, STS2_ZERO, STS3_ZERO, STS4_ZERO, STS5_ZERO]

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def __del__(self):
        # Close port
        self.portHandler.closePort()

    def STS_rad_to_steps(self, servo_id, rad):
        """ Function maps input angle in radians to STS travel steps.

            - Mid-point zero reference from self.sts_zero_points
            - CCW as positive rotation
            - Angle wrapped to range [-π, π] radians
        """
        # Wrap angle to [-π, π]
        rad = np.arctan2(np.sin(rad), np.cos(rad))

        # Retrieve the correct zero point for the servo
        zero_point = self.sts_zero_points[servo_id - 1]

        # Convert radian to steps
        steps = int(zero_point + (rad * (4096 / (2*np.pi))))

        # Debugging output
        print(f"Servo {servo_id} | Input rad: {rad:.4f} | Zero: {zero_point} | Steps: {steps}")

        return steps

    
    def STS_steps_to_rad(self, servo_id, steps):
        """ Converts STS travel steps to radians.

            - Mid-point zero reference from self.sts_zero_points
            - CCW as positive rotation
            - Angle wrapped to range [-π, π] radians
        """
        zero_point = self.sts_zero_points[servo_id-1]

        rad = (steps - zero_point)*((2*np.pi)/4096)

        return rad


    def move_to_q(self, q: np.array = None):
        """ Move STS servos to the given configuration q .

            Args:
                q: configuration vector [rad]
        """
        # Init configuration q
        if q is None:
            q = np.zeros(self.num_sts)

        # Map q from rad to steps
        q = [self.STS_rad_to_steps(i+1, q_i) for i, q_i in enumerate(q)]

        groupSyncRead = GroupSyncRead(self.sts, STS_PRESENT_POSITION_L, 4)

        for sts_id in self.sts_IDs:
            # Add STServo#1~10 goal position\moving speed\moving accc value to the Syncwrite parameter storage
            sts_addparam_result = self.sts.SyncWritePosEx(sts_id, q[sts_id-1], STS_MOVING_SPEED,
                                                               STS_MOVING_ACC)
            if sts_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % sts_id)

        # Syncwrite goal position
        sts_comm_result = self.sts.groupSyncWrite.txPacket()
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.sts.getTxRxResult(sts_comm_result))

        # Clear syncwrite parameter storage
        self.sts.groupSyncWrite.clearParam()
        time.sleep(0.002)  # wait for servo status moving=1
        while 1:
            # Add parameter storage for STServos
            for sts_id in self.sts_IDs:
                sts_addparam_result = groupSyncRead.addParam(sts_id)
                if sts_addparam_result != True:
                    print("[ID:%03d] groupSyncRead addparam failed" % sts_id)

            sts_comm_result = groupSyncRead.txRxPacket()
            if sts_comm_result != COMM_SUCCESS:
                print("%s" % self.sts.getTxRxResult(sts_comm_result))

            sts_last_moving = 0
            for sts_id in self.sts_IDs:
                # Check if groupsyncread data of STServo#1~10 is available
                sts_data_result, sts_error = groupSyncRead.isAvailable(sts_id, STS_PRESENT_POSITION_L, self.num_sts)
                if sts_data_result == True:
                    # Get STServo#scs_id present position moving value
                    sts_present_position = groupSyncRead.getData(sts_id, STS_PRESENT_POSITION_L, 2)
                    sts_present_speed = groupSyncRead.getData(sts_id, STS_PRESENT_SPEED_L, 2)
                    sts_present_moving = groupSyncRead.getData(sts_id, STS_MOVING, 1)
                    # print(sts_present_moving)
                    print("[ID:%03d] PresPos:%d PresSpd:%d" % (
                    sts_id, sts_present_position, self.sts.sts_tohost(sts_present_speed, 15)))
                    if sts_present_moving == 1:
                        sts_last_moving = 1
                else:
                    print("[ID:%03d] groupSyncRead getdata failed" % sts_id)
                    continue
                if sts_error:
                    print(self.sts.getRxPacketError(sts_error))
            print("---")

            # Clear syncread parameter storage
            groupSyncRead.clearParam()
            if sts_last_moving == 0:
                break

    def get_q(self):
        cur_q = np.zeros(self.num_sts)

        groupSyncRead = GroupSyncRead(self.sts, STS_PRESENT_POSITION_L, 4)

        for sts_id in self.sts_IDs:
            # Add parameter storage for STServos
            sts_addparam_result = groupSyncRead.addParam(sts_id)
            if sts_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % sts_id)

        sts_comm_result = groupSyncRead.txRxPacket()
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.sts.getTxRxResult(sts_comm_result))

        for sts_id in self.sts_IDs:
            # Check if groupsyncread data of STServos
            sts_data_result, sts_error = groupSyncRead.isAvailable(sts_id, STS_PRESENT_POSITION_L, 4)
            if sts_data_result == True:
                # Get STServo#scs_id present position value
                sts_present_position = groupSyncRead.getData(sts_id, STS_PRESENT_POSITION_L, 2)
                cur_q[sts_id-1] = self.STS_steps_to_rad(sts_id, sts_present_position)
                print("[ID:%03d] PresPos:%d " % (sts_id, sts_present_position))
            else:
                print("[ID:%03d] groupSyncRead getdata failed" % sts_id)
                continue
            if sts_error != 0:
                print("%s" % self.sts.getRxPacketError(sts_error))

        groupSyncRead.clearParam()
        return cur_q

    def move_STS_step(self, servo_id=1, steps=2048):
        # Write STServo goal position/moving speed/moving acc
        sts_comm_result, sts_error = self.sts.WritePosEx(servo_id, steps, STS_MOVING_SPEED, STS_MOVING_ACC)
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.sts.getTxRxResult(sts_comm_result))
        elif sts_error != 0:
            print("%s" % self.sts.getRxPacketError(sts_error))

        while 1:
            # Read STServo present position
            sts_present_position, sts_present_speed, sts_comm_result, sts_error = self.sts.ReadPosSpeed(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(self.sts.getTxRxResult(sts_comm_result))
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (servo_id, steps, sts_present_position, sts_present_speed))
            if sts_error != 0:
                print(self.sts.getRxPacketError(sts_error))

            # Read STServo moving status
            moving, sts_comm_result, sts_error = self.sts.ReadMoving(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(self.sts.getTxRxResult(sts_comm_result))

            if moving==0:
                break

    def move_STS_rad(self, servo_id=1, rad=np.pi/4):
        steps = self.STS_rad_to_steps(servo_id, steps)
        # Write STServo goal position/moving speed/moving acc
        sts_comm_result, sts_error = self.sts.WritePosEx(servo_id, steps, STS_MOVING_SPEED, STS_MOVING_ACC)
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.sts.getTxRxResult(sts_comm_result))
        elif sts_error != 0:
            print("%s" % self.sts.getRxPacketError(sts_error))

        while 1:
            # Read STServo present position
            sts_present_position, sts_present_speed, sts_comm_result, sts_error = self.sts.ReadPosSpeed(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(self.sts.getTxRxResult(sts_comm_result))
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (servo_id, steps, sts_present_position, sts_present_speed))
            if sts_error != 0:
                print(self.sts.getRxPacketError(sts_error))

            # Read STServo moving status
            moving, sts_comm_result, sts_error = self.sts.ReadMoving(servo_id)
            if sts_comm_result != COMM_SUCCESS:
                print(self.sts.getTxRxResult(sts_comm_result))

            if moving==0:
                break

    def move_manip(self, down=False):
        """ Move manipulator up/down [True/False] the anchor with given servo_id """
        if down:
            scs_comm_result, scs_error = self.scs.WritePos(self.scs_manip_ID, MANIP_DOWN, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.scs.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % self.scs.getRxPacketError(scs_error))
        else:
            scs_comm_result, scs_error = self.scs.WritePos(self.scs_manip_ID, MANIP_UP, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.scs.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % self.scs.getRxPacketError(scs_error))
        sleep(5)

    def lock_anchor(self, servo_id, lock=False):
        """ Function locks/unlocks [True/False] the anchor with given servo_id """
        assert servo_id in self.scs_anchor_IDs, "Only SCS anchor IDs allowed!"

        if lock:
            scs_comm_result, scs_error = self.scs.WritePos(servo_id, LOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.scs.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % self.scs.getRxPacketError(scs_error))
        else:
            scs_comm_result, scs_error = self.scs.WritePos(servo_id, UNLOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % self.scs.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % self.scs.getRxPacketError(scs_error))
        sleep(2)

    def grab_rel_voxel(self, grab=False):
        """ Grab/release voxel [True/False] with voxel manipulator."""
        self.move_manip(down=True)
        self.lock_anchor(servo_id=9, lock=grab)
        self.move_manip(down=False)



        

      
