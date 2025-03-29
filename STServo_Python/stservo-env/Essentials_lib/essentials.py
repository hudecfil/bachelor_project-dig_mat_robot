import sys
import os
import numpy as np
from time import sleep

core_path = os.path.join(os.path.dirname(__file__), 'core')
sys.path.insert(0, core_path)
from se2 import SE2
from so2 import SO2
from geometry import circle_circle_intersection

sys.path.append("..")
from STservo_sdk import *



STS_MOVING_SPEED = 2000 # Default: 2400
STS_MOVING_ACC = 50
SCS_MOVING_TIME = 0
SCS_MOVING_SPEED = 500

LOCK_POS = 35
UNLOCK_POS = 180
MANIP_DOWN = 45
MANIP_UP = 555

# STS zero position [steps]
STS1_ZERO = 2100
STS2_ZERO = 2050
STS3_ZERO = 2100
STS4_ZERO = 2175
STS5_ZERO = 2050

# STS limits [rad]
STS15_UP_LIM = np.pi
STS24_UP_LIM = (3*np.pi)/4
STS3_UP_LIM = 0
STS15_LOW_LIM = -np.pi
STS24_LOW_LIM = -(3*np.pi)/4
STS3_LOW_LIM = -(8*np.pi)/9 # Approx. 8deg from position when grippers on the neighbouring voxels

# Robot link parameters [m]
LEG_LENGTH = 0.180
GRIPPER_HEIGHT = 0.0568
J2_Y_POS = 0.0898

# Voxel parameters [m]
VOX_LATTICE_PITCH = 0.090

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
        self.sts_zero_points = [STS1_ZERO, STS2_ZERO, STS3_ZERO, STS4_ZERO, STS5_ZERO]
        self.sts_up_limits = np.array([STS15_UP_LIM, STS24_UP_LIM, STS3_UP_LIM, STS24_UP_LIM, STS15_UP_LIM])
        self.sts_low_limits = np.array([STS15_LOW_LIM, STS24_LOW_LIM, STS3_LOW_LIM, STS24_LOW_LIM, STS15_LOW_LIM])
        self.link_parameters = np.array([LEG_LENGTH, LEG_LENGTH, J2_Y_POS])
        self.rear_grip_pose = SE2(translation=[0,0], rotation=SO2(np.pi/2))
        self.base_pose = SE2(translation=[0,J2_Y_POS], rotation=SO2(np.pi/2))
        self.front_grip_pose = np.array([0.09,0,0])


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

    def STS_rad_to_steps(self, servo_id, rad) -> int:
        """ Function maps input angle in radians to STS travel steps.

            - Mid-point zero reference from self.sts_zero_points
            - CCW as positive rotation
            - Angle wrapped to range [-π, π] radians
        """
        # Wrap angle to [-π, π]
        rad = np.arctan2(np.sin(rad), np.cos(rad))

        # Retrieve the correct zero point for the servo
        zero_point = self.sts_zero_points[servo_id - 1]

        # Convert radians to steps
        if servo_id == 4:
            steps = int((rad * (4096 / (2*np.pi))) - zero_point)
        else:
            steps = int(zero_point + (rad * (4096 / (2*np.pi))))
        
        # Debugging output
        print(f"Servo {servo_id} | Input rad: {rad:.4f} | Zero: {zero_point} | Steps: {steps}")

        return steps

    
    def STS_steps_to_rad(self, servo_id, steps) -> float:
        """ Converts STS travel steps to radians.

            - Mid-point zero reference from self.sts_zero_points
            - CCW as positive rotation
            - Angle wrapped to range [-π, π] radians
        """
        zero_point = self.sts_zero_points[servo_id-1]

        if servo_id == 4:
            rad = (zero_point-steps)*((2*np.pi)/4096)
        else:
            rad = (steps-zero_point)*((2*np.pi)/4096)

        return rad


    def move_to_q(self, q: np.array = None):
        """ Move STS servos to the given configuration q .

            Args:
                q: configuration vector [rad]
        """
        # Init configuration q
        if q is None:
            q = np.zeros(self.num_sts)
        
        # Check if input q has the right size and the config q lies within the joint limits
        assert q.shape == (5,), "Length of the vector q must be 5!"
        assert np.all((self.sts_low_limits <= q) & (q <= self.sts_up_limits)), \
        f"Joint configuration {config} out of bounds! Must be between {lower_limits} and {upper_limits}."

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
                    # Get STServo#sts_id present position, speed, moving value
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

    def step_fk(self, base_pos: np.array) -> np.array:
        l1 = l4 = GRIPPER_HEIGHT
        l2 = l3 = LEG_LENGTH
        q_wrapped = self.get_q()
        q_unwrapped = np.unwrap(q_wrapped)
        theta_1, theta_2, theta_3, theta_4, theta_5 = q_unwrapped

        x_ee = base_pos[0] + l2*np.sin(theta_2) + l3*np.sin(theta_2 + theta_3) + l4*np.sin(theta_2 + theta_3 + theta_4)
        y_ee = base_pos[1]
        z_ee = base_pos[2] + l2*np.cos(theta_2) + l3*np.cos(theta_2 + theta_3) + l4*np.cos(theta_2 + theta_3 + theta_4)

        ee_pos = np.array([x_ee, y_ee, z_ee])
        return ee_pos

    def step_ik(self, ee_target_pos: np.array) -> np.array:
        assert ee_target_pos.shape == (3,), "Length of the vector ee_target_pos must be 3!"

        l1 = l2 = LEG_LENGTH
        a1 = l1
        a2 = l2
        x = ee_target_pos[0]
        y = ee_target_pos[2]

        q_res = np.zeros(self.num_sts)

        theta_1 = 0
        theta_5 = 0
        
        # Compute theta2 using equation (3)
        r = x**2 + y**2
        D = ((a1**2 + a2**2)**2 - r/r - (a1**2 + a2**2)**2)
        print(D)
        theta_3 = 2 * np.arctan2(np.sqrt(D), 1)  # Ensure correct quadrant selection

        # Compute theta1 using equation (2)
        theta_2 = np.arctan2(y, x) - np.arctan2(a2 * np.sin(theta_3), a1 + a2 * np.cos(theta_3))

        # Compute theta3 using equation (4)
        theta_4 = 3*np.pi - theta_2 - theta_3  # Convert 540 degrees to radians

        # Ensure angles are mapped to [-pi, pi]
        theta_2 = np.arctan2(np.sin(theta_2), np.cos(theta_2))
        theta_3 = np.arctan2(np.sin(theta_3), np.cos(theta_3))
        theta_4 = np.arctan2(np.sin(theta_4), np.cos(theta_4))

        q_res = np.array([theta_1, theta_2, theta_3, theta_4, theta_5])

        return q_res

    def ik_analytical(self, flange_pose_desired: SE2) -> list[np.ndarray]:
        """Compute IK analytically, return all solutions for joint limits being
        from -pi to pi for revolute joints -inf to inf for prismatic joints."""

        def normalize_angle(angle: float) -> float:
            """Normalize angle to interval of [-pi, pi]"""
            return (angle + np.pi) % (2 * np.pi) - np.pi
            #return np.arctan2(np.sin(angle), np.cos(angle))

        all_solutions = []

        # Get flange position, orientation and link parameters
        fl_des_pos = flange_pose_desired.translation
        fl_des_orient = flange_pose_desired.rotation.angle
        # Get base (j0) position, orientation
        j0_pos = self.base_pose.translation
        j0_orient = self.base_pose.rotation.angle

        l = np.copy(self.link_parameters)

        # Calculate position of j2 joint from flange position
        j2_pos = flange_pose_desired.translation - (l[2] * np.array([np.cos(fl_des_orient), np.sin(fl_des_orient)]))

        # Calculate intersection between circles with centers j2, j0 and radius l[1], l[0]
        j1_pos = circle_circle_intersection(j2_pos, l[1], j0_pos, l[0])

        # Calculate joint configurations for both solutions of intersection
        for j1 in j1_pos:
            q1 = np.arctan2(j1[1] - j0_pos[1], j1[0] - j0_pos[0]) - j0_orient
            q2 = np.arctan2(j2_pos[1] - j1[1], j2_pos[0] - j1[0]) - q1 - j0_orient
            q3 = fl_des_orient - q1 - q2 - j0_orient

            q = [q1, q2, q3]
            # Normalize angles
            q = [normalize_angle(q_i) for q_i in q]

            all_solutions.append(q)

        return all_solutions
        



        

      
