import sys
import os
import numpy as np
from time import sleep

import csv
import time
from datetime import datetime

sys.path.append("..")
from STservo_sdk import *

STS_MOVING_SPEED = 1500 # Default: 2400
STS_MOVING_ACC = 50
SCS_MOVING_TIME = 0
SCS_MOVING_SPEED = 500 # 500

LOCK_POS = 35
UNLOCK_POS = 180

MANIP_PICK = 55 # reaches over the right angle, when picking voxel to be sure that the voxel clicks into the manipulator 
MANIP_DOWN0 = 85
MANIP_DOWN1 = 565
MANIP_UP = 575

STS_ZERO_POINT = 2048

# STS limits [rad]
STS15_UP_LIM = np.pi
STS24_UP_LIM = (11*np.pi)/18 # (3*np.pi)/4
STS3_UP_LIM = 0
STS15_LOW_LIM = -np.pi
STS24_LOW_LIM = -(11*np.pi)/18 # -(3*np.pi)/4
STS3_LOW_LIM = -(8*np.pi)/9 # Approx. 8deg from position when grippers on the neighbouring voxels

# Robot link parameters [m]
LEG_LENGTH = 0.179
GRIPPER_HEIGHT = 0.0568
BASE_Z_POS_OFF = 0.0878
BASE_Z_POS_OFF = 0.0878

# Voxel parameters [m]
VOX_LATTICE_PITCH = 0.090

# Anchor actuators load thresholds
ID6_MIN_LOCK_LOAD = 222 #241
ID6_MAX_LOCK_LOAD = 309 #328
ID6_JAM_THRESHOLD = 591 #615
ID6_TRAVEL_EFFORT_MAX_AVG = 298 #296

ID7_MIN_LOCK_LOAD = 200 #208
ID7_MAX_LOCK_LOAD = 296 #328
ID7_JAM_THRESHOLD = 636 #615
ID7_TRAVEL_EFFORT_MAX_AVG = 284 #296

# Split during LOCK into TRAVEL and ENGAGEMENT zones at 80 steps
ZONE_SPLIT_POS = 80 

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
        self.sts_up_limits = np.array([STS15_UP_LIM, STS24_UP_LIM, STS3_UP_LIM, STS24_UP_LIM, STS15_UP_LIM])
        self.sts_low_limits = np.array([STS15_LOW_LIM, STS24_LOW_LIM, STS3_LOW_LIM, STS24_LOW_LIM, STS15_LOW_LIM])
        self.link_parameters = np.array([LEG_LENGTH, LEG_LENGTH, BASE_Z_POS_OFF])
        self.calibrations = {
            6: {
                "MIN_LOCK_LOAD": ID6_MIN_LOCK_LOAD,
                "MAX_LOCK_LOAD": ID6_MAX_LOCK_LOAD,
                "JAM_THRESHOLD": ID6_JAM_THRESHOLD,
                "TRAVEL_EFFORT_MAX_AVG": ID6_TRAVEL_EFFORT_MAX_AVG
            },
            7: {
                "MIN_LOCK_LOAD": ID7_MIN_LOCK_LOAD,
                "MAX_LOCK_LOAD": ID7_MAX_LOCK_LOAD,
                "JAM_THRESHOLD": ID7_JAM_THRESHOLD,
                "TRAVEL_EFFORT_MAX_AVG": ID7_TRAVEL_EFFORT_MAX_AVG
            }
        }

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
        zero_point = STS_ZERO_POINT

        # Convert radians to steps
        if servo_id in [1,4,5]:
            steps = int(zero_point - (rad * (4096 / (2*np.pi))))
        else:
            steps = int(zero_point + (rad * (4096 / (2*np.pi))))

        # Debugging output
        #print(f"Servo {servo_id} | Input rad: {rad:.4f} | Zero: {zero_point} | Steps: {steps}")

        return steps

    
    def STS_steps_to_rad(self, servo_id, steps) -> float:
        """ Converts STS travel steps to radians.

            - Mid-point zero reference 2048 steps
            - Matches the logic of STS_rad_to_steps
        """
        zero_point = STS_ZERO_POINT

        # Scaling factor
        scale = (2 * np.pi) / 4096

        if servo_id in [1, 4, 5]:
            rad = (zero_point - steps) * scale
        else:
            rad = (steps - zero_point) * scale

        # Wrap angle into [-pi, pi] interval
        rad = np.arctan2(np.sin(rad), np.cos(rad))

        return rad

##################################################### MAIN FUNCTIONS #####################################################
    def move_to_q(self, q: np.array):
        """ Move STS servos to the given configuration q .

            Args:
                q: configuration vector [rad]
        """
        
        # Check if input q has the right size and the config q lies within the joint limits
        assert q.shape == (5,), "Length of the vector q must be 5!"
        assert np.all((self.sts_low_limits <= q) & (q <= self.sts_up_limits)), \
        f"Joint configuration {config} out of bounds! Must be between {lower_limits} and {upper_limits}."

        # Map q from rad to steps
        q = [self.STS_rad_to_steps(i+1, q_i) for i, q_i in enumerate(q)]
        print("q_steps: ", q)

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

        sleep(0.05) # Sets secure movement speed

        # Clear syncwrite parameter storage
        self.sts.groupSyncWrite.clearParam()


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
        

    def lock_anchor(self, servo_id, lock=False):
        """ Function locks/unlocks [True/False] the anchor with given servo_id """

        assert servo_id in self.scs_anchor_IDs, "Only SCS anchor IDs allowed!"

        sleep(0.5)
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
            
            
        sleep(0.5)

    
    def lock_anchor_fb(self, servo_id, lock=False):
        """ Function locks/unlocks [True/False] the anchor with given servo_id """

        assert servo_id in self.scs_anchor_IDs, "Only SCS anchor IDs allowed!"

        cal = self.calibrations.get(servo_id)
        if not cal:
            print(f"Error: No calibration data for ID {servo_id}")
            return False

        sleep(0.2)
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

        # Wait for the moving flag moving=1:
        sleep(0.05)

        # If UNLOCKING we don't monitor feedback
        if not lock:
            sleep(0.5)
            return True

        data_log = []
        start_time = time.time()

        # Position-Load feedback monitoring loop
        moving = 1
        while moving:
            scs_present_pos, scs_present_load, scs_comm_result, scs_error = self.scs.ReadPosLoad(servo_id)

            if scs_comm_result != COMM_SUCCESS:
                print(self.scs.getTxRxResult(scs_comm_result))
            else:
                elapsed = time.time() - start_time
                # Get absolute load of the servo movement – the direction of load is omitted
                abs_load = scs_present_load if scs_present_load < 1024 else scs_present_load - 1024
                data_log.append([round(elapsed, 4), scs_present_pos, abs_load])
                
                # Instant Safety Stop in the case of HARD JAM
                if abs_load > cal["JAM_THRESHOLD"]:
                    print(f"!!! EMERGENCY STOP: Jam detected at Pos {scs_present_pos} (Load {abs_load}) !!!")
                    # Return the servo to the UNLOCK_POS
                    scs_comm_result, scs_error = self.scs.WritePos(servo_id, UNLOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
                    if scs_comm_result != COMM_SUCCESS:
                        print("%s" % self.scs.getTxRxResult(scs_comm_result))
                    elif scs_error != 0:
                        print("%s" % self.scs.getRxPacketError(scs_error))
                    return False

            if scs_error != 0:
                print(self.scs.getRxPacketError(scs_error))

            if elapsed > 2.0:
                break

            # Check moving status
            moving, _, _ = self.scs.ReadMoving(servo_id)    
            sleep(0.005)

        # Analyze the LOCKING sequence
        travel_loads = [d[2] for d in data_log if d[1] > ZONE_SPLIT_POS]
        engagement_loads = [d[2] for d in data_log if d[1] <= ZONE_SPLIT_POS]

        # 1. Check if the anchor didn't scrape on the voxel lattice (isn't misaligned)
        # avg_travel = 0
        if travel_loads:
            avg_travel = sum(travel_loads) / len(travel_loads)
            print(f"Avg_travel: {avg_travel}")
            if avg_travel > cal["TRAVEL_EFFORT_MAX_AVG"]:
                print(f"FAILED: High friction during travel ({int(avg_travel)})")
                return False

        # 2. Check if the anchor is successfully LOCKed
        # –> if the load in the engagement zone isn't too low (in the air lock), or too high (jam)
        if engagement_loads:
            peak_lock = max(engagement_loads)
            if peak_lock < cal["MIN_LOCK_LOAD"]:
                print(f"FAILED: Air lock - No engagement detected ({peak_lock})")
                return False
            if peak_lock > cal["MAX_LOCK_LOAD"]:
                # Note: We already checked JAM_LIMIT, so this is just "Tight"
                print(f"SUCCESS: Tight lock confirmed ({peak_lock})")
                sleep(0.5)
                return True
            
            print(f"SUCCESS: Lock confirmed ({peak_lock})")
            sleep(0.5)
            return True

        print("FAILED: No engagement data recorded.")
        return False
            

    def log_lock_event(self, servo_id, label="log"):
        assert servo_id in self.scs_anchor_IDs, "Only SCS anchor IDs allowed!"

        # Generate timestamp for the filename
        timestamp = datetime.now().strftime("%d%m%y_%H%M%S")
        filename = f"calibration_logs/{label}_{timestamp}.csv"

        data_log = []
        start_time = time.time()

        print(f"Recording ID {servo_id} to {filename}...")

        # Command servo to lock
        scs_comm_result, scs_error = self.scs.WritePos(servo_id, LOCK_POS, SCS_MOVING_TIME, SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.scs.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.scs.getRxPacketError(scs_error))

        # Wait for movement to actually start (avoids the "1 data point" exit)
        start_time = time.time()
        for _ in range(5): # Check for 100ms
            moving, _, _ = self.scs.ReadMoving(servo_id)
            if moving: break
            time.sleep(0.005)

        moving = 1
        while moving:
            scs_present_pos, scs_present_load, scs_comm_result, scs_error = self.scs.ReadPosLoad(servo_id)

            if scs_comm_result != COMM_SUCCESS:
                print(self.scs.getTxRxResult(scs_comm_result))
            else:
                elapsed = time.time() - start_time
                # Get absolute load of the servo movement – the direction of load is omitted
                abs_load = scs_present_load if scs_present_load < 1024 else scs_present_load - 1024
                data_log.append([round(elapsed, 4), scs_present_pos, abs_load])
                # print("[ID:{servo_id:03d}] Pos:{scs_present_pos} AbsLoad:{abs_load}")
            if scs_error != 0:
                print(self.scs.getRxPacketError(scs_error))

            # Check movement status
            moving, _, _ = self.scs.ReadMoving(servo_id)    
            sleep(0.005)

        # Save datalog file
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Position", "Load"])
            writer.writerows(data_log)

        print(f"File saved: {filename}")


    def pick_voxel(self):
        self.move_manip(angle_steps=MANIP_PICK)
        self.lock_anchor(servo_id=9, lock=True)
        self.move_manip(angle_steps=MANIP_UP)


    def place_voxel(self, layer: int):
        assert (layer == -1 or layer == 0), "Layer must be -1 or 0!"

        if layer == -1:
            angle = MANIP_DOWN0
        if layer == 0:
            angle == MANIP_DOWN1

        self.move_manip(angle_steps=angle)
        self.lock_anchor(servo_id=9, lock=False)
        self.move_manip(angle_steps=MANIP_UP)

    def home_robot(self):
        # Set the robot actuators to the home pose
        self.lock_anchor(6, True)
        self.lock_anchor(7, True)
        self.move_manip(angle_steps=MANIP_UP)
        self.lock_anchor(9, False)

##################################################### MAIN FUNCTIONS END #####################################################


    def grab_rel_voxel(self, grab=False):
        """ Grab/release voxel [True/False] with voxel manipulator."""
        self.move_manip(down=True)
        self.lock_anchor(servo_id=9, lock=grab)
        self.move_manip(down=False)

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

    def move_manip(self, angle_steps):
        """ Move manipulator to given angle in steps. """
        scs_comm_result, scs_error = self.scs.WritePos(self.scs_manip_ID, angle_steps, SCS_MOVING_TIME, SCS_MOVING_SPEED)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.scs.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % self.scs.getRxPacketError(scs_error))
  
        sleep(2.5)

    def move_STS_rad(self, servo_id, rad):
        assert (self.sts_low_limits[servo_id-1] <= rad) and (rad <= self.sts_up_limits[servo_id-1]), \
        f"Joint configuration {config} out of bounds! Must be between {lower_limits} and {upper_limits}."

        steps = 0
        steps = self.STS_rad_to_steps(servo_id, rad)
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

        
            





        



        

      
