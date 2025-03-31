from essentials import *

robot = Robot()

base_pos = np.array([0,0,0])

ee_pos = robot.step_fk(base_pos)

print("EE position: ", ee_pos)

ee_target_pos = SE2(translation=[2*VOX_LATTICE_PITCH,0], rotation=SO2(-np.pi/2))

cur_q = robot.get_q()
print("Current configuration: ", cur_q)

robot.lock_anchor(7, True)
robot.grab_rel_voxel(True)
robot.lock_anchor(7, False)
robot.step(forward=False)
sleep(2)
robot.step(forward=True)
sleep(1)
robot.lock_anchor(7, True)
robot.grab_rel_voxel(False)
