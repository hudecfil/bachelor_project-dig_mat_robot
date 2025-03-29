from essentials import *

robot = Robot()
base_pos = np.array([0,0,0])

ee_pos = robot.step_fk(base_pos)

print("EE position: ", ee_pos)

ee_target_pos = np.array([0.09, 0, 0])
q = robot.step_ik(ee_target_pos)
print("Configuration for given EE position: ", q)