from essentials import *

robot = Robot()
base_pos = np.array([0,0,0])

ee_pos = robot.step_fk(base_pos)

print("EE position: ", ee_pos)

ee_target_pos = SE2(translation=[0.09,0], rotation=SO2(-np.pi/2))

cur_q = robot.get_q()
print("Current configuration: ", cur_q)

robot.step()