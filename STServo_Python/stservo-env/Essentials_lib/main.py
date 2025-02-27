from essentials import *

robot = Robot()

#robot.grab_rel_voxel(grab=True)
#robot.grab_rel_voxel(grab=False)

cur_q = robot.get_q()
print("Current configuration: ", cur_q)

cur_q_steps = [robot.STS_rad_to_steps(i+1, q_i) for i, q_i in enumerate(cur_q)]
print("Current computed configuration: ", cur_q_steps)
#robot.move_STS_step(2, 3125)
#robot.move_STS_step(2, 2952)


