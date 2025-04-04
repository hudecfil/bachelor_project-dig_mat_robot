from essentials import *

robot = Robot()

cur_q = robot.get_q()
print("Current configuration: ", cur_q)

print("Current FG-base position: ", robot.front_grip_base_pose)
print("Current FG position: ", robot.front_grip_pose)

robot.step(forward=True)
sleep(1)
robot.step(forward=True)
sleep(1)
robot.step(forward=True)

# robot.lock_anchor(7, False)
# robot.step_front_gripper(True)
# sleep(1)
# robot.lock_anchor(7, True)
# print("Current FG-base position: ", robot.front_grip_base_pose)
# print("Current FG position: ", robot.front_grip_pose)

# print("Current RG-base position: ", robot.rear_grip_base_pose)
# print("Current RG position: ", robot.rear_grip_pose)
# robot.lock_anchor(6, False)
# robot.step_rear_gripper(True)
# sleep(1)
# robot.lock_anchor(6, True)
# print("Current RG-base position: ", robot.rear_grip_base_pose)
# print("Current RG position: ", robot.rear_grip_pose)

# robot.lock_anchor(6, False)
# robot.step_rear_gripper(False)
# sleep(1)
# robot.lock_anchor(6, True)
# print("Current RG-base position: ", robot.rear_grip_base_pose)
# print("Current RG position: ", robot.rear_grip_pose)

# robot.lock_anchor(7, False)
# robot.step_front_gripper(False)
# sleep(1)
# robot.lock_anchor(7, True)

# print("Current FG-base position: ", robot.front_grip_base_pose)
# print("Current FG position: ", robot.front_grip_pose)