from essentials import *

robot = Robot()

# robot.log_lock_event(7)

# sleep(0.5)

# robot.lock_anchor(7, False)

<<<<<<< HEAD



=======
>>>>>>> d8ef354 (feedback testing, lowered the ID7_MIN_LOCK_LOAD)
# unlock the grippers
robot.lock_anchor(6, False)
robot.lock_anchor(7, False)

sleep(2)

# # Set the robot actuators to the home pose
anchor6_locked = robot.lock_anchor_fb(6, True)
print(f"Anchor ID:6 locked: {anchor6_locked}")
anchor7_locked = robot.lock_anchor_fb(7, True)
print(f"Anchor ID:7 locked: {anchor7_locked}")



