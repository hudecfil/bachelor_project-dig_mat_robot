from essentials import *
from client import RobotClient

robot = Robot()

# Set the robot actuators to the home pose
robot.lock_anchor(6, True)
robot.lock_anchor(7, True)
robot.move_manip(angle_steps=MANIP_UP)
robot.lock_anchor(9, False)

client = RobotClient(robot=robot, server_ip='192.168.1.93', port=9000)
client.listen()  # This will keep listening for commands from the server

robot.__del__()





