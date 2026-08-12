from essentials import *
from client import RobotClient
from instruct_loader import InstructLoader
# import inquirer
from pathlib import Path

INSTRUCT_DIR_PATH = Path(__file__).parent / "instruction_files"

robot = Robot()

print(f"Initializing dROBek...")
# Set the robot actuators to the home pose
robot.lock_anchor(6, True)
robot.lock_anchor(7, True)
robot.move_manip(angle_steps=MANIP_UP)
robot.lock_anchor(9, False)

inst_loader = InstructLoader(robot)

print(f"Enter the structure shape.")
file_name = input()
inst_loader.file_path = INSTRUCT_DIR_PATH / file_name
inst_loader.load_file()

print(f"Press any key to start the build.")
input()
inst_loader.run()


robot.__del__()





