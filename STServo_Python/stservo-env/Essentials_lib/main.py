from essentials import *
from client import RobotClient
from instruct_loader import InstructLoader
import inquirer

# def get_instruction_file_names(folder: Path | str = Path(__file__).parent / 'instruction_files') -> list:
#     """
#     Return a list of filenames (without the .json extension) found in `folder`.
#     """
#     p = Path(folder)
#     if not p.exists() or not p.is_dir():
#         return []
#     return sorted([f.stem for f in p.glob('*.json')])

# 1. Initialize dROBek
print(f"Initializing dROBek...")
robot = Robot()
robot.home_robot()

# 2. Ask user to select the WLAN/offline mode
ans1 = inquirer.prompt([inquirer.List('mode',
                                           message="Select the mode:",
                                           choices=['WLAN', 'Offline'],
                                           default='WLAN')])

if ans1.get('mode') == 'WLAN':
    # 2.A WLAN mode: create RobotClient instance, start listening for instructions from the server
    client = RobotClient(robot=robot, server_ip='192.168.1.43', port=9000)
    client.listen()  # Keep listening for incoming commands
    print("Connecting to the server...")

else:
    # 2.B Offline mode:
    inst_loader = InstructLoader(robot=robot)

    # 2.B.i Get the list of available instruction file names
    file_names = inst_loader.get_instruction_file_names()
    # 2.B.ii Ask user to select the instruction file
    ans2 = inquirer.prompt([inquirer.List('file',
                                          message="Select the desired structure shape:",
                                          choices=file_names)])
    inst_loader.set_file(ans2.get('file'))
    # sel_file_name = ans2.get('file')
    # sel_file = sel_file_name + '.json'
    # file_path = INSTRUCTION_DIR / sel_file

    # 2.B.iii Load the selected instruction file
    inst_loader.load_file()
    # 2.B.iv Start the build
    print("Press Enter to start building.")
    input()
    inst_loader.run()

print("Structure successfully built! Shutting down dROBek...")
robot.__del__()





