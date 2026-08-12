from pathlib import Path
import json
import numpy as np

from essentials import *

INSTRUCT_DIR_PATH = Path(__file__).parent / 'instruction_files'

class InstructLoader:
    def __init__(self, robot: Robot, inst_dir_path: Path = INSTRUCT_DIR_PATH):
        self.inst_dir_path: Path = inst_dir_path
        self.robot = robot
        self.buffer: list[dict] = []
        self.sel_file = Path()

    def get_instruction_file_names(self) -> list:
        """
        Return a list of filenames (without the .json extension) found in `folder`.
        """
        p = self.inst_dir_path
        if not p.exists() or not p.is_dir():
            return []
        return sorted([f.stem for f in p.glob('*.json')])

    def set_file(self, file_name):
        """
        Adds .json extension and sets the file to be loaded.
        """
        file = file_name + '.json'
        self.sel_file = self.inst_dir_path / file

    def load_file(self) -> None:
        if not self.sel_file.exists():
            raise FileNotFoundError(f"Instruction file not found: {self.sel_file}")
        with self.sel_file.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, list):
            raise ValueError("Instruction file must contain a JSON list of commands.")
        self.buffer = data

    def _exec_command(self, cmd: dict):
        cmd_type = cmd.get('type')
        match cmd_type:

            case 'move_to_q':
                config = np.array(cmd['config'])
                self.robot.move_to_q(config)

            case 'pick':
                self.robot.pick_voxel()

            case 'place':
                layer = int(cmd['layer'])
                self.robot.place_voxel(layer=layer)

            case 'unlock':
                id = 0
                if cmd['gripper'] == 'front':
                    id = 7
                elif cmd['gripper'] == 'rear':
                    id = 6

                self.robot.lock_anchor(servo_id=id, lock=False)

            case 'lock':
                id = 0
                if cmd['gripper'] == 'front':
                    id = 7
                elif cmd['gripper'] == 'rear':
                    id = 6

                self.robot.lock_anchor(servo_id=id, lock=True)

            case 'lock_fb':
                id = 0
                fb = False

                if cmd['gripper'] == 'front':
                    id = 7
                elif cmd['gripper'] == 'rear':
                    id = 6

                fb = self.robot.lock_anchor_fb(servo_id=id, lock=True)
                if not fb:
                    return 'stop'

            case 'end':
                print("Session ended.")
                return 'stop'

            case _:
                print("Unkown command received!")


    
    def run(self) -> None:
        if not self.buffer:
            self.load_file()

        for cmd in self.buffer:
            if self._exec_command(cmd) == 'stop':
                break
