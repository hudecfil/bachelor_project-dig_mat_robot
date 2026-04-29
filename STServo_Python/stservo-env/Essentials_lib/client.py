import socket
import json
from essentials import *

class RobotClient:
    def __init__(self, robot: Robot, server_ip='192.168.1.60', port=9000):
    def __init__(self, robot: Robot, server_ip='192.168.1.60', port=9000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((server_ip, port))
        # self.buffer = b''
        self.robot = robot
        print("Connected to the server, ready for commands.")

    # def receive_command(self):
    #     while b'\n' not in self.buffer:
    #         data = self.sock.recv(1024)
    #         if not data:
    #             return None
    #         self.buffer += data
    #     line, self.buffer = self.buffer.split(b'\n', 1)
    #     return json.loads(line.decode('utf-8'))

    def listen(self):
        buffer = ""
        while True:
            data = self.sock.recv(1024).decode('utf-8')
            if not data:
                break
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                command = json.loads(line)
                result = self.handle_command(command)
                if result == 'stop':
                    return  # Exit listen loop after stop_com

    def handle_command(self, command):
        print("Received command:", command)

        com_type = command.get('type')
        match com_type:

            case 'move_to_q':
                config = np.array(command['config'])
                self.robot.move_to_q(config)

            case 'pick':
                self.robot.pick_voxel()

            case 'place':
                layer = int(command['layer'])
                self.robot.place_voxel(layer=layer)

            case 'unlock':
                id = 0
                if command['gripper'] == 'front':
                    id = 7
                elif command['gripper'] == 'rear':
                    id = 6

                self.robot.lock_anchor(servo_id=id, lock=False)

            case 'lock':
                id = 0
                if command['gripper'] == 'front':
                    id = 7
                elif command['gripper'] == 'rear':
                    id = 6

                self.robot.lock_anchor(servo_id=id, lock=True)

            case 'lock_fb':
                id = 0
                fb = False

                if command['gripper'] == 'front':
                    id = 7
                elif command['gripper'] == 'rear':
                    id = 6

                fb = self.robot.lock_anchor_fb(servo_id=id, lock=True)
                if not fb:
                    return 'stop'

            case 'stop_com':
                self.close()
                print("Communication session ended, port closed.")
                return 'stop'

            case _:
                print("Unkown command received!")

    def close(self):
        self.sock.close()