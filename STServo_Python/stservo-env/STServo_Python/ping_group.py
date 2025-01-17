import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *                   # Uses STServo SDK library

# Default setting
BAUDRATE                = 1000000        # STServo default baudrate : 1000000
DEVICENAME              = "/dev/ttyS0"    # Use /dev/serial0 for GPIO serial communication on Raspberry Pi

NUM_SERVOS = 5

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sts(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Try to ping the STServos
for sts_id in range(0, NUM_SERVOS):
    # Get STServo model number
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(sts_id)
    if sts_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(sts_comm_result))
    else:
        print("[ID:%03d] ping Succeeded. STServo model number : %d" % (sts_id, sts_model_number))
    if sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))

# Close port
portHandler.closePort()
