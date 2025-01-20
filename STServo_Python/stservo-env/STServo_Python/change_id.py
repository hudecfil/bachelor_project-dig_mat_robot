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
STS_ID_current          = 1                # STServo ID : 1
STS_ID_changeto         = 5
BAUDRATE                = 1000000        # STServo default baudrate : 1000000
DEVICENAME              = "/dev/ttyAMA0"    # Use /dev/serial0 for GPIO serial communication on Raspberry Pi

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

#---------------------------Change servo ID--------------------------
# Unlock EPROM
result, error = packetHandler.unLockEprom(STS_ID_current)
print(f"Unlock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Change SERVO ID
result, error = packetHandler.write1ByteTxRx(STS_ID_current, STS_ID, STS_ID_changeto)
print(f"Change ID Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Lock EPROM
result, error = packetHandler.LockEprom(STS_ID_changeto)
print(f"Lock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")
#---------------------------Change servo ID--------------------------

# Close port
portHandler.closePort()