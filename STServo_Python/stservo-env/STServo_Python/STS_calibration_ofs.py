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
sts_ID                  = 2                
BAUDRATE                = 1000000        # STServo default baudrate : 1000000
DEVICENAME              = "/dev/ttyAMA0"    # Use /dev/serial0 for GPIO serial communication on Raspberry Pi
sts_offset_l = 2

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
result, error = packetHandler.unLockEprom(sts_ID)
print(f"Unlock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Change SERVO ID
result, error = packetHandler.write2ByteTxRx(sts_ID, STS_OFS_L, 90)
print(f"Change Offset Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Lock EPROM
result, error = packetHandler.LockEprom(sts_ID)
print(f"Lock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")
#---------------------------Change servo ID--------------------------

# Close port
portHandler.closePort()