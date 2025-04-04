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
SCS_ID                  = 6                # STServo ID : 1
BAUDRATE                = 1000000        # STServo default baudrate : 1000000
DEVICENAME              = "/dev/ttyAMA0"    # Use /dev/serial0 for GPIO serial communication on Raspberry Pi

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = scscl(portHandler)
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

print("SCS ID:", SCS_ID)

#---------------------------Change servo I coefficient--------------------------
# Unlock EPROM
result, error = packetHandler.unLockEprom(SCS_ID)
print(f"Unlock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Rear SERVO P coefficient
P_read, result, error = packetHandler.read1ByteTxRx(SCS_ID, SCSCL_P)
print(f"Read P coefficient Result: {packetHandler.getTxRxResult(result)}")
print("P = ", P_read)
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Rear SERVO D coefficient
D_read, result, error = packetHandler.read1ByteTxRx(SCS_ID, SCSCL_D)
print(f"Read D coefficient Result: {packetHandler.getTxRxResult(result)}")
print("D = ", D_read)
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Rear SERVO I coefficient
I_read, result, error = packetHandler.read1ByteTxRx(SCS_ID, SCSCL_I)
print(f"Read I coefficient Result: {packetHandler.getTxRxResult(result)}")
print("I = ", I_read)
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")

# Lock EPROM
result, error = packetHandler.LockEprom(SCS_ID)
print(f"Lock EEPROM Result: {packetHandler.getTxRxResult(result)}")
if error:
    print(f"Error: {packetHandler.getRxPacketError(error)}")
#---------------------------Change servo I coefficient--------------------------

# Close port
portHandler.closePort()