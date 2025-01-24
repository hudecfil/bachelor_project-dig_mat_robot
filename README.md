# Design and Construction of a Relative Robot for Manipulating Digital Materials

In my bachelor project I will be focusing on design and construction of a relative builder robot, that manipulates with digital
materials. Digital materials come in variety of mechanical properties, which are picked to suit given purpose. For our purpose of building structures we will use discrete metamaterials subunits called VOXELs. Voxels are simply "cubes" formed of 6 faces - lattices.
Those lattices can have earlier mentioned various properties, for example they can be just rigid structures keeping its shape, or they can be able to squeeze in certain directions. Voxels that I'll be using are rigid, but equiped with magnetic joints, so they can be joined together just like LEGO bricks.

## Designing the voxels

First of all I have to design my voxels. I based it on design of simple rigid voxel. I added holes for magnets and edited corner brace - when robot docks on the voxel, cross-shaped anchor has to get through lattice. Whole design is made in Fusion 360. Each lattice is 3D printed from PETG. Specific print settings: sequential printing - printer finishes first one object and than moves to another; no infill - increased number of perimeters to 5; material - PETG;
temperatures (nozzle/heat-bed) - first lr.: 230°C/85°C, other lrs.: 240°C/90°C;  layer height - 0.15 mm.

### Magnetic voxel MK0

My first printed lattice was designed to test strenght of bond between two lattices. Each lattice is equiped with 4 pairs of magnets in each corner. We have to use pairs of magnets because when we put two mirrored lattices next to each other, the polarities of the magnets have to be opposite. Magnets are 3x3 mm (DxH) neodym cylinders, with magnetic
force of 290 g, which seemed sufficient. First idea of fixing the magnets in place was to glue them in the holes (with clearence 0.3 mm), but it would be too difficult when assembling larger number of voxels. Magnets fitted in holes perfectly.

### Magnetic voxel MK1

The updated version is designed for press-fitting the magnets. Therefore the holes are a bit smaller with 0.1 mm offset.

![Magnetic lattice MK1](images/IMG_3946.JPG)
![Magnetic voxel MK1](images/IMG_3981.JPG)

## Design of the robot

### Gripper anchor
When robot gripper lands on the voxel, the robot has to attach to it. For this purpose is the gripper equipped with servo actuated anchor.
The anchor is cross-shaped to pass through the voxel lattice, when it's 45° rotated. Servo then turns the anchor back to its base position,
and locks the gripper in place. I designed prototype which you can see on the picture below [ADD IMAGE]. I made the arms of anchor thicker,
to be more robust, when the robot leans with all its weight to it. I also added round corners to the endings of anchor arms, so the anchor
can lock in place smoothly, when its not positioned perfectly. Another problem that I found out was that the anchor is too far from the voxel
lattice, and that the gripper has wobble when locked on the voxel. So I made multiple spacers that changes distance from motor mount, and tested them.
I got best result with spacer that made the anchor to be without any tolerance.

### Gripper
The design of the gripper was quite challenging. [...COMPLETE THIS PARAGRAPH]

## Bring electronics to life

### Raspberry Pi Zero 2W
I don't want to connect monitor to RasPi so I will be using it in headless setup - connect via SSH. In future I am planning to run ROS2 on the controller, so I needed more capable and powerful controller board, than basic boards like Arduino or ESP.
The ROS2 doesn't support default Raspberry Pi OS, so I needed to run one of the newest versions of Ubuntu Server. I tried out the newest
Ubuntu Server 24.10, then the 24.04.1 LTS but non of them booted, only the Raspberry Pi OS did. Finally older version 22.04.5 LTS (Jammy) booted up,
and I could get into proper work. This version of Ubuntu is supported by Humble Hawksbill ROS2 distribution, which I probably will use.
I connected to RasPi using SSH and cloned my git repository with Waveshare ST series servos Python library. For coding I am using VSCode remote development extension, which installs
VSCode Server version on RasPi. Then I am able to edit code on remote RasPi conveniently from my laptop.

### ST3215 Servos & Bus Servo Adapter
I am using the Waveshare ST3215 are 30 kg.cm torque serial bus programmable servos. Thanks to half-duplex serial communication they also implement feedback
(position, rotation speed, load and input voltage), which substitutes need of any other kind of sensors on the robot. Actuators of those properties are ideal
for controlling travel motion of the robot.<br>
The ST/SC series servos are powered and controlled via Waveshare Bus Servo Adapter (driver board). The driver board can be controlled via USB-C or UART TX/RX pins.
In use with RasPi Zero board I am using the UART option. To set the UART communication on the driver board I had to place yellow jumper caps to the A position.
Using jumper wires I connected RasPi UART GPIOs 14 (TX), 15 (RX) and 13 (GND) to the driver board pins like: TX-TX, RX-RX, GND-GND.<br>
For first test I connected one ST servo to the driver board with 12V power supply from Li-Ion battery. In the Python library I changed the USB port to UART serial port
/dev/ttyS0. When I tried to run the ping.py demo script, which tests both direction communication between servo and Raspi. I ran into the first problem with serial port access permission denied.
So I used sudo chmod on the mentioned port device, but that solved the problem only until the next reboot. The second issue was that the ping to servo wasn't reliable. Respectively the feedback
did't work, cause when I tried to read speed and position, packets were never received. I thought that it could be caused by disturbance arising due to multiple grounds of power sources. So I soldered
wire splits, so the RasPi and the driver board are both powered from the same source, but that didn't help.<br>
From official [RasPi documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configure-uarts) I found out that RasPi Zero supports
two standarts of UART: mini UART (default primary), PLO11 (default secondary). The problem of the mini UART is that its baudrate is dependent on the CPU load. This is problem for higher baudrates like in my
case, the servos are set to 1 Mbd by default. PLO11 is more robust solution which isn't dependent on CPU load. It requires disabling Bluetooth module by adding `dtoverlay=disable-bt` line at the end of the /boot/firmware/config.txt file.
Then also disabling hciuart service, which initializes BT modem is needed: `sudo systemctl disable hciuart`. After reboot the UART communication
finally worked. Now I am using the /dev/ttyAMA0 port (default port for BT) instead of /dev/ttyS0. I solved the problem with permission denied by disabling the serial login shell: in the /boot/cmdline.txt remove text `console=serial0,115200`.<br>
With communication working stably I could get into setting the servos IDs. This is done by accessing the EPROM memory of the servo, and rewriting the ID value on the
`STS_ID = 5` address. I achieved controlling independently all five ST servos connected in series.
![ST servos demo](images/IMG_4198.JPG)

### SC09 & SC15 Servos

