
# Resources
- [Quick Start Inverse3](https://docs.haply.co/docs/quick-start/)
- [Quick Start VerseGrip Stylus](https://docs.haply.co/docs/quick-start-verse-grip-stylus)
- [Developping with inverse 3](https://docs.haply.co/docs/developing-with-inverse3)
- [Haply Python API](https://haply.gitlab.io/Internal/hardware-api-python/index.html)
- [Python Samples](https://gitlab.com/Haply/public/python_samples)

# Setup
```bash
pip install HaplyHardwareAPI
sudo usermod -a -G dialout $USER
# then log out and log in
```

# ROS2 package
```bash
cd /your/ros2_ws/src
git clone https://gitlab.isir.upmc.fr/stech/haply_inverse_3/ros2_haply_inverse3_python.git
cd /your/ros2_ws
colcon build --symlink-install
```

# Before launching
- Connect Inverse 3 with USB cable
- Connect Versegrip USB receiver
- Check that both devices are connected :
```bash
ll /dev/serial/by-id
                total 0
                drwxr-xr-x 2 root root 80 mars  28 11:02 ./
                drwxr-xr-x 4 root root 80 mars  28 11:01 ../
                lrwxrwxrwx 1 root root 13 mars  28 11:01 usb-Teensyduino_USB_Serial_16021850-if00 -> ../../ttyACM0
                lrwxrwxrwx 1 root root 13 mars  28 11:02 usb-ZEPHYR_Haply_USB_Transceiver_7BD7C2F68DA7D969-if00 -> ../../ttyACM1
```

# Launching ROS
- 2 nodes :
    - haply_inverse3_pos : `ros2 run ros2_haply_inverse3_python haply_inverse3_pos`
    - haply_inverse3_quaternion : `ros2 run ros2_haply_inverse3_python haply_inverse3_quaternion`
- 3 topics :
    - `/haply_pos_vel` : [x, y, z, vx, vy, vz]
    - `/haply_quaternion`
    - `/haply_forces` : [fx, fy, fz]

Read topics pos_vel and quaternions for info
Write to `/haply_forces` to apply a force in N: `ros2 topic pub /haply_forces std_msgs/msg/Float32MultiArray "data: [0,0,0]"`

# Debug Inverse3 Led
Colour Codes
The Haply logo status light changes colour based on the device’s operating mode:

|||
|---|---|
|Blink (second on, second off)	|No power |
|Steady colour	|Powered|
|Red	|Connected to the computer, no data exchange|
|Purple (Magenta)	|Uncalibrated, handshake was made with the computer|
|Periodic Yellow Flash	|Connected to or receiving instruction from haply-inverse-service|
|White	|Idle, calibrated|
|Green	|In use: force, torque control|
|Real blue	|In use: position, angle control|
|Light blue (cyan)	|Safety|
|Intermittent white, yellow	|Connection timeout|