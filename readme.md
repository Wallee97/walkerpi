# RT-walker 2019-2020

This project was carried out by Eric Trollsås and Valentine Lin

The software was designed for the Raspberry Pi 4 and SICK tim 310 laser scanner



Required packages are:
1. ROSpy
2. Adafruit CircuitPython MCP4725
3. Sick_tim drivers for ROS
4. python 3

To start the LIDAR:
1: `source setup.bash` from the walkerpi/sicktim/devel/ folder

2: `roslaunch sick_tim sick_tim310s01.launch` from any folder

To run python script:
`python3 ssTest3.py`
