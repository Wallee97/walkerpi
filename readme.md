# RT-walker 2019-2020

This project was carried out by Eric Trollsås and Valentine Lin

The software was designed for the Raspberry Pi 4 and SICK TIM310-1030000S01 laser scanner mounted on the RT Walker of Hirata Laboratory, Tohoku University

`ssTest2.py` is a naïve obstacle avoidance algorithm that detects obstacles near the Walker and attempts to avoid them by turning left or right using a basic cost function
`ssTest3.py` is an experimental obstacle detection and avoidance algorithm that attempts to find and classify nearby obstacles and attempts to avoid them. At the time of writing, this does not properly function due to a bug in the obstacle clustering function. Check each of these scripts for more detail.


Required packages are:
1. [ROSpy](https://wiki.ros.org/rospy)
2. [Adafruit CircuitPython MCP4725](https://github.com/adafruit/Adafruit_CircuitPython_MCP4725)
3. [Sick_tim drivers for ROS](https://github.com/uos/sick_tim)
4. [python 3](https://www.python.org/downloads/)

To start the LIDAR:

1: `source setup.bash` from the sicktim/devel/ folder

2: `roslaunch sick_tim sick_tim310s01.launch` from any folder

To run python script:
`python3 ssTest2.py`

To run experimental obstacle detection script:
`python3 ssTest3.py`
