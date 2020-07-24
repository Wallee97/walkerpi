#!/usr/bin/env python

# Simple obstacle avoidance program for the RT Walker of Hirata Lab, Tohoku University.
# This program uses a SICK TIM310-1030000S01 LIDAR sensor together with a pair of Adafruit MCP4725 DAC modules
#     to actuate powder brakes via amplifiers on the Walker whenever obstacles are detected in front of the walker

# In order to start the program, first run the commands "source setup.bash" from the walkerpi/sicktim/devel folder, then start the
#     LIDAR with the command "roslaunch sick_tim sick_tim310s01.launch" from the walkerpi/sicktim/src/sick_tim/launch folder.
# Then start this script by running it with python3 in a separate terminal, using "python3 ssTest2.py" from its folder
# Alternatively, there is a quick start command by using "~/gitFolder/startUpScript.sh"

# Written by Valentine Lin and Eric Trollsaas, 2020
# Contact: valentinelin@protonmail.ch, eric.trollsas@gmail.com

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
import adafruit_mcp4725
import math
import board
import busio

i2c = busio.I2C(board.SCL,board.SDA)
dac1 = adafruit_mcp4725.MCP4725(i2c, address=0x60)
dac2 = adafruit_mcp4725.MCP4725(i2c, address=0x61)

brakeDistance = 0.5
stairDistance = 10 #Currently effectively unused, but may be useful for stair avoidance
breakVoltage = 0.4 #This can be given as any value between 0 and 1, where 1 is full braking power and 0 is no power.


def turn(direction):
    print("Turning " + direction)
    if direction == "left":
        dac1.normalized_value = breakVoltage
        dac2.normalized_value = 0
    elif direction == "right":
        dac1.normalized_value = 0
        dac2.normalized_value = breakVoltage
    elif direction == "straight":
        dac1.normalized_value = 0
        dac2.normalized_value = 0
    else:
        print("Faulty message")
        dac1.normalized_value = 0
        dac2.normalized_value = 0

def dangerDetection(regionData):
    if regionData > stairDistance or regionData < brakeDistance:
        return True
    else:
        return False

def take_actions(regions):
    
    print(regions['front'])
    
    if dangerDetection(regions['front']) or math.isinf(float(regions['front'])):
        state_description = 'Danger ahead, deciding where to turn'
        
        if dangerDetection(regions['fleft']) and dangerDetection(regions['fright']):
            #both sides are dangerous
            #turn robot until safe direction is found
            print("Both sides dangerous, turn left")

            turn("left")
        elif not dangerDetection(regions['fleft']) and dangerDetection(regions['fright']):
            #left is safe, turn left
            #print("Turn left")
            turn("left")
            
        elif not dangerDetection(regions['fright']) and dangerDetection(regions['fleft']):
            #right is safe, turn right
            #print("Turn right")
            turn("right")
        elif not dangerDetection(regions['fleft']) and not dangerDetection(regions['fright']):
            #Either side is safe. Determine of left or right is best
            #Decision is made by a simple cost function:
            #Sum up the distances of the left and right region; whichever side is highest has less obstacles -> turn to that side
            if sum(ranges[55:99]) < sum(ranges[171:215]):    
                turn('left')
                print('Either side is safe, turning left')
            elif sum(ranges[55:99]) > sum(ranges[171:215]):
                turn('right')
                print('Either side is safe, turning right')
            else:
                print('Either side is safe, Error '+str(sum(ranges[55:99])))
        else:
            print("Error")
    else:
        #Nothing up ahead, keep going straight
        state_description = 'Safe'
        print("Go forward")
        turn("straight")    

    rospy.loginfo(state_description)

def callback(data):
    global ranges
    ranges = list(data.ranges)

    for i in range(len(ranges)):
        if ranges[i] == float('inf'):
            ranges[i]=0
           
    regions = {
        #Data is indexed from 0 to 270, with 0 being the back of the sensor, and then sweeping counterclockwise (viewed from top of the sensor)
        
        #More sophisticated obstacle avoidance algorithms may wish to use more regions such as "right" and "left" here
        #"right": min(min(data.ranges[50:74]),10),
        "fright": min(min(data.ranges[55:99]),10),
        "front": min(min(data.ranges[100:170]),10),
        "fleft": min(min(data.ranges[171:215]),10),
        #"left": min(min(data.ranges[196:215]),10)
        }
    
    take_actions(regions)
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

