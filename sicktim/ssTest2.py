#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
#import Adafruit_MCP4725
import adafruit_mcp4725
import math
import board
import busio
# Create a DAC instance.
#dac1 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
#dac2 = Adafruit_MCP4725.MCP4725(address=0x61, busnum=1)

i2c = busio.I2C(board.SCL,board.SDA)
dac1 = adafruit_mcp4725.MCP4725(i2c, address=0x60)
dac2 = adafruit_mcp4725.MCP4725(i2c, address=0x61)


brakeDistance = 0.5
stairDistance = 10
breakVoltage = 0.4


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
    #message = min(regions, key=lambda x: regions[x])
    #print("Something is close to my" + message)
    print(regions['front'])
    
    if dangerDetection(regions['front']) or math.isinf(float(regions['front'])):
        state_description = 'Danger ahead, deciding where to turn'
        
        if dangerDetection(regions['fleft']) and dangerDetection(regions['fright']):
            #both sides are dangerous
            #turn robot until safe direction is found
            print("Both sides dangerous, turn left")

            turn("left")
        elif not dangerDetection(regions['fleft']) and dangerDetection(regions['fright']):
            #left is safe, turn here
            #print("Turn left")
            turn("left")
            
        elif not dangerDetection(regions['fright']) and dangerDetection(regions['fleft']):
            #right is safe, turn here
            #print("Turn right")
            turn("right")
        elif not dangerDetection(regions['fleft']) and not dangerDetection(regions['fright']):
            #Either side is safe
            if sum(ranges[60:99]) < sum(ranges[171:210]):    
                turn('left')
                print('Either side is safe, turning left')
            elif sum(ranges[60:99]) > sum(ranges[171:210]):
                turn('right')
                print('Either side is safe, turning right')
            else:
                print('Either side is safe, Error '+str(sum(ranges[55:99])))
        else:
            print("Error")
    else:
        #nothing up ahead, keep going straight
        state_description = 'Safe'
        print("Go forward")
        turn("straight")    
    """
    else:
        state_description = 'unknown case'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)
        rospy.loginfo(regions)
    """
    #time.sleep(0.1)
    rospy.loginfo(state_description)

def callback(data):
    global ranges
    ranges = list(data.ranges)

    for i in range(len(ranges)):
        if ranges[i] == float('inf'):
            ranges[i]=0
           
    regions = {
        #"right": min(min(data.ranges[50:74]),10),
        "fright": min(min(data.ranges[55:99]),10),
        "front": min(min(data.ranges[100:170]),10),
        "fleft": min(min(data.ranges[171:215]),10),
        #"left": min(min(data.ranges[196:215]),10)
        }
    
    take_actions(regions)
    #rospy.loginfo(regions['left'])
    #regions: 0:54, 55:109, 110:164, 165:217, 218:271
    #print "it works"



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) #listener

    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

