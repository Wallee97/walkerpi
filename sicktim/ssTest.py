#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
import Adafruit_MCP4725
import math
# Create a DAC instance.
dac1 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
dac2 = Adafruit_MCP4725.MCP4725(address=0x61, busnum=1)



def take_actions(regions):
    #message = min(regions, key=lambda x: regions[x])
    #print("Something is close to my" + message)
    
    brakeDistance = 0.5
    stairDistance = 1.3
    if regions['front'] > stairDistance or math.isinf(float(regions['front'])):
        state_description = 'case 0 - stairs'     
        dac1.set_voltage(1500, True)
        dac2.set_voltage(0, True)

    elif regions['fright'] > stairDistance+0.1:
        state_description = 'case 0 - stairs' 
        dac1.set_voltage(1500, True)
        dac2.set_voltage(0, True)

    elif regions['fleft'] > stairDistance+0.1:
        state_description = 'case 0 - stairs'
        dac2.set_voltage(0, True)
        dac2.set_voltage(1500, True)

    elif regions['front'] > brakeDistance and regions['fleft'] > brakeDistance and regions['fright'] > brakeDistance:
        state_description = 'case 1 - nothing'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)

    elif regions['front'] < brakeDistance and regions['fleft'] < brakeDistance and regions['fright'] < brakeDistance:
        state_description = 'case 7 - front and fleft and fright'

        dac1.set_voltage(1500, True)
        dac2.set_voltage(0, True)

            

    elif regions['front'] < brakeDistance and regions['fleft'] > brakeDistance and regions['fright'] > brakeDistance:
        state_description = 'case 2 - front'
        if (regions['left'] and regions['right']) > brakeDistance+2:
            dac1.set_voltage(1500, True)
            dac2.set_voltage(0, True)

        elif regions['left'] > regions['right']:
            print('left')
            dac1.set_voltage(1500, True)
            dac2.set_voltage(0, True)
        else:
            print('right')
            dac1.set_voltage(0, True)
            dac2.set_voltage(1500, True)
        
    elif regions['front'] > brakeDistance and regions['fleft'] > brakeDistance and regions['fright'] < brakeDistance:
        state_description = 'case 3 - fright'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)
        
    elif regions['front'] > brakeDistance and regions['fleft'] < brakeDistance and regions['fright'] > brakeDistance:
        state_description = 'case 4 - fleft'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)

    elif regions['front'] < brakeDistance and regions['fleft'] > brakeDistance and regions['fright'] < brakeDistance:
        state_description = 'case 5 - front and fright'
        dac1.set_voltage(1500, True)
        dac2.set_voltage(0, True)
        
    elif regions['front'] < brakeDistance and regions['fleft'] < brakeDistance and regions['fright'] > brakeDistance:
        state_description = 'case 6 - front and fleft'
        dac1.set_voltage(0, True)
        dac2.set_voltage(1500, True)


    elif regions['front'] > brakeDistance and regions['fleft'] < brakeDistance and regions['fright'] < brakeDistance:
        state_description = 'case 8 - fleft and fright'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)

    else:
        state_description = 'unknown case'
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)
        rospy.loginfo(regions)
    
    #time.sleep(0.1)
    
    #dac1.set_voltage(1500, True)
    #time.sleep(2.0)
    
    
    rospy.loginfo(state_description)

def callback(data):
    regions = {
        "right": min(min(data.ranges[50:74]),10),
        "fright": min(min(data.ranges[75:99]),10),
        "front": min(min(data.ranges[100:170]),10),
        "fleft": min(min(data.ranges[171:195]),10),
        "left": min(min(data.ranges[196:215]),10)
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
