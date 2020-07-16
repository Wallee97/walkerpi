#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time
import Adafruit_MCP4725
import math
import numpy
# Create a DAC instance.
dac1 = Adafruit_MCP4725.MCP4725(address=0x60, busnum=1)
dac2 = Adafruit_MCP4725.MCP4725(address=0x61, busnum=1)

brakeDistance = 0.35
stairDistance = 1.5
breakVoltage = 1500
ranges = list()
prevRanges = [0,0,0]
prevPrevRanges = [0,0,0]

def polar2Cart(angle, radius, tilt):
    x = radius * math.cos(math.radians(angle))*math.cos(math.radians(tilt))
    y = radius * math.sin(math.radians(angle))*math.cos(math.radians(tilt))
    return x,y

def distanceCalc(x1,x2,y1,y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
        

def segments(dataPoints):
    mergeDistance = 0.8
    downTiltAngle = 0
    startPoints = []
    stopPoints = []
    #start angle: 55 deg, end at 215
    for i,distance in enumerate(dataPoints[1:268]):
        #try:
        if dataPoints[i-1] == 0 and dataPoints[i] != 0: #or dataPoints i-1 to i > mergeDistance:
            startPoints.append(i)
        if dataPoints[i] != 0 and dataPoints[i+1] == 0:
            stopPoints.append(i)
        
        #convert the (polar) data points to points in a cartesian system
        #offset the angle such that X-axis is left-right, Y-axis is forward-backward
        
    if startPoints[0] > stopPoints[0]:
        startPoints.insert(0,0)
    if len(startPoints) > len(stopPoints):
        stopPoints.append(269)
    
    for i,stopIndex in enumerate(stopPoints[0:len(stopPoints)-1]):
        #calculate distances
        x1, y1 = polar2Cart(startPoints[i+1]+45,dataPoints[startPoints[i+1]],downTiltAngle)
        x2, y2 = polar2Cart(stopPoints[i]+45,dataPoints[stopPoints[i]],downTiltAngle)
        killList = []
        
        D = distanceCalc(x2,x1,y2,y1)
        
        if D < mergeDistance:
            killList.append(i)
            
    for i in reversed(killList):
        del startPoints[i+1]
        del stopPoints[i]
        #if the distance smaller than mergeDistance, merge
        
    print(str(startPoints) + "  " + str(stopPoints))
    
    return startPoints, stopPoints

def medianFilter(ranges, prevRanges, prevPrevRanges):
    result = [0]*269
    for i in range(len(ranges)-2):
        current=ranges[i:i+2] + prevRanges[i:i+2] + prevPrevRanges[i:i+2]
        """   
        while float('inf') in current:
            current.remove(float('inf'))
            """
        result[i] = numpy.nanmedian(current)
        if result[i] > stairDistance:
            result[i] = 0
    #print(result) 
    # If one of these 0-edges are found, it is either the start or end of an obstacle
    # At these points, save the data points of that obstacle in a list. One list for each obstacle
    # Keep a minimum size for each list to avoid outlier data points
    # The number of lists corresponds to the number of obstacles in a given scan
    # Decide if each obstacle is a line, circle, whatever
    # For now, only handle obstacles, and not stairs
    return result

"""
def turn(direction):
    print("Turning " + direction)
    if direction == "left":
        dac1.set_voltage(breakVoltage, True)
        dac2.set_voltage(0, True)
    elif direction == "right":
        dac1.set_voltage(0, True)
        dac2.set_voltage(breakVoltage, True)
    elif direction == "straight":
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)
    else:
        print("Faulty message")
        dac1.set_voltage(0, True)
        dac2.set_voltage(0, True)
"""
def dangerDetection(regionData):
    if regionData > stairDistance or regionData < brakeDistance:
        return True
    else:
        return False

def take_actions(regions):
    #message = min(regions, key=lambda x: regions[x])
    #print("Something is close to my" + message)
    #time.sleep(0.1)
    rospy.loginfo(state_description)

def callback(data):
    global ranges
    global prevRanges
    global pub
    prevPrevRanges = prevRanges
    prevRanges = ranges
    ranges = list(data.ranges)

    """
    for i in range(len(ranges)):
        if ranges[i] == float('inf'):
            ranges[i]=0
            """
    
    filteredRanges = medianFilter(ranges, prevRanges, prevPrevRanges)    
    startPoints, stopPoints = segments(filteredRanges)
    
    
    
    newLaserScan = data
    newLaserScan.ranges=filteredRanges
    pub.publish(newLaserScan)
    regions = {
        #"right": min(min(data.ranges[50:74]),10),
        "fright": min(min(data.ranges[55:99]),10),
        "front": min(min(data.ranges[100:170]),10),
        "fleft": min(min(data.ranges[171:215]),10),
        #"left": min(min(data.ranges[196:215]),10)
        }
    #rospy.loginfo(regions['left'])
    #regions: 0:54, 55:109, 110:164, 165:217, 218:271



def listener():
    global pub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) #listener

    rospy.Subscriber("scan", LaserScan, callback)
    pub = rospy.Publisher('scanFiltered', LaserScan, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


