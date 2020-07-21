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
downTiltAngle = 0
breakVoltage = 1500
ranges = list()
prevRanges = [0,0,0]
prevPrevRanges = [0,0,0]

def createObstacleCluster(distances,obstacleCoords):
    listOfObstacles = []
    
    for i,angles in enumerate(startAngles):
        listOfObstacles.append(Obstacles(distances,obstacleCoords))
            

def polar2Cart(angle, radius):
    x = radius * math.cos(math.radians(angle))*math.cos(math.radians(downTiltAngle))
    y = radius * math.sin(math.radians(angle))*math.cos(math.radians(downTiltAngle))
    return x,y

def distanceCalc(x1,x2,y1,y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
        
def segments(dataPoints):
    mergeDistance = 0.8
    startAngles = []
    stopAngles = []
    obstacleAnglesList = []
    obstacleRadiusList = []
    #start angle: 55 deg, end at 215
    #finds start and stop points
    for i,distance in enumerate(dataPoints[1:268]):
        #try:
        if dataPoints[i-1] == 0 and dataPoints[i] != 0: #or dataPoints i-1 to i > mergeDistance:
            startAngles.append(i)
        if dataPoints[i] != 0 and dataPoints[i+1] == 0:
            stopAngles.append(i)
        
        #convert the (polar) data points to points in a cartesian system
        #offset the angle such that X-axis is left-right, Y-axis is forward-backward
    
    # Ensures proper conditions at beginning and end of each scan
    if startAngles[0] > stopAngles[0]:
        startAngles.insert(0,0)
    if len(startAngles) > len(stopAngles):
        stopAngles.append(269)
    
    # Merges close obstacles
    for i,stopIndex in enumerate(stopAngles[0:len(stopAngles)-1]):

        x1, y1 = polar2Cart(startAngles[i+1]+45,dataPoints[startAngles[i+1]])
        x2, y2 = polar2Cart(stopAngles[i]+45,dataPoints[stopAngles[i]])
        killList = []
        
        D = distanceCalc(x2,x1,y2,y1)
        
        if D < mergeDistance:
            killList.append(i)
    
    # Deletes gaps that are too small
    for i in reversed(killList):
        del startAngles[i+1]
        del stopAngles[i]
    
    obstacleAnglesList = list(zip(startAngles,stopAngles))
    for anglePair in obstacleAnglesList:
        obstacleRadiusList.append(dataPoints[ anglePair[0]:anglePair[1] ])
    # example: obstacleAnglesList = [ [15, 20], [33,46], [69,89], [125,148] ]
    # example: obstacleRadiusList = [ [0.1535, 0.1525, 0.1530, 0.1535, 0.1539], [0.7432, 0.7468, 0.7492, ...], ... ]
    return obstacleAnglesList, obstacleRadiusList

def medianFilter(ranges, prevRanges, prevPrevRanges):
    result = [0]*269
    for i in range(len(ranges)-2):
        current=ranges[i:i+2] + prevRanges[i:i+2] + prevPrevRanges[i:i+2]
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

def callback(data):
    global ranges
    global prevRanges
    global pub
    fullAngleList = []
    prevPrevRanges = prevRanges
    prevRanges = ranges
    ranges = list(data.ranges)

    filteredRanges = medianFilter(ranges, prevRanges, prevPrevRanges)    
    obstacleAnglesList, obstacleRadiusList = segments(filteredRanges)
    for pair in obstacleAnglesList:
        fullAngleList.append(  list(range(pair[0], pair[1]))  )

    # Ex: fullAngleList =[ [15, 16, ..., 19,  20], [33, 34, ..., 46], [69, ..., 89], [125, ... , 148] ] 

    obstacleClusters = createObstacleCluster(fullAngleList,obstacleRadiusList, downTiltAngle)
    
    vizRanges = filteredRanges
    
    #for stopIndex,stopAngle in enumerate(stopAngles[0:len(stopAngles)-1]):
    #    vizRanges[stopAngle:startAngles[stopIndex+1]] = [0]*len(vizRanges[stopAngle:startAngles[stopIndex+1]])
    
    newLaserScan = data
    newLaserScan.ranges=vizRanges
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
    pub = rospy.Publisher('scanFiltered', LaserScan, queue_size=4)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


