#!/usr/bin/env python

# This is an experimental algorithm based on the algorithm described below which aimed to detect and classify obstacles detected near the Walker
# It is currently unfinished, as it does not properly classify or place objects in space around the Walker.
# This was written based on the ssTest3 script. More explanations are described there, including how to start it.


# Based on an algorithm developed by Y.Peng, D. Qu, Y, Zhong, S. Xie, J. Luo, and J.Gu,
# "The Obstacle Detection and Obstacle Avoidance Algorithm Based on 2-D Lidar", 
# Proceeding of the 2015 IEEE International Conference on Information and Automation, Lijiang, China, August, 2015


import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
import time
import adafruit_mcp4725
import math
import numpy
import board
import busio
import ObstacleClustering
import matplotlib.pyplot as plt
import matplotlib.animation as animation


i2c = busio.I2C(board.SCL,board.SDA)

brakeDistance = 0.35
stairDistance = 0.6
downTiltAngle = 0
breakVoltage = 1500
ranges = list()
prevRanges = [0,0,0]
prevPrevRanges = [0,0,0]
#plotFigFlag = False



def generatePointCloud(obstacleCart):
    global cloud
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "laser"
    numberOfPoints = len(obstacleCart)
    cloud = PointCloud()
    cloud.header = h
    point = []
    for i in range(0,numberOfPoints):
        cloud.points.append(Point32( obstacleCart[i][0], obstacleCart[i][1], 0 ))
    return cloud

def generatePointList(obstacle):
    coords = []
    
    if obstacle.shape == "circle":
        r = obstacle.r
        x0 = obstacle.x0
        y0 = obstacle.y0
        theta = numpy.linspace(0, 2*math.pi, num=180)
        for i in theta:
            coords.append( list(polar2Cart(i, r)) )
            coords[-1][0] += x0
            coords[-1][1] += y0
        
    elif obstacle.shape == "line":
        p1 = obstacle.p1
        p2 = obstacle.p2
        length = int(100*distanceCalc(p1[0],p2[0],p1[1],p2[1])) #cm
        x = numpy.linspace(p1[0],p2[0],length)
        y = numpy.linspace(p1[0],p2[0],length)
        pair = list(zip(x,y))
        coords = pair
        
    elif obstacle.shape == "rectangle":
        p1 = obstacle.p1
        p2 = obstacle.p2
        p3 = obstacle.p3
        p4 = obstacle.p4
        length1 = int(100*distanceCalc(p1[0],p2[0],p1[1],p2[1]))
        length2 = int(100*distanceCalc(p2[0],p3[0],p2[1],p3[1]))
        length3 = int(100*distanceCalc(p3[0],p4[0],p3[1],p4[1]))
        length4 = int(100*distanceCalc(p4[0],p1[0],p4[1],p1[1]))
        
        L1 = createLine(p1[0],p2[0],p1[1],p2[1],length1)[0]
        L2 = createLine(p2[0],p3[0],p2[1],p3[1],length2)[0]
        L3 = createLine(p3[0],p4[0],p3[1],p4[1],length3)[0]
        L4 = createLine(p4[0],p1[0],p4[1],p1[1],length4)[0]
        
        coords = L1+L2+L3+L4 
        
    else:
        print("Could not generate point list, shape of obstacle unknown")
    return coords

def createLine(x1,x2,y1,y2,obstLen):
    #Interpolates a line between two points in 2D space
    #Also returns the ortogonal vector from this created line, which is useful for drawing a rectangular box shape
    
    m = (y2-y1) / (x2-x1)
    b = y1 - m*x1
    
    ortom = -1/m
    ang = numpy.arctan2(ortom,1)
    xOffset = numpy.cos(ang)
    yOffset = numpy.sin(ang)
    ortoVector = [xOffset, yOffset]
    
    linePointsX = numpy.linspace(x1,x2,num=obstLen)
    linePointsY = numpy.linspace(y1,y2,num=obstLen)
    linePoints = list(zip(linePointsX,linePointsY))
    
    return linePoints, ortoVector

def polar2Cart(angle, radius):
    x = radius * numpy.cos(numpy.radians(angle)) * numpy.cos(numpy.radians(downTiltAngle))
    y = radius * numpy.sin(numpy.radians(angle)) * numpy.cos(numpy.radians(downTiltAngle))
    return x,y

def distanceCalc(x1,x2,y1,y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
        
def segments(dataPoints):
    # Combines clusters of closely placed detected obstacles
    # I.e. if two obstacles are detected within mergeDistance of each other, they are combined and thereon treated as one single obstacle
    mergeDistance = 0.2
    startAngles = []
    stopAngles = []
    obstacleAnglesList = []
    obstacleRadiusList = []
    killList = []
    #start angle: 55 deg, end at 215
    #finds start and stop points
    for i,distance in enumerate(dataPoints[1:268]):
        if dataPoints[i-1] == 0 and dataPoints[i] != 0: #or dataPoints i-1 to i > mergeDistance:
            startAngles.append(i)
        if dataPoints[i] != 0 and dataPoints[i+1] == 0:
            stopAngles.append(i)
        

    
    # Ensures proper conditions at beginning and end of each scan
    try:    
        if startAngles[0] > stopAngles[0]:
            startAngles.insert(0,0)
        if len(startAngles) > len(stopAngles):
            stopAngles.append(269)
    except IndexError:
        print("No obstacles in sight")
        
    # Merges close obstacles
    for i,stopIndex in enumerate(stopAngles[0:len(stopAngles)-1]):

        x1, y1 = polar2Cart(startAngles[i+1]+45,dataPoints[startAngles[i+1]])
        x2, y2 = polar2Cart(stopAngles[i]+45,dataPoints[stopAngles[i]])
        killList = []
        
        D = distanceCalc(x2,x1,y2,y1)
        
        if D < mergeDistance:
            killList.append(i)
    
    # Deletes the gaps that are too small
    for i in reversed(killList):
        del startAngles[i+1]
        del stopAngles[i]
    
    obstacleAnglesList = list(zip(startAngles,stopAngles))
    for anglePair in obstacleAnglesList:
        if anglePair[0] == anglePair[1]:
            obstacleRadiusList.append(dataPoints[ anglePair[0] ])
        else:
            obstacleRadiusList.append(dataPoints[ anglePair[0]:anglePair[1] ])
    # example: obstacleAnglesList = [ [15, 20], [33,46], [69,89], [125,148] ], where each sublist is one obstacle
    # example: obstacleRadiusList = [ [0.1535, 0.1525, 0.1530, 0.1535, 0.1539], [0.7432, 0.7468, 0.7492, ...], ... ]
    return obstacleAnglesList, obstacleRadiusList

def medianFilter(ranges, prevRanges, prevPrevRanges):
    result = [0]*269
    for i in range(len(ranges)-2):
        current=ranges[i:i+2] + prevRanges[i:i+2] + prevPrevRanges[i:i+2]
        result[i] = numpy.nanmedian(current)
        if result[i] > stairDistance:
            result[i] = 0
            
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
    global plotFigFlag
    fullAngleList = []
    prevPrevRanges = prevRanges
    prevRanges = ranges
    ranges = list(data.ranges)

    filteredRanges = medianFilter(ranges, prevRanges, prevPrevRanges)    
    obstacleAnglesList, obstacleRadiusList = segments(filteredRanges)
    
    for pair in obstacleAnglesList:
        if pair[0] == pair[1]:
            fullAngleList.append( [pair[0]] )
        else:
            fullAngleList.append(  list(range(pair[0], pair[1]))  )

    # Ex: fullAngleList =[ [15, 16, ..., 19,  20], [33, 34, ..., 46], [69, ..., 89], [125, ... , 148] ]
    listOfObstacles = []
    for obstNo,angles in enumerate(fullAngleList):
        obstacleClusters = ObstacleClustering.Obstacle(fullAngleList[obstNo],obstacleRadiusList[obstNo], downTiltAngle)
        listOfObstacles.append(obstacleClusters)
    
    
    # Create huge list of all points
    allObstaclePoints = []
    for i in listOfObstacles:
        allObstaclePoints += generatePointList(i) 
    
    # Convert to point cloud and publish
    cloud = generatePointCloud(allObstaclePoints)
    
    xVals = [ o.x for o in cloud.points ]
    yVals = [ o.y for o in cloud.points ]
 
    vizRanges = filteredRanges
    
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

def listener():
    global pub
    global cloudpub

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)
    pub = rospy.Publisher('scanFiltered', LaserScan, queue_size=4)
    cloudpub = rospy.Publisher('point_cloud', PointCloud, queue_size=4)
    
    rospy.spin()

if __name__ == '__main__':
    listener()


