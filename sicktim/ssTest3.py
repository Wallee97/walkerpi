#!/usr/bin/env python

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

# Create a DAC instance.
i2c = busio.I2C(board.SCL,board.SDA)
#dac1 = adafruit_mcp4725.MCP4725(i2c, address=0x60)
#dac2 = adafruit_mcp4725.MCP4725(i2c, address=0x61)

brakeDistance = 0.35
stairDistance = 0.6
downTiltAngle = 0
breakVoltage = 1500
ranges = list()
prevRanges = [0,0,0]
prevPrevRanges = [0,0,0]
#plotFigFlag = False


"""
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    #temp_c = round(tmp102.read_temp(), 2)

    # Add x and y to lists
    #xs.append(xs)
    #ys.append(ys)

    # Limit x and y lists to 20 items
    xs = xs[-1000:]
    ys = ys[-1000:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')"""

def generatePointCloud(obstacleCart):
    global cloud
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "laser"
    numberOfPoints = len(obstacleCart)
    cloud = PointCloud()
    cloud.header = h
    #cloud.points = numberOfPoints
    point = []
    #p = Point32()
    for i in range(0,numberOfPoints):
        #print(obstacleCart[i])
        
        #p.x = obstacleCart[i][0]
        #p.y = obstacleCart[i][1]
        #p.z = 0
        
        #print(p.x)
        
        
        
        cloud.points.append(Point32( obstacleCart[i][0], obstacleCart[i][1], 0 ))
    
    print()
    #cloudpub.publish(cloud)
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
    #Creates a line according to y=mx+b
    m = (y2-y1) / (x2-x1)
    b = y1 - m*x1
    
    ortom = -1/m
    ang = numpy.arctan2(ortom,1)
    xOffset = numpy.cos(ang)
    yOffset = numpy.sin(ang)
    ortoVector = [xOffset, yOffset]
    
    #c*y1 = m*x1 + b -> m*x1 - c*y1 = b -> normal vector = (m,-y1) -> unit vector = sqrt(m^2 + 1^2) * (m,1) Offset = D*unit vector (x,y)
    
    
    linePointsX = numpy.linspace(x1,x2,num=obstLen)
    linePointsY = numpy.linspace(y1,y2,num=obstLen)
    linePoints = list(zip(linePointsX,linePointsY))
    
    return linePoints, ortoVector

def polar2Cart(angle, radius):
    x = radius * numpy.cos(numpy.radians(angle)) * numpy.cos(numpy.radians(downTiltAngle))
    y = radius * numpy.sin(numpy.radians(angle)) * numpy.cos(numpy.radians(downTiltAngle))
    return x,y #return -y because 0 degrees points backwards from sensor

def distanceCalc(x1,x2,y1,y2):
    return math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
        
def segments(dataPoints):
    mergeDistance = 0.2
    startAngles = []
    stopAngles = []
    obstacleAnglesList = []
    obstacleRadiusList = []
    killList = []
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
    
    # Deletes gaps that are too small
    for i in reversed(killList):
        del startAngles[i+1]
        del stopAngles[i]
    
    obstacleAnglesList = list(zip(startAngles,stopAngles))
    for anglePair in obstacleAnglesList:
        if anglePair[0] == anglePair[1]:
            obstacleRadiusList.append(dataPoints[ anglePair[0] ])
        else:
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
    #print(fullAngleList)
    
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
    
    print("x = " + str(xVals) + ", y = " + str(yVals))
    print()
    
    
    """
    plt.axis([ -2, 2, -2, 2 ])
    plt.scatter(xVals, yVals)
    plt.show()
    plt.pause(0.05)
    """
    """
    if plotFigFlag == False:
        plt.axis([ -2, 2, -2, 2 ])
        plt.scatter(xVals, yVals)
        plt.show()
        plotFigFlag = True
        plt.pause(0.05)
    else:
        
        plt.clf()
        plt.axis([ -2, 2, -2, 2 ])
        plt.scatter(xVals, yVals)
        plt.pause(0.05)
        print("second statement")
        #plt.draw()
        """
    #ani = animation.FuncAnimation(fig, animate, fargs=(xVals, yVals), interval=1000)
    #plt.show()
    
    
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
    global cloudpub
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True) #listener

    rospy.Subscriber("scan", LaserScan, callback)
    pub = rospy.Publisher('scanFiltered', LaserScan, queue_size=4)
    cloudpub = rospy.Publisher('point_cloud', PointCloud, queue_size=4)
    
    
    
    
    
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


