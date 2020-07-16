from ssTest3 import polar2Cart
from ssTest3 import distanceCalc

class Obstacle:
    
    def __init__(self, obstaclePoints[],angles[],tilt=0"""shape,p1=0,p2=0,p3=0,p4=0"""):
        #
        #make sure segmentation happens before this
        convert to cartesian
        find width and length of that object using obstaclePoints[0:end]
        determine shape of that object using width and length
        
        cartCoords = []
        for i,distance in enumerate(obstaclePoints[]):
            x,y = polar2Cart(angles[0]+45+i,distance,tilt)
            cartCoords.append([x,y])
        
        S = distanceCalc( cartCoord[0][0], cartCoord[-1][0], cartCoord[0][1], cartCoord[-1][1])
        
        
        if len(obstaclePoints) < 5:
            self.shape = "circle"
        elif 
        
        
        self.shape = shape
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        
        
        
    def clustering():
        pass
    
    def figure():
        pass
    
    