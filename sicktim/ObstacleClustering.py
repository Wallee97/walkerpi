from ssTest3 import polar2Cart
from ssTest3 import distanceCalc
from ssTest3 import createLine
import numpy
import matplotlib.pyplot as plt

class Obstacle:
    
    def __init__(self, angles, obstacleRadius,tilt):
        #
        #make sure segmentation happens before this
        #convert to cartesian
        #find width and length of that object using obstacleRadius[0:end]
        #determine shape of that object using width and length
        
        plt.axis([-1,1,-1,1])
        
        
        if type(obstacleRadius) == numpy.float64:
            obstacleRadius = [obstacleRadius]
        
        obstacleCart = []
        
        if len(obstacleRadius) == 0:
            x,y = polar2Cart(angles[0]+45,distance)
            obstacleCart.append([x,y])
            print("List is length 0!!")
        elif len(obstacleRadius) >= 1:
            for i,distance in enumerate(obstacleRadius):
                x,y = polar2Cart(angles[i]+45,distance)
                obstacleCart.append([x,y])
        else:
            print("Det sket sig")
        #print(angles)
        #print()
        #print(len(obstacleCart))
        #print()
        #S = distanceCalc( cartCoord[0][0], cartCoord[-1][0], cartCoord[0][1], cartCoord[-1][1])
        
        
        if len(obstacleCart) <= 5:
            
            x0 = (obstacleCart[0][0] + obstacleCart[-1][0]) / 2
            y0 = (obstacleCart[0][1] + obstacleCart[-1][1]) / 2
            r = []
            for pair in obstacleCart:
                r.append( distanceCalc(pair[0],x0,pair[1],y0)  )
            rmax = max(r)
            
            self.x0 = x0
            self.y0 = y0
            self.r = rmax
            self.shape = "circle"
        elif len(obstacleCart) > 5:
            x1,y1 = obstacleCart[0]
            x2,y2 = obstacleCart[-1]
            
            linePoints, ortoVector = createLine(x1,x2,y1,y2,len(obstacleCart))
            S = distanceCalc(x1,x2,y1,y2) 
            D = []
            #[(x,y)(x,y)(x,y)()()]
            for index,pair in enumerate(linePoints):
                D.append( distanceCalc(pair[0],obstacleCart[index][0],pair[1],obstacleCart[index][1]) )
            Dmax = max(D)
            
            # Determine rectangle or line
            if Dmax < 0.2*S:
                self.p1 = [x1,y1]
                self.p2 = [x2,y2]
                self.shape = "line"
            
            elif Dmax >= 0.2*S:
                self.p1 = [x1 + ortoVector[0]*Dmax, y1 + ortoVector[1]*Dmax]
                self.p2 = [x1 - ortoVector[0]*Dmax, y1 - ortoVector[1]*Dmax]
                self.p3 = [x2 + ortoVector[0]*Dmax, y2 + ortoVector[1]*Dmax]
                self.p4 = [x2 - ortoVector[0]*Dmax, y2 - ortoVector[1]*Dmax]
                #Use ortoVector to offset endpoints of linePoints by +-D
                self.shape = "rectangle"
                
                #plt.scatter(self.p1[0],self.p1[1])
                #plt.scatter(self.p2[0],self.p2[1])
                #plt.scatter(self.p3[0],self.p3[1])
                #plt.scatter(self.p4[0],self.p4[1])
                
                
            
            else:
                self.shape = "unknown"
                print("Formen sket sig")
                
        else:
            pass
        
        print(self.shape)
        
        #plt.show()
        
        
        #self.shape = shape
        #self.p1 = p1
        #self.p2 = p2
        #self.p3 = p3
        #self.p4 = p4
       
        
    def clustering():
        pass
    
    def figure():
        pass
    
    