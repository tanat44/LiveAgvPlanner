
import numpy as np
import cv2
import sys

sys.setrecursionlimit(10000)

class MotionPlannerBase:
    VEHICLE_SIZE = 30

    def __init__(self, mapFile):
        img = cv2.imread(mapFile)
        grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, self.map) = cv2.threshold(grayImage, 200, 255, cv2.THRESH_BINARY)

        # create collideMap for vehicle collision (circular bounding box)
        # 0 isn't drivable
        vehicleKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(MotionPlannerBase.VEHICLE_SIZE, MotionPlannerBase.VEHICLE_SIZE))
        self.collideMap = cv2.erode(self.map, vehicleKernel, iterations=1)
        cv2.imwrite("collideMap.png", self.collideMap) 
        
    def checkPositionCollision(self, pos):
        return self.collideMap[round(pos[1]), round(pos[0])] == 0           # cv2 pixel is [y, x]
        
    def drawPathOnMap(self, path):
        temp = path[0].astype(np.int32)
        out = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)
        lineColor = (205,255,143)
        dotColor = (245,87,66)
        for pos in path:
            pos = pos.astype(np.int32)
            cv2.line(out, list(temp), list(pos), lineColor, 1)
            cv2.circle(out, pos, 3, dotColor, -1)
            temp = pos
        cv2.imwrite("out.png", out) 
        return out

    def findPath(self, A, B):
        '''OVERRIDE THIS METHOD'''
        pass