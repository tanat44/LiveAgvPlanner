from math import cos, sin
import numpy as np
import cv2
import sys

class MotionPlanner:
    VEHICLE_SIZE = 30

    def __init__(self, mapFile):
        img = cv2.imread(mapFile)
        grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, self.map) = cv2.threshold(grayImage, 200, 255, cv2.THRESH_BINARY)

        # for vehicle collision (circular bounding box)
        # 0 isn't drivable
        vehicleKernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(MotionPlanner.VEHICLE_SIZE, MotionPlanner.VEHICLE_SIZE))
        self.collideMap = cv2.erode(self.map, vehicleKernel, iterations=1)
        cv2.imwrite("collideMap.png", self.collideMap) 
        

    def findPath(self, A, B):
        path = []
        path.append(A)
        X = A
        oldX = None
        STEP_SIZE = MotionPlanner.VEHICLE_SIZE
        escapeDirection = 1

        while True:
            v = B - X
            d = np.linalg.norm(v)
            v_hat = v / d
            collidePos = None

            oldX = X

            for i in range(1, int(d / STEP_SIZE)):
                pos = X + v_hat * i * STEP_SIZE
                if self.checkPositionCollision(pos):
                    collidePos = pos
                    break
                else:
                    path.append(pos)
            
            if collidePos is not None:
                escapePos = self.findEscapeAngle(path[-1], v_hat, escapeDirection)
                if escapePos is not None:
                    X = escapePos
                    path.append(escapePos)
                else:
                    print("Error: cannot find escape at " + str(collidePos))
                    break
            else:
                break

            if np.linalg.norm(X-oldX) < 1:
                print("Changing escape direction")
                escapeDirection *= -1
                # break
        
        path.append(B)
        return path

    count = 0

    def findPathRecursive(self, A, B, path, direction = 1):   
        MotionPlanner.count += 1     

        if MotionPlanner.count == 50:
            print("MAX stack reached")
            return 0

        if np.linalg.norm(B-A) < 1:
            return 0 # total distance

        if len(path) > 3 and np.linalg.norm(path[-3]-A) < 1:     # running into loop
            print("loop detected", direction, MotionPlanner.count)
            removeDistance = np.linalg.norm(path[-1]-path[-2])
            del path[-3:-1]
            return self.findPathRecursive(path[-1], B, path, direction*-1) - 2*removeDistance  
            
        v = B - A
        d = np.linalg.norm(v)
        v_hat = v / d
        stepSize = d if d < MotionPlanner.VEHICLE_SIZE else MotionPlanner.VEHICLE_SIZE
        pos = A + v_hat * stepSize

        if self.checkPositionCollision(pos):
            cwPos = self.findEscapeAngle(A, v_hat, direction)
            if cwPos is not None:
                stepSize = np.linalg.norm(cwPos - A)
                path.append(cwPos)
                return self.findPathRecursive(cwPos, B, path, direction) + stepSize
            else:
                print("No solution, aborting")
                return 0

        path.append(pos)
        totalDistance = self.findPathRecursive(pos, B, path, direction) + stepSize
        
        return totalDistance

    def rotationMatrix(deg):
        theta = np.radians(deg)
        c, s = np.cos(theta), np.sin(theta)
        return np.array(((c, -s), (s, c)))

    def findEscapeAngle(self, pos, heading, direction = 1):
        STEP_ANGLE_DEG = 10 
        R = MotionPlanner.rotationMatrix(STEP_ANGLE_DEG * direction)
        for i in range(1, int(360/STEP_ANGLE_DEG)):
            heading = np.dot(R, heading)
            newPos = pos + heading * MotionPlanner.VEHICLE_SIZE
            if not self.checkPositionCollision(newPos):
                return newPos

        return None

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


if __name__=="__main__":
    motionPlanner = MotionPlanner("../Assets/map.png")
    # path = motionPlanner.findPath(np.array([50,50]), np.array([400,300]))
    path = []
    distance = motionPlanner.findPathRecursive(np.array([50,50]), np.array([400,300]), path, 1)
    print("distance=", distance)
    motionPlanner.drawPathOnMap(path)


    # hard case
    # 300 150
    # 400 300