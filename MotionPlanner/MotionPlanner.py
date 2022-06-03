from math import cos, sin
import numpy as np
import cv2

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
                escapePos = self.findEscapeAngle(path[-1], v_hat)
                if escapePos is not None:
                    X = escapePos
                    path.append(escapePos)
                else:
                    print("Error: cannot find escape at " + str(collidePos))
                    break
            else:
                break

            if np.linalg.norm(X-oldX) < 1:
                print("Error: No progress")
                break
        
        path.append(B)
        return path

    def rotationMatrix(deg):
        theta = np.radians(deg)
        c, s = np.cos(theta), np.sin(theta)
        return np.array(((c, -s), (s, c)))

    def findEscapeAngle(self, pos, heading):
        STEP_ANGLE_DEG = 10 
        R = MotionPlanner.rotationMatrix(STEP_ANGLE_DEG)
        for i in range(1, int(360/STEP_ANGLE_DEG)):
            heading = np.dot(R, heading)
            newPos = pos + heading * MotionPlanner.VEHICLE_SIZE
            if not self.checkPositionCollision(newPos):
                return newPos

        return None

    def checkPositionCollision(self, pos):
        return self.collideMap[round(pos[1]), round(pos[0])] == 0           # cv2 pixel is [y, x]
        
    def drawPathOnMap(self, path):
        temp = path[0]
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
    path = motionPlanner.findPath(np.array([50,50]), np.array([200,200]))
    motionPlanner.drawPathOnMap(path)


    # 300 150 fail

    # rot = MotionPlanner.rotationMatrix(45)
    # x = np.array([1,0])
    # x = np.dot(rot, x)
    # x = np.dot(rot, x)
    # print(x)
    # print(np.dot(rot, np.array([1,0])))
    # print(np.dot(rot, np.dot(rot, np.array([1,0]))))