from MotionPlanner.MotionPlannerBase import MotionPlannerBase
import numpy as np
import copy

class MotionPlannerT(MotionPlannerBase):
    
    def findPath(self, A, B):
        path = [A]
        self.findPathRecursive(A, B, path)
        path = self.pathSmoothing(path)
        return path
    
    def findPathRecursive(self, A, B, path, direction = 1):   
        if np.linalg.norm(B-A) < 1:
            return 0 # total distance

        if len(path) > 3 and np.linalg.norm(path[-3]-A) < 1:     # running into loop
            print("loop detected")
            return float('inf') 
            
        v = B - A
        d = np.linalg.norm(v)
        v_hat = v / d
        stepSize = d if d < MotionPlannerBase.VEHICLE_SIZE else MotionPlannerBase.VEHICLE_SIZE
        pos = A + v_hat * stepSize

        if self.checkPositionCollision(pos):
            v0 = path[-1] - path[-2]
            v0_hat = v0 / np.linalg.norm(v0)

            DOT_THRES = -0.98

            pos1 = self.findEscapeAngle(A, v_hat, direction)
            distance1 = float('inf')
            path1 = []
            v1 = pos1 - A
            v1_hat = v1 / np.linalg.norm(v1)
            if np.dot(v0_hat, v1_hat) > DOT_THRES:
                path1 = copy.deepcopy(path)
                stepSize = np.linalg.norm(v1)
                path1.append(pos1)
                distance1 = self.findPathRecursive(pos1, B, path1, direction) + stepSize

            pos2 = self.findEscapeAngle(A, v_hat, direction*-1)
            distance2 = float('inf')
            path2 = []
            v2 = pos2 - A
            v2_hat = v2 / np.linalg.norm(v2)
            if np.dot(v0_hat, v2_hat) > DOT_THRES:
                path2 = copy.deepcopy(path)
                stepSize = np.linalg.norm(v2)
                path2.append(pos2)
                distance2 = self.findPathRecursive(pos2, B, path2, direction) + stepSize

            del path[:]
            if distance1 < distance2:
                for x in path1:
                    path.append(x)
                return distance1
            else:
                for x in path2:
                    path.append(x)
                return distance2

        path.append(pos)
        totalDistance = self.findPathRecursive(pos, B, path, direction) + stepSize
        
        return totalDistance

    def rotationMatrix(deg):
        theta = np.radians(deg)
        c, s = np.cos(theta), np.sin(theta)
        return np.array(((c, -s), (s, c)))

    def findEscapeAngle(self, pos, heading, direction = 1):
        STEP_ANGLE_DEG = 10 
        R = MotionPlannerT.rotationMatrix(STEP_ANGLE_DEG * direction)
        for i in range(1, int(360/STEP_ANGLE_DEG)):
            heading = np.dot(R, heading)
            newPos = pos + heading * MotionPlannerBase.VEHICLE_SIZE
            if not self.checkPositionCollision(newPos):
                return newPos

        return None
    
    def checkLineCollision(self, A, B):
        v = B-A
        d = np.linalg.norm(v)
        v_hat = v / d
        STEP = MotionPlannerBase.VEHICLE_SIZE / 4.0
        for i in range(0, int(d / STEP )):
            pos = A + v_hat*i*STEP
            if self.checkPositionCollision(pos):
                return True
        return False
    
    def pairWiseSmoothing(self, path, startIdx=0):
        for i in range(startIdx, len(path)):
            X = path[i]
            YIdx = i+1
            for j in range(i+1, len(path)):
                Y = path[j]
                if self.checkLineCollision(X, Y):
                    continue
                YIdx = j
            if YIdx > i+1:
                return path[:i+1] + path[YIdx:]
        return path

    def pathSmoothing(self, path):
        startIdx = 0
        while startIdx < len(path)-1:
            path = self.pairWiseSmoothing(path, startIdx)
            startIdx += 1

        return path



if __name__=="__main__":
    motionPlanner = MotionPlannerT("../Assets/map.png")

    A = np.array([50,50])
    B = np.array([400,300])

    # path = motionPlanner.findPath(np.array([50,50]), np.array([400,300]))
    # path2 = motionPlanner.pathSmoothing(path)
    # motionPlanner.drawPathOnMap(path2)

    path = [A]
    distance = motionPlanner.findPathRecursive(A, B, path, 1)
    print("distance", distance)
    # path = motionPlanner.pathSmoothing(path)
    motionPlanner.drawPathOnMap(path)

    # hard case
    # 300 150
    # 400 300