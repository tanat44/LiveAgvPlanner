from MotionPlanner.MotionPlannerBase import MotionPlannerBase
import numpy as np
from queue import PriorityQueue
import math
import copy

'''PRIMOZ'''

class Candidate(object):
    def __init__(self, pos, path, commands, priority):
        self.priority = priority
        self.path = path
        self.commands = commands
        self.pos = pos
        return

    def __lt__(self, other):
        return self.priority < other.priority

class MotionPlannerP(MotionPlannerBase):

    STEP_SIZE = 3

    def heuristic(self, pos, target, commands):
        return np.linalg.norm(target-pos) + (len(commands) * 1)
    
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

    def findPath(self, A, B):
        seen = dict()

        candidates = PriorityQueue()
        candidates.put(Candidate(A, [], "START:", 0))
        candidate = candidates.get()

        while np.linalg.norm(B-candidate.pos) > MotionPlannerP.STEP_SIZE:
            for (dx, dy, step_name) in [(-MotionPlannerP.STEP_SIZE, 0, "L"), (MotionPlannerP.STEP_SIZE, 0, "R"), (0, -MotionPlannerP.STEP_SIZE, "D"), (0, MotionPlannerP.STEP_SIZE, "U")]:
                delta = np.array([dx, dy])
                newPos = candidate.pos + delta

                posKey = str(newPos[0])+","+str(newPos[1])

                if not self.checkPositionCollision(newPos) and (posKey not in seen.keys()):
                    newPath = copy.deepcopy(candidate.path)
                    newPath.append(newPos)
                    newCommands = candidate.commands+step_name
                    h = self.heuristic(newPos, B, candidate.path)
                    candidates.put(Candidate(newPos, newPath, newCommands, h))
                    seen[posKey] = True

            candidate = candidates.get()

        candidate.path.append(B)
        smoothPath = self.pathSmoothing(candidate.path)
        return smoothPath