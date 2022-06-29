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

    STEP_SIZE = MotionPlannerBase.VEHICLE_SIZE

    def heuristic(self, pos, target, commands):
        return np.linalg.norm(target-pos) + len(commands) * MotionPlannerP.STEP_SIZE
    
    def findPath(self, A, B):
        candidates = PriorityQueue()
        candidates.put(Candidate(A, [], "START:", 0))
        candidate = candidates.get()

        while np.linalg.norm(B-candidate.pos) > MotionPlannerP.STEP_SIZE:
            for (dx, dy, step_name) in [(-MotionPlannerP.STEP_SIZE, 0, "L"), (MotionPlannerP.STEP_SIZE, 0, "R"), (0, -MotionPlannerP.STEP_SIZE, "D"), (0, MotionPlannerP.STEP_SIZE, "U")]:
                delta = np.array([dx, dy])
                newPos = candidate.pos + delta

                if not self.checkPositionCollision(newPos):
                    newPath = copy.deepcopy(candidate.path)
                    newPath.append(newPos)
                    newCommands = candidate.commands+step_name
                    h = self.heuristic(newPos, B, candidate.path)
                    candidates.put(Candidate(newPos, newPath, newCommands, h))

            candidate = candidates.get()

        candidate.path.append(B)
        return candidate.path