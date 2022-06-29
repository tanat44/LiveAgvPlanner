from PIL import Image 
from collections import defaultdict
import math
from queue import PriorityQueue

  
# Function to return a default
# values for keys that is not
# present
def default_wall():
    return 1

class Candidate(object):
    def __init__(self, x, y, path, priority):
        self.priority = priority
        self.path = path
        self.x = x
        self.y = y
        return

    def __lt__(self, other):
        return self.priority < other.priority

def get_map():
    # open method used to open different extension image file
    im = Image.open("./Assets/map.png") 
    px = im.load()

    map = defaultdict(default_wall)
    for x in range(0, im.width):
        for y in range(0, im.height):
            if (255,255,255,255) == px[x,y]:
                map[str(x)+","+str(y)] = 0
    return map
    
def heuristic(x, y, xEnd, yEnd, path):
    return math.dist([x,y], [xEnd, yEnd]) + len(path) * 1

def findPath(xStart, yStart, xEnd, yEnd, map):
    candidates = PriorityQueue()
    candidates.put(Candidate(xStart, yStart, "START:", 0))
    candidate = candidates.get()
    while candidate.y != yEnd or candidate.x != xEnd:     
        for (dx, dy, step_name) in [(-1, 0, "L"), (1, 0, "R"), (0, -1, "D"), (0, 1, "U")]:
            if map[str(candidate.x+dx)+","+str(candidate.y+dy)] == 0:
                candidates.put(Candidate(candidate.x+dx, candidate.y+dy, candidate.path+step_name, heuristic(candidate.x+dx, candidate.y+dy, xEnd, yEnd, candidate.path)))
        candidate = candidates.get()
    return candidate.path

if __name__=="__main__":
    print(findPath(100,100,109,105,get_map()))
