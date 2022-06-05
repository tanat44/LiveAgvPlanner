class Grid:
    def __init__(self):
        self.nodes = []

    def getNodePos(self):
        pos = []
        for x in self.nodes:
            pos.append(x.pos)
        return pos

class GridNode:
    def __init__(self, pos):
        self.up = None
        self.down = None
        self.left = None
        self.right = None
        self.visited = False
        self.pos = pos