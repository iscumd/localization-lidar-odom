
class Obstacle:
    def __init__(self, x, y, distance, heading):
        self.x = x
        self.y = y
        self.distance_to_robot = distance
        self.heading_from_robot = heading

class Map:
    def __init__(self):
        self.obstacles = []

class Point:
    def __init__(self,x,y):
        self.x = x
        self.y = y