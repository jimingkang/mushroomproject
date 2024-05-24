import math


class Mushroom:
    def __init__(self,dist,trace_id,x,y,z=0):
        self.dist=dist
        #self.score=score
        self.trace_id=trace_id
        self.x=x
        self.y=y
        self.z=z
    def distance(self):
        return math.sqrt(self.x*self.x+self.y*self.y)