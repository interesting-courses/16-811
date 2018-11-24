import numpy as np

class Stack:
    def __init__(self):
        self.items = []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        return self.items.pop()

    def is_empty(self):
        return (self.items == [])

    def top(self):
        return self.items[-1]

class Point:
    def __init__(self, x, y, polygonid = None):
        self.x = x
        self.y = y
        self.polygonId = polygonid

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getPolygonId(self):
        return self.polygonId

    def setX(self, x):
        self.x = x

    def setY(self, y):
        self.y = y

    def setPolygonId(self, polygonid):
        self.polygonId = polygonid