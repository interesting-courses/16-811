# Computational Geometry Assignment

## Question 1 - Convex Hull

Define some points and pass it as an numpy array to the function findConvexHull()

```
    points = np.random.rand(500, 2)
    c_hull = findConvexHull(points)
```

## Question 2 - Find Shortest Path for Point Robot with Obstacles in Field

Define a list of lists as the obstacle points. Define the start and target vertices and then call the shortest path function from the VisibilityGraph class.

```
points = [[[0.0, 1.0], [1.5, 4.0], [1.0, 6.0]], [[4.0, 4.0], [7.0, 4.0], [5.5, 8.0]]]
start = np.array([0, 0])
end = np.array([8, 8])
vg = VisibilityGraph(points, start, end)
vg.findShortestPath()
print (vg.shorttestPath)
# plots the obstacles and the path
vg.plotPolygonsAndPaths()
```

## Question 3 - Find Shortest Path for Convex Polygonal Robot with Obstacles in Field

Define a list of lists as the obstacle points. Define the start, target vertices and then call the VisibilityGraph class. Before calling the shortestpath function, define the robot and call the getMinkowskiSum() function. For plotting, pass the robot and the isRobot parameter as True

```
points = [[[0.7, 4.06], [0.6, 2.01], [2.42, 2.95]], [[4.5, 2.59], [5.76, 1.95], [5.14, 3.81], [6.4, 3.17]], [[4.98, 4.87], [6.0, 5.0], [6.4, 5.95], [5.77, 6.77], [4.75, 6.64], [4.36, 5.69]]]
start = np.array([0, 0])
end = np.array([8, 8])
robot = np.array([[-1,-1],[-1,1],[0,1],[1,1],[1,-1]], np.float32)
vg = VisibiltyGraph(points, start, end)
# call function for configuration space
vg.getMinkowskiSum(robot)
vg.findShortestPath()
print (vg.shortestPath)
vg.plotPolygonsAndPaths(robot, isRobot=True)
```