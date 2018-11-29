import sys
import itertools
import numpy as np
from q1 import findConvexHull
from matplotlib import pyplot as plt

class VisibiltyGraph:
    def __init__(self, points, start, end):
        self.polygons = []
        self.original_polygons = []

        # Assign the start and end for the robot to move
        self.start = start
        self.end = end
        self.shortestPath = None
        # Form the initial set of points
        self.formPolygonsFromPoints(points)
        # for plotting
        self.original_polygons = self.polygons
        # Fire functions for computing checks and visibility graph computation
        self.checkAndCompute()
    
    def formPolygonsFromPoints(self, points):
        # form convex polygons and assign them indices
        for i, set_points in enumerate(points):
            set_points = np.array(set_points, np.float32)
            chull = findConvexHull(set_points.copy())
            poly_elem = {"convex_hull": chull, "points": set_points, 'index': i}
            self.polygons.append(poly_elem)

    def checkAndCompute(self):
        # ensure valid start and end
        self.checkForBaseConditions()
        # check for overlapping polygons
        isOverlap = self.checkOverlappingPolygons()
        if isOverlap:
            self.recreateOverlappingPolygons()

        # compute visibility graph
        self.formVisibiltyGraph()

    def findSetOfTwoPoints(self, points):
        # return 2 points in order starting from index 0
        indices = [i for i in range(0, points.shape[0])] + [0]
        for i in range(0, len(indices)-1):
            val = indices[i:i+2]
            if len(val) == 2:
                yield val

    def isPointInGivenPolygon(self, point, polygon):
        # check if point if inside given polygon
        x, y = point
        inside = False
        hull_points = polygon['convex_hull']

        n = len(hull_points)
        p1x,p1y = hull_points[0][0], hull_points[0][1]
        for i in range(0, n+1):
            p2x,p2y = hull_points[i % n][0], hull_points[i%n][1]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside

    def arePolygonsIntersecting(self, poly1, poly2):
        # check if given two polys intersect
        for point in poly1['convex_hull']:
            if self.isPointInGivenPolygon(point, poly2):
                return True
        return False

    def checkOverlappingPolygons(self):
        # check if any two polys are overlapping
        for (poly1, poly2) in itertools.combinations(self.polygons, 2):
            if self.arePolygonsIntersecting(poly1, poly2):
                return True
        return False

    def checkForBaseConditions(self):
        for polygon in self.polygons:
            # is start not in polygon
            if not self.isPointInGivenPolygon(self.start, polygon):
                # is end not in polygon
                if not self.isPointInGivenPolygon(self.end, polygon):
                    continue
                else:
                    print ('No Valid Path exists!')
                    sys.exit(1)
            else:
                print ('No Valid Path exists!')
                sys.exit(1)

    def recreateOverlappingPolygons(self):
        new_polygons = []

        intersecting_polys = {self.polygons[i]['index']:set() for i in range(0, len(self.polygons))}
        for (poly1, poly2) in itertools.combinations(self.polygons, 2):
            if self.arePolygonsIntersecting(poly1, poly2):
                intersecting_polys[poly1['index']].add(poly2['index'])
                intersecting_polys[poly2['index']].add(poly1['index'])

        for k, v in intersecting_polys.items():
            if len(v) == 0:
                new_polygons.append(self.polygons[k])
            else:
                _point = self.fetchHullPoints(k)
                new_index = [k]
                for _v in list(v):
                    _point = np.vstack((_point, self.fetchHullPoints(_v)))
                    new_index.append(_v)

                new_index.sort(reverse=True)
                new_index = [str(a) for a in new_index]
                new_index = int(''.join(new_index))
                hull = findConvexHull(_point)
                new_polygons.append({"convex_hull":hull, "points":_point, 'index': new_index})

        self.polygons = []
        q_dict = {}
        for pol in new_polygons:
            if pol['index'] in q_dict:
                continue
            q_dict[pol['index']]=1
            self.polygons.append(pol)

        overlap = self.checkOverlappingPolygons()
        if overlap:
            self.formVisibiltyGraph()
            self.recreateOverlappingPolygons()

    def fetchHullPoints(self, index):
        for i in self.polygons:
            if i['index'] == index:
                return i['convex_hull']

    def areConnectedNeighbors(self, v1, v2, e1, e2):
        # check if the vertices are connected to the neighboring edges
        if (v2==e2).all() and not (v1==e1).all():
            return True
        if (v2==e1).all() and not (v1==e2).all():
            return True
        if (v1==e2).all() and not (v2==e2).all():
            return True
        if (v1==e1).all() and not (v2==e2).all():
            return True
        return False

    def getPowerofPoint(self, line, point):
        # return the power of point wrt line y=mx+c format
        e1, e2 = line[0], line[1]
        m = e1[1] - e2[1]
        if  (e1[0]-e2[0]) == 0:
            return point[1]-e1[0]

        m /= e1[0]-e2[0]
        c = e1[1] - m*e1[0]
        side = m*point[0] - point[1] + c
        side = side/np.sqrt(1 + m**2)
        if np.abs(side) < 1e-4:
            side = 0
        sign = np.sign(side)
        return sign

    def findPolygonalConnectionsinVisibiltyGraph(self):
        # add all possible missing vertices in visiblilty graph for exising items
        for k, _ in self.visibility_graph.items():
            temp_k = tuple(k)
            polygon_id = int(self.vertex_map[temp_k])
            try:
                poly_pts = self.polygons[polygon_id]['convex_hull']
            except Exception:
                print ('No valid path exist as no visibility vertex is found!')
                sys.exit(1)
            lngth = poly_pts.shape[0]
            
            for i in range(0, poly_pts.shape[0]):
                if (poly_pts[i] == k).all():
                    self.visibility_graph[temp_k].add(tuple(poly_pts[(i-1)%lngth]))
                    self.visibility_graph[temp_k].add(tuple(poly_pts[(i+1)%lngth]))
                    break

    def formVisibiltyGraph(self):
        if len(self.polygons) == 0:
            print ('No obstacles. Shortest path is a straight line from {0} to {1}'.format(self.start, self.end))
            sys.exit(1)
        
        # load the points per poly
        temp_graph = []
        for polynum, poly in enumerate(self.polygons):
            for point in poly['convex_hull']:
                index = np.array([polynum])
                a = np.hstack((point, index))
                temp_graph.append(a)
        
        # add the source and end vertex as they dont exist in any poly
        temp_graph.append(np.array([self.start[0], self.start[1], -1]))
        temp_graph.append(np.array([self.end[0], self.end[1], -2]))
        temp_graph = np.array(temp_graph)

        # create a map of vertices
        self.vertex_map = {}
        counter = 0
        for _ in self.polygons:
            for vertex in temp_graph:
                self.vertex_map[tuple(vertex[:-1])] = vertex[-1]
                counter += 1
        
        self.inverse_vertex_map = {v:k for k, v in self.vertex_map.items()}

        visibility_graph = {}
        for g in temp_graph:
            # for each vertex, add a possible visible graph
            visibility_graph[tuple(g[:-1])] = set()

        # compute visible vertices from graph
        edges = []
        for poly in self.polygons:
            for _, (i1, i2) in enumerate(self.findSetOfTwoPoints(poly['convex_hull'])):
                p1, p2 = poly['convex_hull'][i1], poly['convex_hull'][i2]
                # dont add the same point to the edge
                if (p1 == p2).all():
                    continue
                edges.append((p1, p2))

        edges = np.array(edges)
        # for each vertex in the graph
        for _, vertex1 in enumerate(temp_graph):
            id1 = vertex1[-1] 
            vertex1 = vertex1[:-1]
            for _, vertex2 in enumerate(temp_graph):
                id2 = vertex2[-1] 
                vertex2 = vertex2[:-1]
                if id1 == id2:
                    break
                if (vertex1 == vertex2).all():
                    continue

                for _ , (edge1, edge2) in enumerate(edges):
                    # ensure no same edges are computed or no connected neighbors are there
                    if (edge1 == edge2).all():
                        continue
                    if (edge1 == vertex1).all() and (edge2 == vertex2).all() or (edge1 == vertex2).all() and (edge2 == vertex1).all(): 
                        continue
                    if self.areConnectedNeighbors(vertex1, vertex2, edge1, edge2):
                        continue

                    x1, y1 = vertex1
                    x2, y2 = vertex2
                    x3, y3 = edge1
                    x4, y4 = edge2
                    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
                    if denom != 0:
                        px = ((x1*y2 - y1*x2)*(x3-x4) -  (x1-x2)*(x3*y4 - y3*x4)) / denom
                        py = ((x1*y2 - y1*x2)*(y3-y4) -  (y1-y2)*(x3*y4 - y3*x4)) / denom
                        list_vertex = [vertex1,vertex2]
                        if self.getPowerofPoint(list_vertex, edge1) == self.getPowerofPoint(list_vertex, edge2):
                            continue
                        if px >= min(vertex1[0], vertex2[0]) and px <= max(vertex1[0], vertex2[0]) and py >= min(vertex1[1],vertex2[1]) and py <= max(vertex1[1],vertex2[1]):
                            # valid vertices not gotten
                            pass
                        else:
                            continue
                        break
                else:
                    visibility_graph[tuple(vertex1)].add(tuple(vertex2))
                    visibility_graph[tuple(vertex2)].add(tuple(vertex1))

        # assign visibility graph
        self.visibility_graph = visibility_graph
        self.findPolygonalConnectionsinVisibiltyGraph()

    def createAdjacencyMatrix(self):
        # from the visibility graph get valid graph points
        keys = list(self.visibility_graph.keys())
        
        adjMatrix = np.full(shape=(len(keys), len(keys)), fill_value=np.inf)
        for k, v in self.visibility_graph.items():
            # form the weighted graph by adding edges equal to euclidian distance
            key_index = keys.index(k)
            for vertices in v:
                v_index = keys.index(vertices)
                weight = np.linalg.norm(np.array(k) - np.array(vertices))
                adjMatrix[int(key_index), int(v_index)] = weight
                adjMatrix[int(v_index), int(key_index)] = weight
        
        # set the keys
        self.keys = keys
        # set the index of the start and end
        source_index = keys.index((self.start[0], self.start[1]))
        target_index = keys.index((self.end[0], self.end[1]))
        # ensure infinte weights for now - unknown shortest distance
        adjMatrix[source_index, target_index] = np.inf
        adjMatrix[target_index, source_index] = np.inf
        
        return adjMatrix, source_index, target_index

    def findShortestPath(self):
        # compute adjacency matrix
        final_matrix, source, target = self.createAdjacencyMatrix()
        
        num_vertices = len(self.keys)
        dist = [np.inf for i in range(0, num_vertices)]
        prev = [None for i  in range(0, num_vertices)]

        dist[source] = 0
        Q = [i for i in range(0, num_vertices)]

        # find min element and index - implementation from CLRS
        def findMininQueue(Q, dist):
            min_dist = np.inf
            min_index = None
            for q in Q:
                d = dist[q]
                if d < min_dist:
                    min_dist = d
                    min_index = q
            return min_index

        # djikstra computation of shortest path - implementation from CLRS
        visited = set()
        while len(Q):
            u = findMininQueue(Q, dist)
            visited.add(u)
            Q.remove(u)
            neighbors = np.where(final_matrix[u] != np.inf)[0]
            for v in neighbors:
                if v not in visited:
                    alt = dist[u] + final_matrix[u, v]
                    if alt < dist[v]:
                        dist[v] = alt
                        prev[v] = u

        # reform the path from target to source and reverse it
        u = target
        path = [u]
        while u != source:
            u = prev[u]
            path.append(u)
        path = path[::-1]

        # find actual path using keys
        actual_path = [self.keys[i] for i in path]
        self.shortestPath = actual_path

    def getMinkowskiSum(self, robot):
        # form the convex robot and take its inverse
        crobot = findConvexHull(robot)
        robot = -1*crobot

        # for the configuration space
        curr_polys = []
        for polygon in self.polygons:
            curr_poly = []
            for poly_pt in polygon['convex_hull']:
                for robot_pt in robot:
                    curr_poly.append(poly_pt + robot_pt)
            curr_polys.append(curr_poly)

        # form all the new polygons here
        final_polys = []
        for curr_index, poly_points in enumerate(curr_polys):
            chull = findConvexHull(poly_points)
            poly_elem = {"convex_hull": chull, "points": poly_points, 'index': curr_index}
            final_polys.append(poly_elem)
        
        # reassign the polygons
        self.polygons = final_polys
        # fire vg computation and base checks
        self.checkAndCompute()

    def plotPolygonsAndPaths(self, robot = None, isRobot=False):        
        for i, poly in enumerate(self.original_polygons):
            poly_hull = poly['convex_hull']
            hull_x = poly_hull[:,0]
            hull_y = poly_hull[:,1]
            
            # actually plot the polygon
            hull_x = np.append(hull_x, hull_x[0])
            hull_y = np.append(hull_y, hull_y[0])
            plt.plot(hull_x, hull_y, "b-")

        if self.shortestPath:
            spath_x = [np.float(i[0]) for i in self.shortestPath]
            spath_y = [np.float(i[1]) for i in self.shortestPath]

            for i in range(0, len(self.shortestPath)-1):
                plt.plot([spath_x[i], spath_x[i+1]], [spath_y[i], spath_y[i+1]], 'k-')
                
        plt.plot(self.start[0], self.start[1], 'ro')
        plt.plot(self.end[0], self.end[1], 'go')

        if isRobot:
            if robot is None:
                print ('Pass the robot for plotting')
                sys.exit(1)
            else:
                plot_robo = np.array(robot)
                hull_x = plot_robo[:,0]
                hull_y = plot_robo[:,1]
                hull_x = np.append(hull_x, hull_x[0])
                hull_y = np.append(hull_y, hull_y[0])
                plt.plot(hull_x, hull_y, "g-")

        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # overlapping test case
    points = [[[0.0, 1.0], [4.0, 2.0], [4.0, 0.0]], [[4.0, 8.0], [6.0, 7.0], [4.0, 5.0], [7.0, 6.0]], [[6.0, 8.0], [9.84, 8.87], [7.16, 11.76]], [[6.0, 10.0], [8.0, 10.0], [5.0, 11.74], [6.0, 7.46], [8.0, 13.46], [9.0, 11.73]]]

    # isolated test case
    # points = [[[2.0, 2.0], [0.0, 6.0], [4.0, 10.0], [10.0, 7.0], [7.0, 3.0]], [[7.0, -2.0], [7.0, 0.0], [9.0, 0.0], [9.0, -2.0]], [[22.0, 0.0], [20.0, 4.0], [24.0, 8.0], [30.0, 5.0], [27.0, 1.0]], [[12.0, 14.0], [10.0, 21.0], [14.0, 30.0], [20.0, 23.0], [17.0, 14.0]], [[12.0, 12.0], [17.0, 15.0], [26.0, 15.0], [23.0, 12.0]], [[25.0, 20.0], [23.0, 28.0], [29.0, 29.0], [34.0, 21.0]]]
    
    # another isolated test case
    # points = [[[0.7, 4.06], [0.6, 2.01], [2.42, 2.95]], [[4.5, 2.59], [5.76, 1.95], [5.14, 3.81], [6.4, 3.17]], [[4.98, 4.87], [6.0, 5.0], [6.4, 5.95], [5.77,6.77], [4.75, 6.64], [4.36, 5.69]]]
    
    # yet another isolated case
    # points = [[[0.0, 1.0], [1.5, 4.0], [1.0, 6.0]], [[4.0, 4.0], [7.0, 4.0], [5.5, 8.0]]]
    
    start = np.array([0, 0])
    end = np.array([8, 8])

    vg = VisibiltyGraph(points, start, end)
    vg.findShortestPath()
    print (vg.shortestPath)
    vg.plotPolygonsAndPaths()