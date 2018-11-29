import numpy as np

def readFile(filename):
    points = []
    with open(filename, "r") as f:
        s = []
        for line in f.readlines():
            line = line.strip('\r\n')
            if 'polygon' in line:
                if s == []:
                    pass
                else:
                    points.append(s)
                    s = []
                continue
            s.append(list(map(float, line.split())))
        else:
            points.append(s)

    if "overlap" not in filename:
        mode = "isolated"
    else:
        mode = "overlap"

    return points, mode

points, mode = readFile('../data/overlapping.txt')
# points, mode = readFile('../data/isolated.txt')
print (points, mode)