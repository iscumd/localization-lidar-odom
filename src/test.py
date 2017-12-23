import math


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def distance_cluster(points):
    clusters = [2]
    clusters[0] = []
    clusters[0].append([points[0][0], points[0][1]])
    for i in range(1, len(points)):
        isFound = False
        for j in range(0,len(clusters)):
            dist = distance(points[i][0], points[i][1], clusters[j][0][0], clusters[j][0][1])
            if dist <= 0.25:
                clusters[j].append([points[i][0], points[i][1]])
                isFound = True
                break
        if not isFound:
            clusters.append([[points[i][0], points[i][1]]])

    averages = []
    for i in range(0, len(clusters)):
        x_avg = 0
        y_avg = 0
        for j in range(0, len(clusters[i])):
            x_avg = x_avg + clusters[i][j][0]
            y_avg = y_avg + clusters[i][j][1]
        averages.append([x_avg/len(clusters[i]), y_avg/len(clusters[i])])

    return averages


deez_points = []
deez_points.append([1, 1])
deez_points.append([1, 1.1])
deez_points.append([1, 1.2])
deez_points.append([5, 5])
deez_points.append([5, 5.1])
deez_points.append([5, 5.2])
deez_points.append([8, 8])
deez_points.append([8, 8.1])
deez_points.append([8, 8.2])

print distance_cluster(deez_points)