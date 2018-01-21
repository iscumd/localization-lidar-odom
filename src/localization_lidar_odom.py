import rospy
import math
import time
from random import randint
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries
import obstacles.msg as Obstacles

scan_filtered_pub = rospy.Publisher('scan_filtered', sensors.PointCloud,queue_size=10)
obstacle1_pub = rospy.Publisher('obstacle1', geometries.Pose2D,queue_size=10)
obstacle2_pub = rospy.Publisher('obstacle2', geometries.Pose2D,queue_size=10)

now = time.clock()

global numScans
numScans = 0
global obstacles
map_ready = False
obstacles_prev = [[0,0],[0,0]]
robot_pose = [0,0]
robot_pose_prev = [0,0]

def distance(x1,y1,x2,y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def polar_to_cartesian(theta, rot):
    x = math.cos(theta) * rot
    y = math.sin(theta) * rot
    return [x,y]

def triangulate(A, B, dist_to_A, dist_to_B):
    pose = []
    baseline = distance(A[0],A[1],B[0],B[1])


def filter_laser_scan(data,robot_x,robot_y):
    start = data.angle_min
    points = []
    for i in range(0, len(data.ranges)):
        point = polar_to_cartesian(start + data.angle_increment * i, data.ranges[i])
        if((robot_x + point[0] > -2) and (robot_x + point[0] < 1.7) and (robot_y + point[1] > -2.5) and (robot_y + point[1] < 11.5) and (point[0] != 0 and point[1] != 0)):
            if(abs(point[0]) > .1 and abs(point[1]) > 1):
                points.append([robot_x + point[0], robot_y + point[1]])
            elif(map_ready and abs(point[0]) > .08 and abs(point[1]) > .08):
                points.append([robot_x + point[0], robot_y + point[1]])
    return points


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


def close_enough(a,b):
    if abs(a[0] - b[0]) < 0.2 and abs(b[1] - b[1]) < 0.2:
        return True
    return False


def update_map(obstacles):
    global numScans
    global map_ready
    global obstacles_prev
    global robot_pose
    global robot_pose_prev
    #points = filter_laser_scan(data, robot_pose[0], robot_pose[1])
    #scan_filtered = sensors.PointCloud()

    #for i in range(0,len(points)):
    #    new_point = geometries.Point32()
    #    new_point.x = points[i][0]
    #    new_point.y = points[i][1]
    #    scan_filtered.points.append(new_point)

    #scan_filtered_pub.publish(scan_filtered)
    if not map_ready:
        if numScans > 50:
            map_ready = True
            print "Map ready!"
        else:
            print "Number of scans: " + str(numScans)

            if not close_enough(obstacles_prev[0],obstacles[0]) and not close_enough(obstacles_prev[1],obstacles[1]):
                numScans = 0
                obstacles_prev = obstacles



    if map_ready:
        print "localization using map..."

    numScans = numScans + 1


def listener():

    rospy.init_node('localization_lidar_odom', anonymous=True)
    #rospy.Subscriber("scan", sensors.LaserScan, update_map)
    rospy.Subscriber("/obstacle_detection/obstacles", Obstacles, update_map)
    rospy.spin()


if __name__ == '__main__':
    listener()