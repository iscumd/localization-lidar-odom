#!/usr/bin/env python
import rospy
import math
import time
from random import randint
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries
import yeti_snowplow.msg as yeti_snowplow

scan_filtered_pub = rospy.Publisher('scan_filtered', sensors.PointCloud,queue_size=10)
obstacle1_pub = rospy.Publisher('obstacle1', geometries.Pose2D,queue_size=10)
obstacle2_pub = rospy.Publisher('obstacle2', geometries.Pose2D,queue_size=10)

now = time.clock()

global numScans
numScans = 0
global obstacles
map_ready = False
obstacles = []
obstacles_prev = [[0,0],[0,0]]
robot_location_prev = geometries.Pose2D()


def distance_pose2D(one, two): #geometries.Pose2D, geometries.Pose2D
    return distance(one.x, one.y, two.x, two.y)


def distance(x1,y1,x2,y2):
    x = (x1 - x2)
    y = (y1 - y2)
    return math.sqrt(x*x + y*y)


def polar_to_cartesian(theta, rot):
    x = math.cos(theta) * rot
    y = math.sin(theta) * rot
    return [x,y]


def calculate_robot_position(landmarks, robot_location_prev, landmarks_prev):
    #tuple of two yeti_snowplow.obstacle, geometries.Pose2D, tuple of two yeti_snowplow.obstacle
    heading0 = -(landmarks[0].heading - landmarks_prev[0].heading) + robot_location_prev.theta
    heading1 = -(landmarks[1].heading - landmarks_prev[1].heading) + robot_location_prev.theta
    heading = (heading0 + heading1)/2
    
    xa = landmarks[0].x
    ya = landmarks[0].y
    da = landmarks[0].distance
    xb = landmarks[1].x
    yb = landmarks[1].y
    db = landmarks[1].distance

    o = (2*yb - 2*ya)
    p = (da*da - db*db - xa*xa + xb*xb - ya*ya + yb*yb)/o
    q = -(2*xb - 2*xa)/o

    s = p-ya
    t = q*q + 1
    u = 2*s*q - 2*xa
    v = xa*xa + s*s - da*da

    answer_one = geometries.Pose2D()
    answer_one.x = (-u + math.sqrt(u*u - 4*t*v))/(2*t)
    answer_one.y = q*answer_one.x + p
    answer_one.theta = heading
    answer_two = geometries.Pose2D()
    answer_two.x = (-u - math.sqrt(u*u - 4*t*v))/(2*t)
    answer_two.y = q*answer_two.x + p
    answer_two.theta = heading

    if(distance_pose2D(answer_one,robot_location_prev) < distance_pose2D(answer_two,robot_location_prev)):
        return answer_one
    else:
        return answer_two


def filter_laser_scan(data,robot_x,robot_y):
    start = data.angle_min
    points = []
    for i in range(0, len(data.ranges)):
        point = polar_to_cartesian(start + data.angle_increment * i, data.ranges[i])
        if((robot_x + point[0] > -2) and (robot_x + point[0] < 1.7) and (robot_y + point[1] > -2.5) and (robot_y + point[1] < 11.5) and (point[0] != 0 and point[1] != 0)):
            if(abs(point[0]) > .1 and abs(point[1]) > 1):
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


def update_map(data):
    global numScans
    global map_ready
    global obstacles
    global obstacles_prev
    points = filter_laser_scan(data, 0, 0)
    scan_filtered = sensors.PointCloud()

    for i in range(0,len(points)):
        new_point = geometries.Point32()
        new_point.x = points[i][0]
        new_point.y = points[i][1]
        scan_filtered.points.append(new_point)

    scan_filtered_pub.publish(scan_filtered)
    if not map_ready:
        if numScans > 50:
            map_ready = True
            print "Map ready!"
        else:
            print "Number of scans: " + str(numScans)
            obstacles = distance_cluster(points)
            obstacle1 = geometries.Pose2D()
            obstacle1.theta = 0
            obstacle1.x = obstacles[0][0]
            obstacle1.y = obstacles[0][1]
            obstacle1_pub.publish(obstacle1)

            obstacle2 = geometries.Pose2D()
            obstacle2.theta = 0
            obstacle2.x = obstacles[1][0]
            obstacle2.y = obstacles[1][1]
            obstacle2_pub.publish(obstacle2)
            if not close_enough(obstacles_prev[0],obstacles[0]) and not close_enough(obstacles_prev[1],obstacles[1]):
                numScans = 0
                obstacles_prev = obstacles



    if map_ready:
        #print "localization using map..."
        obstacle1 = geometries.Pose2D()
        obstacle1.theta = 0
        obstacle1.x = obstacles[0][0]
        obstacle1.y = obstacles[0][1]
        obstacle1_pub.publish(obstacle1)

        obstacle2 = geometries.Pose2D()
        obstacle2.theta = 0
        obstacle2.x = obstacles[1][0]
        obstacle2.y = obstacles[1][1]
        obstacle2_pub.publish(obstacle2)
    numScans = numScans + 1


def listener():

    rospy.init_node('localization_lidar_odom', anonymous=True)
    rospy.Subscriber("scan", sensors.LaserScan, update_map)
    rospy.spin()


if __name__ == '__main__':
    listener()