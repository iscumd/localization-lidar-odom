#!/usr/bin/python

import rospy
import math
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries


class ObstacleDetector:

    def __init__(self):
        self.obstacles_map = geometries.PoseArray()
        self.obstacles = geometries.PoseArray()
        self.robot_pose = geometries.Pose2D()
        self.robot_pose.x = 0
        self.robot_pose.y = 0
        self.robot_pose.theta = 0
        self.robot_pose_prev = geometries.Pose2D()
        #self.scan_filtered_pub = rospy.Publisher('scan_filtered', sensors.PointCloud, queue_size=10)
        #self.obstacles_pub = rospy.Publisher('obstacles', geometries.PoseArray, queue_size=10)
        self.numScans = 0
        self.map_ready = False

    def distance(self, x1,y1,x2,y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def polar_to_cartesian(self,theta, dist):
        x = math.cos(theta) * dist
        y = math.sin(theta) * dist
        return [x, y]

    def filter_laser_scan(self, data, robot_x, robot_y, robot_t):  # input data as polar coordinates, returns absolute points
        start = data.angle_min + robot_t
        points = []
        for i in range(0, len(data.ranges)):
            point = self.polar_to_cartesian(start + data.angle_increment * i, data.ranges[i])
            point.append(data.angle_increment * i)
            point.append(data.ranges[i])
            if ((robot_x + point[0] > -2) and (robot_x + point[0] < 1.7) and (robot_y + point[1] > 0) and ( # change this to proper filter dimensions later
                    robot_y + point[1] < 8) and (point[0] != 0 and point[1] != 0)):
                if (abs(point[0]) > .1 and abs(point[1]) > 1):
                    points.append([robot_x + point[0], robot_y + point[1], robot_t + point[2], point[3]])
                #elif (self.map_ready and abs(point[0]) > .08 and abs(point[1]) > .08):
                #    points.append([robot_x + point[0], robot_y + point[1], robot_t + point[2]])
        return points

    def distance_cluster(self, points):
        clusters = [2]
        clusters[0] = []
        if len(points) == 0:
            return 0
        clusters[0].append([points[0][0], points[0][1], points[0][2], points[0][3]])
        for i in range(1, len(points)):
            isFound = False
            for j in range(0, len(clusters)):
                dist = self.distance(points[i][0], points[i][1], clusters[j][0][0], clusters[j][0][1])
                if dist <= 0.25:
                    clusters[j].append([points[i][0], points[i][1], points[i][2], points[i][3]])
                    isFound = True
                    break
            if not isFound:
                clusters.append([[points[i][0], points[i][1], points[i][2], points[i][3]]])

        averages = []
        for i in range(0, len(clusters)):
            x_avg = 0
            y_avg = 0
            ang_avg = 0
            dist_avg = 0
            for j in range(0, len(clusters[i])):
                x_avg = x_avg + clusters[i][j][0]
                y_avg = y_avg + clusters[i][j][1]
                ang_avg = ang_avg + clusters[i][j][2]
                dist_avg = dist_avg + clusters[i][j][3]
            averages.append([x_avg / len(clusters[i]), y_avg / len(clusters[i]), ang_avg / len(clusters[i]), dist_avg/len(clusters[i])])
        return averages

    def close_enough(self, a, b):
        if abs(a.x - b.x) < 0.2 and abs(a.y - b.y) < 0.2:
            return True
        return False

    def detect_obstacles(self, scan):
        self.curr_scan = scan
        self.numScans = self.numScans + 1
        filtered_scan = self.filter_laser_scan(scan, self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta)
        self.filtered_scan = filtered_scan
        #self.scan_filtered_pub.publish(self.filtered_scan)
        if(len(filtered_scan) < 0):
            return 0
        obstacle_list = self.distance_cluster(filtered_scan)
        obstacles = geometries.PoseArray()
        if obstacle_list == 0:
            return 0
        for i in range(0, len(obstacle_list)):
            obs = geometries.Pose()
            obs.position.x = obstacle_list[i][0]
            obs.position.y = obstacle_list[i][1]
            obs.orientation.w = obstacle_list[i][2]
            obs.orientation.z = obstacle_list[i][3]
            obstacles.poses.append(obs)
        self.obstacles = obstacles
        if self.map_ready == False:
            self.obstacles_map = obstacles
        #self.obstacles_pub.publish(obstacles)
        return obstacles