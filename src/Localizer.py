#!/usr/bin/python

import rospy
import geometry_msgs.msg as geometries
import math


class Localizer:

    def __init__(self):
        self.pose_pub = rospy.Publisher('pose', geometries.Pose2D, queue_size=10)
        self.map_ready = False
        self.robot_pose_prev = geometries.Pose2D()
        self.obstacles_map = geometries.PoseArray()
        emptyPose = geometries.Pose()
        emptyPose.position.x = 0
        emptyPose.position.y = 0
        emptyPose.orientation.w = 0
        self.obstacles_map.poses.append(emptyPose)
        self.obstacles_map.poses.append(emptyPose)

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def distance_pose2D(self, one, two):  # geometries.Pose2D, geometries.Pose2D
        return self.distance(one.x, one.y, two.x, two.y)


                    #obstacles is a generated map

    def calculate_robot_position(self, obstacles, obstacles_prev, robot_location_prev, map):
        # geometries.PoseArray, geometries.Pose2D, geometries.PoseArray, returns Pose2D absolute coordinates
        if (len(obstacles.poses) < 2):
            return 0
        heading0 = -(
        obstacles.poses[0].orientation.w - obstacles_prev.poses[0].orientation.w) + robot_location_prev.theta
        heading1 = -(
        obstacles.poses[1].orientation.w - obstacles_prev.poses[1].orientation.w) + robot_location_prev.theta
        heading = (heading0 + heading1) / 2

        xa = map.poses[0].position.x
        ya = map.poses[0].position.y
        #da = self.distance(obstacles.poses[0].position.x, obstacles.poses[0].position.y, robot_location_prev.x,
                      #robot_location_prev.y)
        da = obstacles.poses[0].orientation.z


        xb = map.poses[1].position.x
        yb = map.poses[1].position.y
        #db = self.distance(obstacles.poses[1].position.x, obstacles.poses[1].position.y, robot_location_prev.x,
                      #robot_location_prev.y)
        db = obstacles.poses[1].orientation.z

        o = (2 * yb - 2 * ya)
        p = (da * da - db * db - xa * xa + xb * xb - ya * ya + yb * yb) / o
        q = -(2 * xb - 2 * xa) / o

        s = p - ya
        t = q * q + 1
        u = 2 * s * q - 2 * xa
        v = xa * xa + s * s - da * da
        # print str(map)
        # print str(t) + ", " + str(u) + ", " + str(v)

        answer_one = geometries.Pose2D()
        answer_one.x = (-u + math.sqrt(u * u - 4 * t * v)) / (2 * t)
        answer_one.y = q * answer_one.x + p
        answer_one.theta = heading
        answer_two = geometries.Pose2D()
        answer_two.x = (-u - math.sqrt(u * u - 4 * t * v)) / (2 * t)
        answer_two.y = q * answer_two.x + p
        answer_two.theta = heading

        if (self.distance_pose2D(answer_one, robot_location_prev) < self.distance_pose2D(answer_two, robot_location_prev)):
            return answer_one
        else:
            return answer_two

    def close_enough(self, a, b):
        if abs(a.x - b.x) < 0.2 and abs(b.y - b.y) < 0.2:
            return True
        return False

    def localize(self, obstacles,obstacles_prev, map):
        self.pose = self.calculate_robot_position(obstacles,obstacles_prev,self.robot_pose_prev,map)
        if self.pose == 0:
            pass #use odom
        else:
            self.robot_pose_prev = self.pose
        #self.pose_pub.publish(self.pose)
        return self.pose
