#!/usr/bin/env python
# coding=UTF-8
import rospy
import math
import time
from random import randint
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries
import yeti_snowplow.msg as yeti_snowplow

robot_location_pub = rospy.Publisher('/localization/robot_location', geometries.Pose2D, queue_size=5)
robot_velocity_pub = rospy.Publisher('/localization/velocity', geometries.Twist, queue_size=5)

class Localization:
    def __init__(self):
        #initialize
        self.last_time = time.clock()
        self.numScans = 0
        self.map_ready = False
        self.obstacles_prev = (yeti_snowplow.obstacle(),yeti_snowplow.obstacle())
        self.robot_location_prev = geometries.Pose2D()
    
    def distance_pose2D(self, one, two): #geometries.Pose2D, geometries.Pose2D
        return self.distance(one.x, one.y, two.x, two.y)

    def distance(self, x1,y1,x2,y2):
        x = (x1 - x2)
        y = (y1 - y2)
        return math.sqrt(x*x + y*y)


    def calculate_robot_position(self, landmarks, robot_location_prev, landmarks_prev):
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

        if(self.distance_pose2D(answer_one,robot_location_prev) < self.distance_pose2D(answer_two,robot_location_prev)):
            return answer_one
        else:
            return answer_two


    def close_enough(self,a,b):
        if abs(a.x - b.x) < 0.2 and abs(a.y - b.y) < 0.2:
            return True
        return False


    def update_map(self, data):
        obstacles = data.obstacles
        
        if not self.map_ready:
            if self.numScans > 50:
                self.map_ready = True
                print "Map ready!"
                self.last_time = time.clock() #get a better velocity estimate
            elif self.numScans == 0:
                pass
            else:
                print "Number of scans: " + str(self.numScans)
                if not self.close_enough(self.obstacles_prev[0],obstacles[0]) and not self.close_enough(self.obstacles_prev[1],obstacles[1]):
                    self.numScans = 0
            self.numScans = self.numScans + 1
        else: #if self.map_ready:
            number_of_obstacles = len(obstacles)
            if number_of_obstacles == 2:
                robot_location = self.calculate_robot_position(obstacles, self.robot_location_prev, self.obstacles_prev)
                robot_location_pub.publish(robot_location)

                delta_robot_location = self.distance_pose2D(robot_location, self.robot_location_prev)
                delta_time = time.clock() - self.last_time
                velocity = geometries.Twist()
                velocity.linear.x = delta_robot_location/delta_time #in meters per second
                velocity.angular.z = (robot_location.theta - self.robot_location_prev.theta)/delta_time #in radians per second
                robot_velocity_pub.publish()

                self.robot_location_prev = robot_location
                self.last_time = time.clock()
            elif number_of_obstacles < 2:
                pass
            else:
                pass #¯\_(ツ)_/¯
        self.obstacles_prev = obstacles


def listener():
    rospy.init_node('localization_lidar_odom', anonymous=True)
    localization = Localization()
    rospy.Subscriber("/obstacle_detection/obstacles", yeti_snowplow.obstacles, localization.update_map)
    rospy.spin()


if __name__ == '__main__':
    listener()