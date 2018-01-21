import rospy
import math
import time
from ObstacleDetector import ObstacleDetector
from Localizer import Localizer
from random import randint
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries

L = Localizer()
D = ObstacleDetector()
pose_pub = rospy.Publisher('pose', geometries.Pose2D, queue_size=10)
obstacles_pub = rospy.Publisher('map', geometries.PoseArray, queue_size=10)


def localize(scan):
    global L
    global D

    obstacles = D.detect_obstacles(scan)
    if obstacles == 0:
        return 0
    if obstacles != 0:
        obstacles_pub.publish(D.obstacles_map)
    #print "NumObstacles: " + str(len(obstacles.poses))
    if (D.numScans > 50):
        D.map_ready = True
        L.map_ready = True
        if(D.obstacles != 0 and D.obstacles_map != 0):
            pose = L.localize(D.obstacles, D.obstacles_map)
        if (pose != 0):
            D.robot_pose = pose
            pose_pub.publish(pose)
        print pose
    elif (D.numScans > 2 and not D.close_enough(obstacles.poses[0].position,
                                                D.obstacles_map.poses[0].position) and not D.close_enough(
            obstacles.poses[1].position, D.obstacles_map.poses[1].position)):
        print "Map initialization Failed at scan num: " + str(D.numScans)
        D.numScans = 0
    else:
        print "Map initializing, scan " + str(D.numScans) + "/50"


def main():
    rospy.init_node('localization_lidar_odom')
    rospy.Subscriber("scan", sensors.LaserScan, localize)
    rospy.spin()


if __name__ == '__main__':
    main()
