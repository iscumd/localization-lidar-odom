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
map_pub = rospy.Publisher('map', geometries.PoseArray, queue_size=10)
obstacles_pub = rospy.Publisher('obstacles', geometries.PoseArray, queue_size=10)


def localize(scan):
    global L
    global D

    obstacles = D.detect_obstacles(scan)
    if obstacles == 0:
        return 0
    if obstacles != 0:
        map_pub.publish(D.obstacles_map)
        obstacles_pub.publish(D.obstacles)
    print "NumObstacles: " + str(len(obstacles.poses))
    if (D.numScans > 50):
        D.map_ready = True
        L.map_ready = True
        if(D.obstacles != 0 and D.obstacles_map != 0):
            pose = L.localize(D.obstacles, D.obstacles_prev, D.obstacles_map)
        if (pose != 0):
            D.robot_pose = pose
            pose_pub.publish(pose)
        #print pose
    elif (D.numScans > 2 and not D.close_enough(obstacles.poses[0].position,
                                                D.obstacles_map.poses[0].position) and not D.close_enough(
            obstacles.poses[1].position, D.obstacles_map.poses[1].position)):
        print "Map initialization Failed at scan num: " + str(D.numScans)
        D.numScans = 0
    else:
        print "Map initializing, scan " + str(D.numScans) + "/50"


def test():
    global L
    obs1 = geometries.Pose()
    obs1.position.x = 1
    obs1.position.y = 2

    obs2 = geometries.Pose()
    obs2.position.x = 0
    obs2.position.y = 7

    map = geometries.PoseArray()
    map.poses.append(obs1)
    map.poses.append(obs2)
    robot = geometries.Pose2D()

    for i in range(0,60):
        robot.x = 0
        robot.y = i/10.0
        da = L.distance(robot.x,robot.y,obs1.position.x,obs1.position.y)
        db = L.distance(robot.x, robot.y, obs2.position.x, obs2.position.y)

        obs3 = geometries.Pose()
        obs3.orientation.z = da
        obs3.orientation.w = 0

        obs4 = geometries.Pose()
        obs4.orientation.z = db
        obs4.orientation.w = 0

        obstacles = geometries.PoseArray()
        obstacles.poses.append(obs3)
        obstacles.poses.append(obs4)

        pose = L.localize(obstacles,obstacles,map)
        pose_pub.publish(pose)
        map_pub.publish(map)
        obstacles_pub.publish(obstacles)
        print robot.y
        #print "da: " + str(da) + ", db: " + str(db)
        time.sleep(0.3)
    #rospy.spinOnce()


def main():
    rospy.init_node('localization_lidar_odom')
    rospy.Subscriber("scan", sensors.LaserScan, localize)
    rospy.spin()


if __name__ == '__main__':
    main()
    #test()


