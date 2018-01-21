import rospy
import math
import time
from random import randint
import sensor_msgs.msg as sensors
import geometry_msgs.msg as geometries

pose_pub = rospy.Publisher('pose', geometries.Pose2D,queue_size=10)

now = time.clock()

global numScans
numScans = 0

map_ready = False
global robot_pose_prev
robot_pose_prev = geometries.Pose2D()
global obstacles_previous
obstacles_previous = geometries.PoseArray()
obstacles_previous.poses.append

def init_globals():
    emptyPose = geometries.Pose()
    emptyPose.position.x = 0
    emptyPose.position.y = 0
    emptyPose.orientation.w = 0
    obstacles_previous.poses.append(emptyPose)
    obstacles_previous.poses.append(emptyPose)

def distance_pose2D( one, two):  # geometries.Pose2D, geometries.Pose2D
    return distance(one.x, one.y, two.x, two.y)


def calculate_robot_position(obstacles, robot_location_prev, obstacles_prev):
    # tuple of two yeti_snowplow.obstacle, geometries.Pose2D, tuple of two yeti_snowplow.obstacle
    if(len(obstacles.poses) < 2):
        return 0
    heading0 = -(obstacles.poses[0].orientation.w - obstacles_prev.poses[0].orientation.w) + robot_location_prev.theta
    heading1 = -(obstacles.poses[1].orientation.w - obstacles_prev.poses[1].orientation.w) + robot_location_prev.theta
    heading = (heading0 + heading1) / 2

    xa = obstacles.poses[0].position.x
    ya = obstacles.poses[0].position.y
    da = distance(obstacles.poses[0].position.x,obstacles.poses[0].position.y,robot_location_prev.x,robot_location_prev.y)
    xb = obstacles.poses[1].position.x
    yb = obstacles.poses[1].position.y
    db = distance(obstacles.poses[1].position.x,obstacles.poses[1].position.y,robot_location_prev.x,robot_location_prev.y)

    o = (2 * yb - 2 * ya)
    p = (da * da - db * db - xa * xa + xb * xb - ya * ya + yb * yb) / o
    q = -(2 * xb - 2 * xa) / o

    s = p - ya
    t = q * q + 1
    u = 2 * s * q - 2 * xa
    v = xa * xa + s * s - da * da

    answer_one = geometries.Pose2D()
    answer_one.x = -(-u + math.sqrt(u * u - 4 * t * v)) / (2 * t)
    answer_one.y = q * answer_one.x + p
    answer_one.theta = heading
    answer_two = geometries.Pose2D()
    answer_two.x = -(-u - math.sqrt(u * u - 4 * t * v)) / (2 * t)
    answer_two.y = q * answer_two.x + p
    answer_two.theta = heading

    if (distance_pose2D(answer_one, robot_location_prev) < distance_pose2D(answer_two, robot_location_prev)):
        return answer_one
    else:
        return answer_two


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


def close_enough(a,b):
    if abs(a.x - b.x) < 0.2 and abs(b.y - b.y) < 0.2:
        return True
    return False


def update_map(obstacles):
    global numScans
    global map_ready
    global obstacles_previous
    global robot_pose_prev

    if not map_ready:
        if numScans > 50:
            map_ready = True
            print "Map ready! Localizing ...."
            robot_pose = calculate_robot_position(obstacles,robot_pose_prev,obstacles_previous)
            pose_pub.publish(robot_pose)
            robot_pose_prev = robot_pose
        else:
            print "Number of scans: " + str(numScans)

            if not close_enough(obstacles_previous.poses[0].position,obstacles.poses[0].position) or not close_enough(obstacles_previous.poses[1].position,obstacles.poses[1].position):
                numScans = 0
                obstacles_previous = obstacles



    if map_ready:
        robot_pose = calculate_robot_position(obstacles, robot_pose_prev, obstacles_previous)
        if(robot_pose):
            pose_pub.publish(robot_pose)
            robot_pose_prev = robot_pose
            print robot_pose

    numScans = numScans + 1


def main():
    init_globals()
    rospy.init_node('localization_lidar_odom', anonymous=True)
    #rospy.Subscriber("scan", sensors.LaserScan, update_map)
    rospy.Subscriber("obstacles", geometries.PoseArray, update_map)
    rospy.spin()


if __name__ == '__main__':
    main()