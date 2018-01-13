#!/usr/bin/env python
import rospy
import math
import geometry_msgs.msg as geometries
import yeti_snowplow.msg as yeti_snowplow

robot_position_pub = rospy.Publisher('robot_position', geometries.Pose2D, queue_size=10)

robot_location_prev = geometries.Pose2D()

def distance_pose2D(one, two): #two geometries.Pose2D()
    return distance(one.x, one.y, two.x, two.y)

def distance(x1,y1,x2,y2):
    x = (x1 - x2)
    y = (y1 - y2)
    return math.sqrt(x*x + y*y)


def calculate_robot_position(landmarks): #tuple of two yeti_snowplow.obstacle
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
    answer_two = geometries.Pose2D()
    answer_two.x = (-u - math.sqrt(u*u - 4*t*v))/(2*t)
    answer_two.y = q*answer_two.x + p

    if(distance_pose2D(answer_one,robot_location_prev) < distance_pose2D(answer_two,robot_location_prev)):
        return answer_one
    else:
        return answer_two


def listener():
    rospy.init_node('robot_position_test', anonymous=True)
    # rospy.Subscriber("landmarks", yeti_snowplow.obstacle, calculate_robot_position)
    # rospy.spin()
    landmark1 = yeti_snowplow.obstacle()
    landmark1.x = 3
    landmark1.y = 4
    landmark1.distance = 5
    landmark2 = yeti_snowplow.obstacle()
    landmark2.x = 5
    landmark2.y = 6
    landmark2.distance = 7
    robot_position_pub.publish(calculate_robot_position((landmark1,landmark2)))


if __name__ == '__main__':
    listener()    