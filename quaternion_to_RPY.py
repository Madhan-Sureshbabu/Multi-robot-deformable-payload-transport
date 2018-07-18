#!/usr/bin/env python  
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
# from tf.msg import *
from geometry_msgs.msg import Pose2D 
from tf.transformations import euler_from_quaternion

def handle_quaternion_robot0(msg):
    global eu
    # print msg.pose.pose
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    eu = euler_from_quaternion(quaternion)
    # print eu[2]
    pose_msg.x = msg.pose.pose.position.x
    pose_msg.y = msg.pose.pose.position.y
    pose_msg.theta = eu[2]
    # print pose_msg
    pub0.publish(pose_msg)

def handle_quaternion_robot1(msg):
    global eu
    # print msg.pose.pose
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    eu = euler_from_quaternion(quaternion)
    # print eu[2]
    pose_msg.x = msg.pose.pose.position.x
    pose_msg.y = msg.pose.pose.position.y
    pose_msg.theta = eu[2]
    # print pose_msg
    pub1.publish(pose_msg)

def handle_quaternion_robot2(msg):
    global eu
    # print msg.pose.pose
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    eu = euler_from_quaternion(quaternion)
    # print eu[2]
    pose_msg.x = msg.pose.pose.position.x
    pose_msg.y = msg.pose.pose.position.y
    pose_msg.theta = eu[2]
    # print pose_msg
    pub2.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('quaternion_to_RPY',anonymous=True)
    # turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/Robot0_odom',Odometry,handle_quaternion_robot0)
    pub0 = rospy.Publisher('poseRPY0',Pose2D,queue_size=1)
    pub1 = rospy.Publisher('poseRPY1',Pose2D,queue_size=1)
    pub2 = rospy.Publisher('poseRPY2',Pose2D,queue_size=1)
    pose_msg = Pose2D()
    rospy.spin()