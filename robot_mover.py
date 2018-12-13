#!/usr/bin/env python
import rospy
import rospkg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Time
import math
# new code
from robot.msg import Test
import tf
import numpy as np

objectPosition = [0, 0, 0]
objectOrientation = [0, 0, 0]
robotPosition = [0, 0, 0]
robotOrientation = [0, 0, 0]
error = -1
movementDone = False


def callbackDestinationReached(data):
    global movementDone
    movementDone = True


def callbackCamera(data):
    global objectPosition
    global objectOrientation
    global error
    objectPosition = [data.translation[0], data.translation[1], data.translation[2]]
    objectOrientation = [data.rotation[0], data.rotation[1], data.rotation[2]]
    error = data.error

def callbackRobot(data):
    global robotPosition
    global robotOrientation
    robotPosition = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    quaternion = (data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    robotOrientation = [euler[0], euler[1], euler[2]]
    print(robotOrientation)


def listener():
    rospy.init_node('robot_mover', anonymous=True)

    rospy.Subscriber("chatter", Test, callbackCamera)
    rospy.Subscriber("/iiwa/state/CartesianPose", PoseStamped, callbackRobot)
    rospy.Subscriber("/iiwa/state/DestinationReached", PoseStamped, callbackRobot)
    pub = rospy.Publisher("/iiwa/command/CartesianPose", PoseStamped, queue_size = 1)
    msg = PoseStamped()
    global robotPosition
    global robotOrientation
    global objectOrientation
    global objectPosition
    global error
    global movementDone
    while True:
        """
        if robotPosition[2] != 0:
            msg.pose.position.x = robotPosition[0]
            msg.pose.position.y = robotPosition[1]
            msg.pose.position.z = robotPosition[2]
            quaternion = tf.transformations.quaternion_from_euler(robotOrientation[0], robotOrientation[1],
                                                                  robotOrientation[2]+0.01)
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            pub.publish(msg)
            """
        if error != -1:
            if math.fabs(objectOrientation[2]) > 0.08:
                msg.pose.position.x = robotPosition[0]
                msg.pose.position.y = robotPosition[1]
                msg.pose.position.z = robotPosition[2]
                quaternion = tf.transformations.quaternion_from_euler(-3.13, 0,
                                                                      robotOrientation[2] + 0.4*objectOrientation[2])
                msg.pose.orientation.x = quaternion[0]
                msg.pose.orientation.y = quaternion[1]
                msg.pose.orientation.z = quaternion[2]
                msg.pose.orientation.w = quaternion[3]
                pub.publish(msg)
            else:
                goal = [robotPosition[0]-objectPosition[1], robotPosition[1]-objectPosition[0], robotPosition[2]-objectPosition[2]+0.35]
                #print("current", robotPosition)
                #print("goal:", goal)
                if robotPosition[2] != 0:
                    msg.pose.position.x = robotPosition[0]
                    msg.pose.position.y = robotPosition[1]
                    msg.pose.position.z = robotPosition[2]
                    quaternion = tf.transformations.quaternion_from_euler(robotOrientation[0], robotOrientation[1],
                                                                          robotOrientation[2])
                    msg.pose.orientation.x = quaternion[0]
                    msg.pose.orientation.y = quaternion[1]
                    msg.pose.orientation.z = quaternion[2]
                    msg.pose.orientation.w = quaternion[3]


                    if (math.sqrt(math.pow(robotPosition[0]-goal[0],2)+math.pow(robotPosition[1]-goal[1],2)+math.pow(robotPosition[2]-goal[2],2))) > 0.02:
                        #print("not at goal")
                        if (math.sqrt(math.pow(robotPosition[0]-goal[0],2))>0.01):
                            if robotPosition[0] > goal[0]:
                                msg.pose.position.x = robotPosition[0] - 0.005
                            else:
                                msg.pose.position.x = robotPosition[0] + 0.005
                        if (math.sqrt(math.pow(robotPosition[1]-goal[1],2))>0.01):
                            if robotPosition[1] > goal[1]:
                                msg.pose.position.y = robotPosition[1] - 0.005
                            else:
                                msg.pose.position.y = robotPosition[1] + 0.005
                        if (math.sqrt(math.pow(robotPosition[2]-goal[2],2))>0.02):
                            if robotPosition[2] > goal[2]:
                                msg.pose.position.z = robotPosition[2] - 0.015
                            else:
                                msg.pose.position.z = robotPosition[2] + 0.015
                    else:
                        print("at goal!!!")

                    quaternion = tf.transformations.quaternion_from_euler(-3.13, 0,
                                                                          robotOrientation[2] + 0.4* objectOrientation[
                                                                              2])
                    msg.pose.orientation.x = quaternion[0]
                    msg.pose.orientation.y = quaternion[1]
                    msg.pose.orientation.z = quaternion[2]
                    msg.pose.orientation.w = quaternion[3]
                    #print("publishing")
                    pub.publish(msg)
                    """
                    while ~movementDone:
                        print("not done")
                    print("done")
                    movementDone = False
                    """
                rospy.sleep(1/25)
        else:
            print("can't read camera data")



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
