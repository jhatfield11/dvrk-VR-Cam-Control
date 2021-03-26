#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import numpy
import tf

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point 
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

def EndEffectorPoint(POS1, POS2, Zoom):

    FP = [(POS1.x + POS2.x)/2, (POS1.y + POS2.y)/2, (POS1.z + POS2.z)/2]

    EP = [FP[0] - Zoom*(FP[0] / abs(FP[0])), FP[1] - Zoom*(FP[1] / abs(FP[1])), FP[2] - Zoom*(FP[2] / abs(FP[2]))]

    V_x = FP
    V_z = [(POS1.x - POS2.x), (POS1.y - POS2.y), (POS1.z - POS2.z)]

    Rx = numpy.linalg.norm(V_x)
    Rz = numpy.linalg.norm(numpy.cross(V_x,V_z))
    Ry = numpy.linalg.norm(V_z)

    Rp = tf.transformations.concatenate_matrices(Rx,Ry,Rz)

    q =tf.transformations.quaternion_from_matrix(Rp)

    return Quaternion(*q), Point(*EP)


def EndEffectorRotation(vec_x, vec_z):

    V_x = [vec_x.x,vec_x.y,vec_x.z]
    V_z = [vec_z.x,vec_z.y,vec_z.z]

    
    Rx = numpy.linalg.norm(V_x)
    Rz = numpy.linalg.norm(numpy.cross(V_x,V_z))
    Ry = numpy.linalg.norm(V_z)

    Rp = tf.transformations.concatenate_matrices(Rx,Ry,Rz)

    q =tf.transformations.quaternion_from_matrix(Rp)
    return Quaternion(*q)

def Talker():
    z_max = 0.025
    Cam_pos = rospy.Publisher('/dvrk/ECM/set_position_goal_cartesian', Pose, queue_size=10)
    rospy.init_node('Camera_control', anonymous=True)
    rate = rospy.Rate(10.0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.wait_for_message('/unity/ECM/base_frame_output/', Pose)

    while not rospy.is_shutdown():

        try:
            COM_1 = tfBuffer.lookup_transform(
                'ecm_base_link', 'PSM1_tool_wrist_shaft_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            COM_2 = tfBuffer.lookup_transform(
                'ecm_base_link', 'PSM2_tool_wrist_shaft_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        try:
            COM_3 = tfBuffer.lookup_transform(
                'PSM1_tool_wrist_shaft_link', 'PSM2_tool_wrist_shaft_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Pose()

        [msg.orientation, msg.position]  = EndEffectorPoint(COM_1.transform.translation, COM_2.transform.translation, z_max)

        #msg.orientation = EndEffectorRotation(center_Point, COM_3.transform.translation)
        Cam_pos.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    Talker()

