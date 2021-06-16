#!/usr/bin/env python
"""
    
"""
import rospy
import dvrk 
import yaml
import tf_conversions
import tf2_ros
import tf
import geometry_msgs.msg

from geometry_msgs.msg import Pose


def callback(data, arm):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"

    if  (arm == "PSM1" )| (arm == "PSM2"):
        t.child_frame_id = arm + "_psm_base_link"
    else:
        t.child_frame_id = "ecm_setup_base_link"

    t.transform.translation = data.position
    t.transform.rotation = data.orientation

    quaternion = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    #rospy.set_param("%s_xyz" % arm, "%d %d %d" % (data.position.x, data.position.x, data.position.x))
    #rospy.set_param("%s_xyz" % arm, "%d %d %d" % (roll, pitch, yaw))
    

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('Base_Subscriber', anonymous=True)
    arm = rospy.get_param('~arm')
    rospy.Subscriber("/unity/%s/base_frame_output/" % arm, Pose, callback, arm)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
