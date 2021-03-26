#!/usr/bin/env python

import rospy


from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from ros_dvrk_vr.msg import XYZcloud



def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.init_node(ros_node_name, anonymous=True)

    # Start the Server Endpoint with a ROS communication objects dictionary for routing messages
    tcp_server.start({
        
        #"/unity/ECM/base_frame/": RosPublisher('/dvrk/ECM/set_base_frame/', Pose, queue_size=10),
        #"/unity/PSM1/base_frame/": RosPublisher('/dvrk/PSM1/set_base_frame/', Pose, queue_size=10),
        #"/unity/PSM2/base_frame/": RosPublisher('/dvrk/PSM2/set_base_frame/', Pose, queue_size=10),
        "/unity/ECM/base_frame/": RosPublisher('/unity/ECM/base_frame_output/', Pose, queue_size=10),
        "/unity/PSM1/base_frame/": RosPublisher('/unity/PSM1/base_frame_output/', Pose, queue_size=10),
        "/unity/PSM2/base_frame/": RosPublisher('/unity/PSM2/base_frame_output/', Pose, queue_size=10),
        

        '/unity/Left_Controller/pose/': RosPublisher('/dvrk/PSM1/set_position_goal_cartesian', Pose, queue_size=10),
        '/unity/Right_Controller/pose/': RosPublisher('/dvrk/PSM2/set_position_goal_cartesian', Pose, queue_size=10),

        '/unity/XYZCloud': RosPublisher('/unity/XYZCloud', XYZcloud, queue_size = 10),

        '/dvrk/PSM1/joint_states/': RosSubscriber('/dvrk/PSM1/joint_states/', JointState, tcp_server),
        '/dvrk/PSM2/joint_states/': RosSubscriber('/dvrk/PSM2/joint_states/', JointState, tcp_server),
        '/dvrk/ECM/joint_states/': RosSubscriber('/dvrk/ECM/joint_states/', JointState, tcp_server),

    })
    
    rospy.spin()


if __name__ == "__main__":
    main()
