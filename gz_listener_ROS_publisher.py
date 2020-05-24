#!/usr/bin/python
# Navid Kayhani (navidk381) | May 20,2020
'''This script listens to /gazebo/default/pose/info on gazebo topics
 and publishes PoseStamped messages on a ROS topic'''

import trollius as asyncio
from trollius import From
import pygazebo
import pygazebo.msg.poses_stamped_pb2
# https://github.com/tensorflow/tensorflow/issues/890
# https://github.com/jpieper/pygazebo
# https://stackoverflow.com/questions/43948454/python-invalid-syntax-with-async-def
######################
import numpy as np
import rospy

from dsl__utilities__msg.msg import StateData
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import String, UInt32
#################################
NODE_NAME = 'gz_pose_publisher'
RATE = 60.0  # Hz
##############################

class GazeboMessageSubscriber:

    def __init__(self, timeout=30):
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout
        rospy.init_node(NODE_NAME)
        self.rate = rospy.Rate(RATE)
        self.gz_publisher = GzPublisher()
        self.gz_pose_msg = PoseStamped()

    @asyncio.coroutine
    def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = yield From(pygazebo.connect(('localhost', 11345)))
                connected = True
                break
            except Exception as e:
                pass
            yield From(asyncio.sleep(1))

        if connected:
            self.gz_publisher.initialized = True
            self.poses_subscriber = self.manager.subscribe(
                '/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.poses_callback)

            yield From(self.poses_subscriber.wait_for_connection())
            self.running = True
            while self.running and not rospy.is_shutdown():
                yield From(asyncio.sleep(0.1))
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def poses_callback(self, data):
        # Check the msg for more detail (poses_stamped_pb2.py)
        # Read the message form Gazebo
        self.poses_stamped = pygazebo.msg.poses_stamped_pb2.PosesStamped()
        s = self.poses_stamped.FromString(data)
        self.gz_pose_msg = s.pose[0]

        # Publish in ROS
        self.gz_publisher.publish(self.gz_pose_msg)
        self.rate.sleep()

#############################

class GzPublisher(object):
    ''' Publishes the pose of bebop despite the wired incompatibility of Sphinx'''

    def __init__(self):
        # initialize your instant
        self.initialized = False
        self.last_time = rospy.Time.now()

        # Publishers.
        # Specify the Topic Name and define the publihsers
        topic_name = '/gazebo/' + "bebop/" + "pose"
        msg_type = PoseStamped
        queue_size = 10
        self.pose_pub = rospy.Publisher(topic_name, msg_type,
                                        queue_size=queue_size)

    def publish(self, msg):
        ''' Publish pose estimate. '''
        if not self.initialized:
            return
        # Create a message and publish it (make sure you have checked the message skeleton)
        
        ros_msg = self.compose_ros_PoseStamped_msg(msg)
        # publihster.publish(msg)
        self.pose_pub.publish(ros_msg)


    def compose_ros_PoseStamped_msg(self, gz_msg_PosesStamped):
        # For more information about the parsed Gazebo messages, please see
        # <your_ros_ws>/gz_listener/src/py3gazebo_x/pygazebo/msg/poses_stamped_pb2.py ([Hint] Ctrl+p for file search in VSCode)
        # For the ROS message you should check: http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html
        msg = PoseStamped()

        # Header.
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'

        # Pose.
        # Position
        msg.pose.position = Point(gz_msg_PosesStamped.position.x,
                                       gz_msg_PosesStamped.position.y,
                                       gz_msg_PosesStamped.position.z)

        # Orientation
        msg.pose.orientation = Quaternion(gz_msg_PosesStamped.orientation.x,
                                               gz_msg_PosesStamped.orientation.y,
                                               gz_msg_PosesStamped.orientation.z,
                                               gz_msg_PosesStamped.orientation.w)
        return msg


def main():
    gztest = GazeboMessageSubscriber()
    gztest.loop.run_until_complete(gztest.connect())


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
