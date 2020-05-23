#!/usr/bin/python
#Navid Kayhani (navidk381) | May 20,2020
'''This script listens to /gazebo/default/pose/info on gz topics
 and publishes it on a ROS topic'''
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
RATE = 200.0  # Hz

##############################
class GazeboMessageSubscriber: 

    def __init__(self, timeout=30):
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout
        rospy.init_node(NODE_NAME)
        self.rate = rospy.Rate(RATE)
        self.gz_publisher = GzPublisher()

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
            self.poses_subscriber = self.manager.subscribe('/gazebo/default/pose/info', 'gazebo.msgs.PosesStamped', self.poses_callback)

            yield From(self.poses_subscriber.wait_for_connection())
            self.running = True
            while self.running:
                yield From(asyncio.sleep(0.1))
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def poses_callback(self, data):
        # message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)
        # print('Received message:', message.data)
        self.poses_stamped = pygazebo.msg.poses_stamped_pb2.PosesStamped() #check the msg for more detail (poses_stamped_pb2.py)
        s = self.poses_stamped.FromString(data)
        self.gz_publisher.initialized = True
        test_string= String()
        test_string.data ="hello"
        self.gz_publisher.publish(test_string)
        self.rate.sleep()



############################# 
class GzPublisher(object):
    ''' Publishes the pose of bebop despite the wired incompatibility of Sphinx'''
    def __init__(self):
        # initialize your instant    
        #example1 = 
        self.initialized = False
        #example2 = 
        self.last_time = rospy.Time.now()
        
        # Publishers.
        # Specify the Topic Name and define publihsers 
        #example 
        topic_name = '/gazebo/' + "bebop" + '/' + "string"
        msg_type = String
        #compose PoseStamped
        queue_size = 10
        self.pose_pub = rospy.Publisher(topic_name, msg_type,   
                                             queue_size= queue_size)


    def publish(self,msg):
        ''' Publish pose estimate. '''
        if not self.initialized:
            return
        
        # Do somthing 

        # Create a message and publish it (make sure you have checked the message skeleton)
        # msg = compose_msg(t, q
        #publihster.publish(msg)
        self.pose_pub.publish(msg)

def main():
    gztest = GazeboMessageSubscriber()
    gztest.loop.run_until_complete(gztest.connect())

if __name__ == "__main__":
    main()
        