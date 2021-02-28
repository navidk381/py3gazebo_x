#!/usr/bin/python
# Navid Kayhani (navidk381) | May 26,2020

'''
Publish the difference between Gazebo pose and tag-based estimates.
I'm looking for a systemic error caused by transfrom inconsistencies. 
'''

import numpy as np
import rospy
import tf


from dsl__utilities__msg.msg import StateData
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, UInt32
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, TransformStamped


NODE_NAME = 'gz_error_pub_node'
RATE = 60.0  # Hz

def decompose_pose_msg(pose_msg):
    ''' Decompose a Pose message into translation and roll-pitch-yaw. '''
    trans = pose_msg.position
    trans = np.array([trans.x, trans.y, trans.z])

    rpy = rpy_from_quat_msg(pose_msg.orientation)

    return trans, rpy

def rpy_from_quat_msg(quat_msg):
    ''' Convert a Quaternion message to roll-pitch-yaw. '''
    q = np.array([quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w])
    rpy = np.array(tf.transformations.euler_from_quaternion(q))
    return rpy

def decompose_transfrom_msg(transform_msg):
    ''' Decompose a Transform message into translation and roll-pitch-yaw. '''
    trans = transform_msg.translation
    trans = np.array([trans.x, trans.y, trans.z])

    rpy = rpy_from_quat_msg(transform_msg.rotation)

    return trans, rpy


class GzErrorPubNode(object):
    ''' What does this class do? '''
    def __init__(self):
        self.initialized = False
        self.t_gz = np.zeros(3)  # Drone Translation from gazebo world.
        self.rpy_gz = np.zeros(3)    # Roll-pitch-yaw from gazebo world..
        self.t_tag = np.zeros(3)  # DroneTranslation from tags.
        self.rpy_tag = np.zeros(3)    # Roll-pitch-yaw from tags.
        self.trans = np.zeros(3)  # Translation error.
        self.rpy = np.zeros(3)    # Roll-pitch-yaw error.
 #########################

        # Publishers.
        # Specify the Topic Name and define publihsers 
        #example 
        pub_topic_name = "/gazebo/" + "bebop/" + "pose/" + "error"
        msg_type = PoseStamped
        queue_size = 10
        self.gz_error_pub = rospy.Publisher(pub_topic_name, msg_type,   
                                             queue_size= queue_size)

        
#############################
        sub_topic_name = '/gazebo/bebop/pose'
        sub_msg_type = PoseStamped
        # Subscribers
        rospy.Subscriber(sub_topic_name, sub_msg_type, self.CB_gz_pose)
#############################
        sub_topic_name2 = '/tags/Bebop_2_58_cons/Bebop_2_58_cons'
        sub_msg_type2 = TransformStamped
        # Subscribers
        rospy.Subscriber(sub_topic_name2, sub_msg_type2, self.CB_gz_pose_tag)

        

    def CB_gz_pose(self, msg):
        ''' Callback for Gazebo poses /gazebo/bebop/pose (PoseStamped)'''
        self.t_gz, self.rpy_gz = decompose_pose_msg(msg.pose)
       
        #self.trans, self.rpy = decompose_pose_msg(msg.pose.pose)

    def CB_gz_pose_tag(self, msg):
        ''' Callback for tag-based estimates '/tags/Bebop_2_58_cons/Bebop_2_58_cons' (TransformStamped)'''
        self.initialized = True
        self.t_tag, self.rpy_tag = decompose_transfrom_msg(msg.transform)
        self.trans = self.t_gz - self.t_tag
        self.rpy = self.rpy_gz - self.rpy_tag
    
    def root_sqr_error(self):
         t_error2 = sum((self.t_gz - self.t_tag)**2)
         t_error = t_error2 ** (0.5)
         return t_error   

    def publish(self):
        ''' Publish pose estimate. '''
        if not self.initialized:
            return
       
        ######## Create a PoseStamped msg ### 
        msg = PoseStamped()

        # Header.
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        
        # Pose.
        # Position
        msg.pose.position = Point(*self.trans)
        # Orientation
        msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(*self.rpy))
        ####### Create a PoseStamped msg ### 
        rounded = list(map(lambda x : round(x,3),self.trans))
        rounded_rpy = list(map(lambda x : round(x,2),self.rpy))
        rospy.loginfo("(gz-tag) [xyz]error in x = {}, y = {}, z = {}".format(*rounded))
        rospy.loginfo("(gz-tag) [rpy]error in r = {},   p = {},   y = {}".format(*rounded_rpy))
        rospy.loginfo("(Error(m)) = {}".format(round(self.root_sqr_error(),3)))
        # rospy.loginfo("(Avg 30-Error(m)) = {}".format(round(self.root_sqr_error(),3)))     
        #publihster.publish (msg)
        self.gz_error_pub.publish(msg)



def main():
    rospy.init_node(NODE_NAME)

    rate = rospy.Rate(RATE)
    my_instance = GzErrorPubNode()

    while not rospy.is_shutdown():
        my_instance.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
