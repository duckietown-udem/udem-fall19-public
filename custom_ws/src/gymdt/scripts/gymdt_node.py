#!/usr/bin/env python3

import sys
if '/duckietown/' not in sys.path:
    sys.path.append('/duckietown/')

from utils.ros_helpers import launch_env

import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from gymdt.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2


class ROSAgent(object):
    def __init__(self):
        # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle = os.getenv('HOSTNAME')

        self.ik_action_sub = rospy.Subscriber('/{}/wheels_driver_node/wheels_cmd'.format(
            self.vehicle), WheelsCmdStamped, self._ik_action_cb)
        # Place holder for the action, which will be read by the agent in solution.py
        self.action = np.array([0, 0])
        self.updated = True

        # Publishes onto the corrected image topic 
        # since image out of simulator is currently rectified
        self.cam_pub = rospy.Publisher('/{}/image/compressed'.format(
            self.vehicle), CompressedImage, queue_size=10)
        
        # Publisher for camera info - needed for the ground_projection
        self.cam_info_pub = rospy.Publisher('/{}/camera_info_topic'.format(
            self.vehicle), CameraInfo, queue_size=1)

        # Initializes the node
        rospy.init_node('GymDuckietown')

    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True
    
    def _publish_info(self):
        """
        Publishes a default CameraInfo
        """

        self.cam_info_pub.publish(CameraInfo())      

    def publish_img(self, obs):
        """
        Publishes the image to the compressed_image topic, which triggers the lane following loop
        """
        img_msg = CompressedImage()

        time = rospy.get_rostime()
        img_msg.header.stamp.secs = time.secs
        img_msg.header.stamp.nsecs = time.nsecs

        img_msg.format = "jpeg"
        contig = cv2.cvtColor(np.ascontiguousarray(obs), cv2.COLOR_BGR2RGB)
        img_msg.data = np.array(cv2.imencode('.jpg', contig)[1]).tostring()

        self.cam_pub.publish(img_msg)
        self._publish_info()

if __name__ == '__main__':
    rosagent = ROSAgent()
    env = launch_env()
    obs = env.reset()
    r = rospy.Rate(15)  
    rosagent.publish_img(obs)

    while not rospy.is_shutdown():
        action = rosagent.action
        obs, reward, done, _ = env.step(action)

        if done:
            obs = env.reset()

        rosagent.publish_img(obs)
        r.sleep()

