#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# Directory management
import os
#Tf Management
import tf2_ros 
#Import utility functions
import utils
import scipy.io as sio
import numpy as np
import threading
import time

class data_collector:
    def __init__(self):
        # Ge the current working directory and open a data folder
        path = os.getcwd()
        self.data_folder = os.path.join(path, "l3_mapping_data/")
        if not os.path.exists(self.data_folder):
            os.mkdir(self.data_folder)

        #Create a lock
        self.lock = threading.Lock()

        #Initialize the ros nodes
        rospy.init_node('image_listener')

        # Instantiate CvBridge
        self.bridge = CvBridge()

        #Instantiate a Tf Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tfstorage = np.zeros((1, 4, 4))

        # Subscriber bank
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.image_callback, queue_size=1) 

        #Initialize Variable
        self.stop = False   
        return

    def image_callback(self, msg):
        rate = rospy.timer.Rate(10)
        self.lock.acquire()
        if not self.stop:
            self.lock.release()
            print("Received an image!")
            time = msg.header.stamp
            try:
                # Convert your ROS Image message to OpenCV2
                cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                curr_tf = self.tf_buffer.lookup_transform('odom', 'base_footprint', time)
            except CvBridgeError, e:
                print(e)
            else:
    	        print("Saved")
                # Save your OpenCV2 image as a jpeg
                self.lock.acquire()
                if not self.tfstorage.any():
                    print('inital storage')
                    self.tfstorage = utils.tf_to_tf_mat(curr_tf.transform)[None, :, :]
                    self.lock.release()
                else:
                    self.tfstorage = np.append(self.tfstorage, utils.tf_to_tf_mat(curr_tf.transform)[None, :, :], axis=0)
                    self.lock.release()
                cv2.imwrite(os.path.join(self.data_folder, 'camera_image_{}.jpeg'.format(self.tfstorage.shape[0] - 1)), cv2_img)
        else:
            self.lock.release()
            rate.sleep()
        return

    def save_transforms(self):
        self.lock.acquire()
        print("There are {} Poses".format(self.tfstorage.shape[0]))
        sio.savemat(os.path.join(self.data_folder,'tf.mat'), dict(tf = self.tfstorage))
        self.lock.release()
        return 

def main():
    data_c = data_collector()
    # Spin until ctrl + c
    while True:    # infinite loop
        n = raw_input("Press s to stop data collection ")
        if n == "s":
            break  # stops the loop
    print('Stopping the Saving!')
    data_c.lock.acquire()
    data_c.stop=True
    data_c.lock.release()
    print("Saving Poses")
    data_c.save_transforms()

if __name__ == '__main__':
    main()
