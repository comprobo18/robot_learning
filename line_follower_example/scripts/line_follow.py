#!/usr/bin/env python

from __future__ import print_function
import rospy
import rospkg
from sensor_msgs.msg import Image
from neato_node.msg import Bump
from cv_bridge import CvBridge import cPickle as pickle
import numpy as np
import cv2
from tensorflow.keras.models import load_model
from skimage.transform import resize
from geometry_msgs.msg import Twist, Vector3


class FollowLine(object):
    def __init__(self):
        rospy.init_node('line_follower')
        r = rospkg.RosPack()
        self.im_to_classify = None
        model_path = r.get_path('line_follower_example') + \
            "/models/convolutional_model_python2.h5"
        self.model = load_model(model_path)
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/image_raw', Image, self.process_image)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print(self.model.summary())

    def lower_half_img(self, img):
        """
        Returns the lower half rows of an image

        Args: array (array): the array you want to extract the lower half from

        Returns: The lower half of the array
        """
        return img[int(round(img.shape[0]/2)):, :, :]

    def preprocess_img(self, img):
        img = self.lower_half_img(img)
        img = resize(img, (120, 320))
        return img

    def process_image(self, m):
        im = self.bridge.imgmsg_to_cv2(m, desired_encoding="rgb8")
        im = im / 255.0
        im = self.preprocess_img(im)
        self.im_to_classify = im[np.newaxis, :, :, :]

    def run(self):
        while not rospy.is_shutdown():
            if self.im_to_classify is not None:
                omega = self.model.predict(self.im_to_classify)
                msg = Twist(linear=Vector3(x=.1), angular=Vector3(z=omega))
                self.pub.publish(msg)
            cv2.waitKey(100)


if __name__ == "__main__":
    node = FollowLine()
    node.run()
