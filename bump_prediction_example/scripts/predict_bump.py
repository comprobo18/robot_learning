#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from neato_node.msg import Bump
from cv_bridge import CvBridge
import cPickle as pickle
import numpy as np
import cv2


class PredictBumpNode(object):
    def __init__(self):
        rospy.init_node('bump_predictor')
        rospy.Subscriber('/bump', Bump, self.process_bump)
        r = rospkg.RosPack()
        self.im_downsampled = None
        model_path = r.get_path('bump_prediction_example') + \
            "/models/bump_detector.pkl"
        self.model = None
        with open(model_path, "rb") as f:
            self.model = pickle.load(f)
        self.bridge = CvBridge()
        rospy.Subscriber('camera/image_raw', Image, self.process_image)

    def process_image(self, m):
        im = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
        self.im_downsampled = cv2.resize(im, (48, 48))
        im_vectorized = self.im_downsampled.flatten()
        print(self.model.predict_proba(im_vectorized[np.newaxis, :]))

    def process_bump(self, msg):
        pass

    def run(self):
        while not rospy.is_shutdown():
            if self.im_downsampled is not None:
                cv2.imshow("image", self.im_downsampled)
            cv2.waitKey(10)


if __name__ == "__main__":
    node = PredictBumpNode()
    node.run()
