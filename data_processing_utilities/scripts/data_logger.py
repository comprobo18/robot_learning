#!/usr/bin/env python

import rospy
import Queue as queue
from sensor_msgs.msg import LaserScan, Image
from neato_node.msg import Bump, Accel
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import rospkg
import errno
import os
import csv
import tf


class DataLogger(object):
    def __init__(self):
        self.last_ranges = None
        self.last_bump = None
        self.last_accel = None
        self.last_cmd_vel = None
        self.lbutton_down_registered = False
        self.last_x, self.last_y = -1, -1
        self.q = queue.Queue()
        self.sensor_latency_tolerance = rospy.Duration(0.5)

        rospy.init_node('data_logger')
        r = rospkg.RosPack()
        self.data_dir = rospy.get_param('~data_dir', 'mydataset')
        self.data_save_path = r.get_path('data_processing_utilities') + \
            '/data/' + self.data_dir
        try:
            os.mkdir(self.data_save_path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        rospy.loginfo("data save directory " + self.data_save_path)
        rospy.Subscriber('extrapolated_scan', LaserScan, self.process_scan)
        rospy.Subscriber('camera/image_raw', Image, self.process_image)
        rospy.Subscriber('bump', Bump, self.process_bump)
        rospy.Subscriber('accel', Accel, self.process_accel)
        rospy.Subscriber('cmd_vel', Twist, self.process_cmd_vel)
        self.tf_listener = tf.TransformListener()
        self.b = CvBridge()
        cv2.namedWindow('camera image')
        cv2.setMouseCallback('camera image', self.process_mouse_event)

    def process_scan(self, msg):
        self.last_ranges = (msg.header.stamp, msg.ranges)

    def process_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # this flag gets consumed elsewhere
            self.lbutton_down_registered = True
        self.last_x, self.last_y = x, y

    def process_cmd_vel(self, msg):
        self.last_cmd_vel = (rospy.Time.now(),
                             (msg.linear.x,
                              msg.angular.z))

    def process_bump(self, msg):
        self.last_bump = (rospy.Time.now(),
                          (msg.leftFront,
                           msg.leftSide,
                           msg.rightFront,
                           msg.rightSide))

    def process_accel(self, msg):
        self.last_accel = (rospy.Time.now(),
                           (msg.accelXInG, msg.accelYInG, msg.accelZInG))

    def process_image(self, m):
        im = self.b.imgmsg_to_cv2(m, desired_encoding="bgr8")
        print(im.shape)
        self.q.put((m.header.stamp,
                    self.last_x,
                    self.last_y,
                    self.lbutton_down_registered,
                    im))
        self.lbutton_down_registered = False

    def run(self):
        with open(self.data_save_path + "/metadata.csv", "w") as csv_file:
            writer = csv.writer(csv_file)
            image_count = 0
            writer.writerows([['stamp',
                               'image_file_name',
                               'mouse_x',
                               'mouse_y',
                               'lbutton_down',
                               'key'] +
                              ['ranges_' + str(i) for i in range(361)] +
                              ['cmd_vel_linear_x',
                               'cmd_vel_angular_z'] +
                              ['bump_leftFront',
                               'bump_leftSide',
                               'bump_rightFront',
                               'bump_rightSide'] +
                              ['accelXInG', 'accelYInG', 'accelZInG'] +
                              ['odom_trans_x',
                               'odom_trans_y',
                               'odom_trans_z'] +
                              ['odom_orient_x',
                               'odom_orient_y',
                               'odom_orient_z',
                               'odom_orient_w']])
            while not rospy.is_shutdown():
                stamp, x, y, lbutton_down, image = self.q.get(timeout=10)

                # extrapolated scan so I don't know if it is or isn't)
                scan = [float('Inf')]*361
                if (self.last_ranges and abs(stamp - self.last_ranges[0]) <
                        self.sensor_latency_tolerance):
                    scan = self.last_ranges[1]
                else:
                    print("TIME OUT LASER")
                cmd_vel = [float('Inf')]*2
                if self.last_cmd_vel:
                    cmd_vel = self.last_cmd_vel[1]

                bump = [float('Inf')]*4
                if (self.last_bump and abs(stamp - self.last_bump[0]) <
                        self.sensor_latency_tolerance):
                    bump = self.last_bump[1]

                accel = [float('Inf')]*3
                if (self.last_accel and abs(stamp - self.last_accel[0]) <
                        self.sensor_latency_tolerance):
                    accel = self.last_accel[1]

                # check for odom transform
                transform_ts = \
                    self.tf_listener.getLatestCommonTime('odom',
                                                         'base_link')
                trans = [float('Inf')]*3
                rot = [float('Inf')]*4
                if abs(transform_ts - stamp) < self.sensor_latency_tolerance:
                    trans, rot = self.tf_listener.lookupTransform('odom',
                                                                  'base_link',
                                                                  transform_ts)
                    print trans

                cv2.imshow("camera image", image)
                key = cv2.waitKey(5) & 0xFF

                filename = "%010d.jpg" % (image_count,)
                print(len(scan))
                cv2.imwrite(self.data_save_path + "/" + filename, image)
                writer.writerows([[stamp.to_sec(),
                                   filename,
                                   x,
                                   y,
                                   int(lbutton_down),
                                   key] +
                                  list(scan) +
                                  list(cmd_vel) +
                                  list(bump) +
                                  list(accel) +
                                  list(trans) +
                                  list(rot)])
                image_count += 1


if __name__ == '__main__':
    node = DataLogger()
    node.run()
