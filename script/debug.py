#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from depth_image_proc import point_cloud2_to_image

class PointCloudToImage:
    def __init__(self):
        rospy.init_node('point_cloud_to_image')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('projected_image', Image, queue_size=1)
        self.pc_sub = rospy.Subscriber('point_cloud', PointCloud2, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, pc):
        # Convert point cloud to RGB image
        rgb_image = point_cloud2_to_image(pc, fx=912.513, fy=913.079, cx=641.488, cy=356.656)

        # Publish RGB image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, 'rgb8'))

if __name__ == '__main__':
    try:
        PointCloudToImage()
    except rospy.ROSInterruptException:
        pass