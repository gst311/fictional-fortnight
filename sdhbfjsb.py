#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from plyfile import PlyData, PlyElement

def read_ply_file(filename):
    # Read PLY file using PlyData library
    plydata = PlyData.read(filename)
    vertices = np.array(plydata['vertex'].data)

    # Extract x, y, z coordinates from vertices
    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    # Combine x, y, z coordinates into a single 3D array
    pointcloud = np.zeros((vertices.shape[0], 3))
    pointcloud[:, 0] = x
    pointcloud[:, 1] = y
    pointcloud[:, 2] = z

    return pointcloud

def publish_pointcloud(pointcloud, topic_name):
    # Initialize ROS node and publisher
    rospy.init_node('ply_publisher', anonymous=True)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)

    # Create PointCloud2 message header
    header = pc2.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"

    # Create PointCloud2 message from point cloud data
    pointcloud_msg = pc2.create_cloud_xyz32(header, pointcloud)

    # Publish PointCloud2 message on specified topic
    while not rospy.is_shutdown():
        pub.publish(pointcloud_msg)

if __name__ == '__main__':
    # Specify input and output files
    filename = 'input.ply'
    topic_name = 'pointcloud'

    # Read PLY file and publish as ROS message
    pointcloud = read_ply_file(filename)
    publish_pointcloud(pointcloud, topic_name)
