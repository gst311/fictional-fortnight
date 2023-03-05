import rospy
import rosbag
from sensor_msgs.msg import PointCloud2
import pclpy
from pclpy import pcl

# Initialize ROS node
rospy.init_node('ply_to_rosbag')

# Load PLY file using PCL
cloud = pcl.PointCloud.PointXYZ()
pcl.io.loadPLYFile("input.ply", cloud)

# Convert PCL point cloud to ROS PointCloud2 message
header = rospy.Header()
header.frame_id = 'map'
msg = pclpy.convert_to_PointCloud2(cloud, header)

# Write PointCloud2 message to rosbag file
bag = rosbag.Bag('output.bag', 'w')
bag.write('/point_cloud_topic', msg)
bag.close()
