import pclpy
from pclpy import pcl
from sensor_msgs.msg import PointCloud2
import numpy as np

# Load PLY file using PCL
cloud = pcl.PointCloud.PointXYZRGB()
pcl.io.loadPLYFile("input.ply", cloud)

# Extract point cloud data
points = cloud.xyz
colors = cloud.rgb

# Convert data to PointCloud2 message
header = pclpy.pcl_helper.get_default_header()
header.frame_id = "map"

# Create PointCloud2 message
msg = PointCloud2()
msg.header = header
msg.height = 1
msg.width = points.shape[0]
msg.fields = [
    pclpy.pcl_helper.create_field('x', 0, PointField.FLOAT32, 1),
    pclpy.pcl_helper.create_field('y', 4, PointField.FLOAT32, 1),
    pclpy.pcl_helper.create_field('z', 8, PointField.FLOAT32, 1),
    pclpy.pcl_helper.create_field('rgb', 16, PointField.UINT32, 1),
]
msg.is_bigendian = False
msg.point_step = 20
msg.row_step = msg.point_step * msg.width
msg.is_dense = True
msg.data = np.hstack((
    points.astype(np.float32).tostring(),
    colors.astype(np.uint8).tostring()
))

# Save PointCloud2 message to file
with open("output.bag", "wb") as f:
    f.write(msg.serialize())
