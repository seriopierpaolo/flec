#This message encapsulates the LiDAR Pointcloud data + Odometry algorithm output

#Header

# Standard Header inherited from odometry part
std_msgs/Header header

#Odometry Data
#nav_msgs/Odometry Odometry
geometry_msgs/PoseStamped Odometry

#Lidar Pointcloud

#std_msgs/Header header is it necessary?
sensor_msgs/PointCloud2 PointCloud