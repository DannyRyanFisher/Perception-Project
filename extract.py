from pcl_helper import *
from filtering import *
from segmentation import *
from clustering import *

def extract_objects_from_ros_point_cloud_data(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # Filter data points; using VoxelGrid and Passthrough methods
    filtered_point_cloud = filter_point_cloud(pcl_data)

    # Extract inliers (table) and outliers (objects)
    # from point cloud using RANSAC segmentation method
    table, objects =\
    segment_inliers_and_outliers_completely(filtered_point_cloud)

    # Complete Euclidean Clustering to extract cloud of clusters
    cluster_cloud, cluster_indices, white_cloud = extract_clusters(objects)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(objects)
    ros_cloud_table = pcl_to_ros(table)
    ros_cloud_cluster = pcl_to_ros(cluster_cloud)

    return ros_cloud_objects, ros_cloud_table, ros_cloud_cluster, cluster_indices, objects, white_cloud


