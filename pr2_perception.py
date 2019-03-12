#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

from extract import *
from detect import *

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    #Initialize variables

    test_scene_num = Int32()
    test_scene_num.data = 2 #scene number
    dict_list = []
    object_name_v = String()
    pick_pose = Pose()
    place_pose = Pose()
    arm_name = String()
    #Get/Read parameters

    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    #Parse parameters into individual variables

    for i in range(len(object_list_param)):
        object_name = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

    #loop through detected object and calculate their centroids

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    #Loop through the pick list
        for j, object in enumerate(object_list):



            if object_name == object.label :

                points_arr = ros_to_pcl(object.cloud).to_array()

                #get the centroid of object detected and exist in the pick list
                centroid = np.mean(points_arr, axis=0)[:3]

                object_name_v.data = object_name
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])

                #Assign the arm to be used for pick_place
                if object_group == dropbox_param[0]['group']:

                    arm_name.data = dropbox_param[0]['name']

                    place_pose.position.x = dropbox_param[0]['position'][0]
                    place_pose.position.y = dropbox_param[0]['position'][1]
                    place_pose.position.z = dropbox_param[0]['position'][2]

                elif object_group == dropbox_param[1]['group']:

                    arm_name.data = dropbox_param[1]['name']

                    place_pose.position.x = dropbox_param[1]['position'][0]
                    place_pose.position.y = dropbox_param[1]['position'][1]
                    place_pose.position.z = dropbox_param[1]['position'][2]

                #Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name_v, pick_pose, place_pose)
                dict_list.append(yaml_dict)




                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                    # TODO: Insert your message variables to be sent as a service request
                    resp = pick_place_routine(test_scene_num, arm_name, object_name_v, pick_pose, place_pose)

                    print("Response: ", resp.success)


                except rospy.ServiceException, e:
                    print ("Service call failed: %s" % e)
    #Output your request parameters into output yaml file
    send_to_yaml("output_"+str(test_scene_num.data), dict_list)

###############################################################################
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    # Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    PCL_data = ros_to_pcl(pcl_msg)
    # Voxel Grid Downsampling

    vox = PCL_data.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.01

    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # PassThrough Filter
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.601
    axis_max = 1
    passthrough.set_filter_limits(axis_min, axis_max)

    # use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()

    # Assign axis and range to the passthrough filter object.
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max = 0.4
    passthrough.set_filter_limits(axis_min, axis_max)

    # use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()



    outlier_filter = cloud_filtered.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(10)

    # Set threshold scale factor
    x = 0.1

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)


    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    # RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.02
    seg.set_distance_threshold(max_distance)

    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers

    extracted_inliers = cloud_filtered.extract(inliers, negative=False)

    extracted_outliers = cloud_filtered.extract(inliers, negative=True)
    cloud_objects = extracted_outliers
    # Euclidean Clustering

    # converting XYZRGB to XYZ
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)

    # creating kdtree
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(10000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_outliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(extracted_inliers)
    # Publish ROS messages
    pcl_clusters_pub.publish(ros_cluster_cloud)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)

#Note:objects not cloud_objects
    detected_objects = \
    detect_objects(cluster_indices, extracted_outliers, white_cloud, clf,
                   object_markers_pub, scaler, encoder)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

    return



    # TODO: Output your request parameters into output yaml file


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pr2_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2,\
              pcl_callback, queue_size = 1)
    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

    object_markers_pub = rospy.Publisher("/object_markers",
                                           Marker,
                                           queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects",
                                             DetectedObjectsArray,
                                             queue_size=1)



   # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown

    while not rospy.is_shutdown():
     rospy.spin()

###############################################################################

    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
