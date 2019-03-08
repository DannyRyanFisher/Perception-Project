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

def pr2_mover(detected_objects):

    # Initialize variables
    from std_msgs.msg import Int32
    test_scene_num = Int32()
    from std_msgs.msg import String
    object_name = String()
    arm_name = String()
    from geometry_msgs.msg import Pose
    pick_pose = Pose()
    place_pose = Pose()


    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Loop through the pick list
    dict_list = []
    for i in range(0, len(object_list_param)):
        # Parse parameters into individual variables
        object_list_name = object_list_param[i]['name']
        object_list_group = object_list_param[i]['group']
        for object in detected_objects:
        # Get the PointCloud for a given object
            if object.label == object_list_name:
                # Populate various ROS messages
                test_scene_num.data = 3
                object_name.data = object_list_name
                points_arr = ros_to_pcl(object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                pick_pose.position.x=np.asscalar(centroid[0])
                pick_pose.position.y=np.asscalar(centroid[1])
                pick_pose.position.z=np.asscalar(centroid[2])

                if object_list_group == 'green':
                    arm_name.data = 'right'
                    position = dropbox_param[1]['position']
                else:
                    arm_name.data = 'left'
                    position = dropbox_param[0]['position']
                place_pose.position.x=position[0]
                place_pose.position.y=position[1]
                place_pose.position.z=position[2]

                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                dict_list.append(yaml_dict)

                # Wait for 'pick_place_routine' service to come up
                # rospy.wait_for_service('pick_place_routine')

                # try:
                #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                #     # TODO: Insert your message variables to be sent as a service request
                #     resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                #     print ("Response: ",resp.success)

                # except rospy.ServiceException, e:
                #     print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
#    send_to_yaml('output_3.yaml', dict_list)
#    send_to_yaml('output_{}.yaml'.format(test_scene_num.data), dict_list)

###############################################################################
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    filtered = filter_with_sof_voxel_passthorugh(pcl_data)

    table_pcl, objects_pcl = segment_inliers_and_outliers_using_ransac(filtered)

    cluster_cloud, cluster_indices, white_cloud = extract_clusters(objects_pcl)

    table = pcl_to_ros(table_pcl)
    objects = pcl_to_ros(objects_pcl)
    clusters = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
#NOTE
    pcl_objects_pub.publish(objects)
    pcl_table_pub.publish(table)
    pcl_cluster_pub.publish(clusters)

  #Note:objects not cloud_objects
    detected_objects = \
    detect_objects(cluster_indices, objects_pcl, white_cloud, clf,
                   object_markers_pub, scaler, encoder)

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)


    return

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass



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
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)

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
