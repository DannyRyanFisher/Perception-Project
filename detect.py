from sensor_stick.pcl_helper import *
from sensor_stick.marker_tools import *
from pr2_robot.srv import *

from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.msg import DetectedObject
from sensor_stick.msg import DetectedObjectsArray
from visualization_msgs.msg import Marker

import rospy
import numpy as np
from sensor_stick.srv import GetNormals

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def detect_objects(cluster_indices, cloud_objects, white_cloud, clf,
                   object_markers_pub, scaler, encoder):
    # Create some empty lists
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab points for the cluster from the extracted outliers(cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # Convert pcl_cluster to ros_msg data type
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
#        labeled_features.append([feature, model_name])

        # Make prediction, retrieve the label for the result
        # then add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish label to RViz
        label_pos = list(white_cloud[pts_list[0]])

        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)


    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels),
    detected_objects_labels))


    return detected_objects
