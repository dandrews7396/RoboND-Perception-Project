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

# Helper function to create a yaml friendly dictionary from ROS messages
def make_response_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    response_dict = {}
    response_dict["test_scene_num"] = test_scene_num.data
    response_dict["arm_name"]  = arm_name.data
    response_dict["object_name"] = object_name.data
    response_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    response_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return response_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    
    # Statistical Outlier Filtering
    sof = cloud.make_statistical_outlier_filter()

    # Set means-K cluster: Number of neighbouring points to analyse for any given point
    sof.set_mean_k(50)
    # Threshold scale factor
    x = 1.0
    # Set standard devation multiplier threshold (x). Any point with mean distance larger than
    # mean distance + x * std_dev will be considered an outlier
    sof.set_std_dev_mul_thresh(x)
    # Apply statistical outlier filter 
    sof_filtered = sof.filter()

    # Voxel Grid Downsampling
    vox = sof_filtered.make_voxel_grid_filter()

    # create leaf size
    LEAF_SIZE = 0.005
    # Set leaf size for voxel filter
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Apply Voxel filter
    vox_filtered = vox.filter()

    # PassThrough Filter
    filter_axis1 = 'z' ## Set filter axis
    passthrough1 = vox_filtered.make_passthrough_filter()
    passthrough1.set_filter_field_name(filter_axis1)
    # Set filter range
    axis_min = 0.605 # Taken from Exercise 1
    axis_max = 1
    passthrough1.set_filter_limits(axis_min, axis_max)
    # Apply Passthrough filter
    pt1_filtered = passthrough1.filter()

    # PassThrough Filter 2
    filter_axis2 = 'x' ## Set filter axis
    passthrough2 = pt1_filtered.make_passthrough_filter()
    passthrough2.set_filter_field_name(filter_axis2)
    # Set filter range
    axis_min = 0.4 # Taken from Exercise 1
    axis_max = 1
    passthrough2.set_filter_limits(axis_min, axis_max)
    # Apply Passthrough filter
    pt2_filtered = passthrough2.filter()

    # RANSAC Plane Segmentation
    seg = pt2_filtered.make_segmenter()

    # Set model for fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max Distance for point fitting models.
    max_distance = 0.0186
    seg.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = pt2_filtered.extract(inliers, negative = False)
    cloud_objects = pt2_filtered.extract(inliers, negative = True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # and min and max cluster sizes
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(25)
    ec.set_MaxClusterSize(10000)

    # Search KD tree for clusters
    ec.set_SearchMethod(tree)
    # Extract co-ords for each discovery
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud with uniquely coloured items
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    
    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


### Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert from pcl to ROS
        ros_cloud_array = pcl_to_ros(pcl_cluster)
        
        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cloud_array, using_hsv=True)
        normals = get_normals(ros_cloud_array)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud_array
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)
    
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):

    # Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()
    
    
    # Get/Read parameters
    do_list = detected_objects
    pick_list = rospy.get_param('/object_list')
    dbox_list = rospy.get_param('/dropbox')
    test_scene_num.data = rospy.get_param('test_scene_num')

    responses = []
    yaml_dict_list = []
    yaml_file = 'output_{}.yaml'.format(str(test_scene_num.data))
    
    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Check items on the pick list
    for pick in pick_list:
        # Check available drop boxes
        for box in dbox_list:
            # Check if box corresponds to ojects group
            if pick['group'] == box['group']:
                # Assign the arm to be used for pick_place
                arm_name.data = box['name']
                #Ensure item falls within box and not 'on' last item
                box['position'][0] -= 0.025
                # Create place_pose for object
                place_pose.position.x = box['position'][0]
                place_pose.position.y = box['position'][1]
                place_pose.position.z = box['position'][2]
        # Check objects that have been detected
        for do in do_list:
            # Check if the object is on the pick list
            if do.label == pick['name']:
                # Assign object name
                object_name.data = pick['name']
                # Create 'pick_pose' for the object
                points_array = np.mean(ros_to_pcl(do.cloud).to_array(), axis=0)[:3]
                
                pick_pose.position.x = np.asscalar(points_array[0])
                pick_pose.position.y = np.asscalar(points_array[1])
                pick_pose.position.z = np.asscalar(points_array[2])
                

        
        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        yaml_dict_list.append(yaml_dict)
        response_dict = make_response_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        responses.append(response_dict)
        print yaml_dict

    # Output your request parameters into output yaml file
    send_to_yaml(yaml_file, yaml_dict_list)
    print ".yaml file created!"

# Call responses as a separate function
# This should allow the whole process to run much more quickly
def send_moves(response_list):
    
    for response in response_list:
        test_scene_num = response["test_scene_num"]
        arm_name = response["arm_name"]
        object_name = response["object_name"]
        pick_pose = response["pick_pose"]
        place_pose = response["place_pose"]
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous = True)
    
    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
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
