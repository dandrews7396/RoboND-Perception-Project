## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

[//]: # (Image References)

[Image1]: ./pr2_robot/scripts/images/noisey.png
[Image2]: ./pr2_robot/scripts/images/sof_filtered.png
[Image3]: ./pr2_robot/scripts/images/filtered.png
[Image4]: ./pr2_robot/scripts/images/ncm.png
[Image5]: ./pr2_robot/scripts/images/ncm2.png
[Image6]: ./pr2_robot/scripts/images/ncm3.png
[Image7]: ./pr2_robot/scripts/images/clusters.png
[Image8]: ./pr2_robot/scripts/images/RANSAC_plot.png
[Image9]: ./pr2_robot/scripts/images/output_1.png
[Image10]: ./pr2_robot/scripts/images/output_2.png
[Image11]: ./pr2_robot/scripts/images/output_3.png
---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This document serves as the writeup for the Perception project.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

For this element of the project, I carried out the filtering as previously covered in the exercises for the perception module of the course. However, as the pcl library had been updated, I was able to make use of the statistical outlier filter. This serves to reduce the 'noise' contained within the raw image data, the result of which, can be seen in the following images.

![alt text][Image1]

![alt_text][Image2]

The next step was to implement a Voxel Downsampling filter.  Voxel Downsampling filters an image by splitting it into a 3D grid of Voxels, all data within that Voxel is then assigned to detected items within its bounds. These bounds can be altered in each axis, and are given the variable name `LEAF_SIZE` in my script.

After Voxel Downsampling, I carried out a Passthrough filter, however I deviated from the module exercises slightly in that I carried this out twice, in both the Z and X axes. A Passthrough filter operates in a similar manner to a band-pass filter in audio electronics in that in that you set two points within a sample, and the filter only allows the data between them to pass. From the exercises, we were only using this in the Z-axis. But by the time I carried out the clustering phase of this project, I saw that the camera was also picking up the dropboxes on either side and labeling them as detected objects. So, for this reason, I decided that implementing a second Passthrough filter would be the best method stop this from occuring. There was no particular reason for choosing the X-axis for this at the time, but I believe this would be more appropriate should I move on to carry out the extra challenges.

Finally, I carried out the RAndom SAmple Consenus (RANSAC) filter on the image. RANSAC works in almost the opposite manner to Voxel Downsampling, in that it takes in all the data of the point cloud and groups together the different items by using a distance limit. Any points that fall within the limit are designated to the object (inliers), and those outside are ignored (outliers). The following plot from [Wikipedia](https://en.wikipedia.org/wiki/Random_sample_consensus) is a good visualisation of this concept:

![alt_text][Image8]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

For the clustering portion of `project.py` I, again, implemented the same code as for the exercises. The inliers from the RASAC filter were converted to XYZRGB format and passed through `EuclideanClusterExtraction()` using a Kd-tree with the same tolerances as in the exercises. Each of the extracted clusters were then assigned a colour, and appended to `color_cluster_point_list` which was then converted to a ROS message as `ros_cluster_cloud` and published on `pcl_cluster_pub` as seen in the following image.

![alt_text][Image7]

The cluster indices were also passed on for feature extraction and object recognition.

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

The next step was to implement object recognition on the point cloud. In order to do this, I would first need to train a classifier to recognise the objects. I started out using the same binning strategy (64) and amount of samples taken (45) as in the exercises. This produced a good Normalised Confusion Matrix, see below, but would produce inconsistant results across the board.

![alt_text][Image4]

Because of this, I chose to increase the sampling to fifty which again produced a seemingly better Confusion Matrix, but again inconsistant results.

![alt_text][Image5]

I then spent some time altering the values for binning and sampling, to see if I could make any improvement to the recognised objects. I found that using 64 bins produced the best results, although in altering sampling in proportion to the changes in binning always produced similar Confusion Matrices. That being said, I came to settle on using 64 bins, and a sample size of 65 produced the best results in the simulator. It does tend to classify glue as sticky notes roughly once in every five times I run test world 3. But overall, it was the most accurate in the simulator. The Normalised Confusion Matrix for this can be seen below.

![alt_text][Image6]


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

In carrying out the object recognition, the individual pcl clusters are extracted from the detected objects cloud and converted into an ROS array.  Then, using the `compute_color_histograms()` and `compute_normal_histograms()` functions of the `features.py` script from the `sensor_stick` exercises, we turn these arrays into feature vectors.

The features are then passed to the classifier to make a prediction, and have a label assigned depending on that prediction.  The object markers are then published by the object_markers_pub Publisher and it's various aspects are turned into a `DetectedObject` object.

Once this is completed for each of the detected object arrays, the script then uses `rospy.loginfo` to tell the user how many objects were detected, and how they were labeled by the classifier. A list containing each `DetectedObject` is then published to the `detected_objects_pub` publisher and, finally, the `pr2_mover()` function is called.

To start, `pr2_mover()` initialises a number of variables that are to be used when passing object information to the `.yaml` dictionary builder. More variables are then created using information that is read in from the launch file. Having seen a conversation on the slack channel regarding this area, I decided to alter the `pick_place_project.launch` file so that it would pass the number of the test environment to the python script as a parameter.

I then moved on to decide how my script would create it's reponses to pass to the robot.
I initially used nested for loops to iterate through the pick list, then another for the detected objects list with a conditional statement to match the names in each list. This worked fine to start with but soon I realised that, in doing this, the script would not correctly record items when they were incorrectly classified. To counter this behaviour, I separated the loops into one that assigned the pick arm and placement co-ordinates, and another that assigned detected objects. These were dependent on item group and object name respectively.

This now meant that if multiple items of the same name were detected or, if an item was wrongly classified as another item on the table, then both items would still be sent to the `.yaml` file. Images showing the results for each of the test environments can be seen below.

![alt_text][Image9]
Test World 1

![alt_text][Image10]
Test World 2

![alt_text][Image11]

Test World 3

In building the dictionary to send to the `.yaml` files, I used the `make_yaml_dict()` and `send_to_yaml()` functions provided in the template code and added a print statements to show the yaml dictionary
and notify when the `.yaml` file had been created, for debug purposes.

I then went on to create another utility function to send the response commands to the robot. I did this because, for some time, I was sending the commands on the fly as they were created. But this meant that the robot would need to carry out it's pick and place cycle between creating the commands and create the yaml dictionaries after. But quite often, it would crash into the table and fail before the full yaml file could be created, which was highly frustrating.

If I were to continue in developing this program further, I would need to consider a way of updating the collision data passed to the robot at the end of each cycle. I see the fact that I have separated the command creation out as a benefit, as I could use that funtion to help do this. I would potentially iterate through the detected objects point cloud, removing each one as their pick and place cycle came around. The collision map for the table could be implemented by publishing the `cloud_table` array to the `pr2/3d_map/points` topic, having rotated the robot around the base axis to ensure full coverage of the table surface and placement bins. However, due to summer holidays, I have overrun the submission deadline and will have to come back to this at a later date.
