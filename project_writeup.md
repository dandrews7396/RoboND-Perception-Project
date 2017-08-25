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
[Image4]: ./pr2_robot/scripts/images/ncm2.png
[Image5]: ./pr2_robot/scripts/images/ncm3.png
[Image6]: ./pr2_robot/scripts/images/clusters.png
[Image7]: ./pr2_robot/scripts/images/RANSAC_plot.png
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

![alt_text][Image7]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

For the clustering portion of `project.py` I, again, implemented the same code as for the exercises. The inliers from the RASAC filter were converted to XYZRGB format and passed through `EuclideanClusterExtraction()` using a Kd-tree with the same tolerances as in the exercises. Each of the extracted clusters were then assigned a colour, and appended to `color_cluster_point_list` which was then converted to a ROS message as `ros_cluster_cloud` and published on `pcl_cluster_pub` as seen in the following image.

![alt_text][Image6]

The cluster indices were also passed on for feature extraction and object recognition.

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.



![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)




Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

And here's another image!
![demo-2](https://user-images.githubusercontent.com/20687560/28748286-9f65680e-7468-11e7-83dc-f1a32380b89c.png)

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  
