## Project: Perception Pick & Place
### Writeup of the project.

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

#### Note
For Exercise 1, 2 and 3, the functions are called within pcl_callback 
function of pr2_perception.py but they're contents are contained within 
the scripts directory.


#### Filtering

- The filtering performed on the input cloud data [image 1] is performed in 
  filter_with_sof_voxel_passthorugh() (filtering.py). The image is processed
  with a statistical outlier filter [image 2: SOF] which eliminates most of the 
  noise. 
- The data points are then downsampled using a volume-element (Voxel)
  grid method to reduce the number of data points required for processing; 
  this can be observed in [image 3: voxel grid]. 
- Lastly, the objects and the table are isolated by applying a passthrough 
  filter (voxel_to_passthrough) and are returned as pointcloud in 
  [image 4: passthough]. 

![image_1](./photos/Image_1-Noisy-PC.png)
![image_2](./photos/Image_2-SOF-Filtered.png)
![image_3](./photos/Image_3-Voxel-Filtered.png)
![image_4](./photos/Image_4-Passthrough-filtered.png)


#### Segmenation

- Segmentation occurs within the segment_inliers_and_outliers_using_ransac()
  function in segmentation.py. This function is used to remove the table from 
  the point cloud [image 5: table ransac] and as such, extract the objects from 
  the point cloud [image 6: objects ransac]. 

- It performs this operation by comparing each data point to the pre-defined 
  model (in this case the table). If the data points are within a certain 
  threshold of the model, the data point is considered an inlier (valid table 
  data point) and if not it is an outlier (objects or noise).

![image_5](./photos/Image_5-Table-RANSAC.png)
![image_6](./photos/Image_6-Objects-RANSAC.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  


#### Clustering

- Euclidean clustering is implemented within extract_clusters() (imported from
  clustering.py). This groups data points that are within the Euclidean 
  distance of each other. Each group of data points are assigned a color and 
  outputted (cluster_cloud). The cluster_cloud for scene XXX can be seen below. 

![image_7](./photos/Image_7-Clustering.png)


#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

#### Object_Recognition

##### Classifier
- The classifier was trained using both capture_features.py and train_svm.py.

- Firstly capture_features spawns the models of each object in a Gazebo
  environment, taking images of the objects. 
- It generates histograms from compute_color_histograms() and 
  compute_normal_histograms() and saves these to a file which obtains
  a unique feature profile of colour and surface normals for each object.
- The optimal captures for each objects was found to be 25.


- Next the support vector machine is trained in train_svm, defining the 
  classifier type. The optimal was configuration was found to be:
  clf = svm.SVC(kernel ='linear', gamma='scale', C=5). 

  The output of the trained classifier can be seen in 
  [image 8: output of train_svm]. 

![image_8](./photos/Image_8-Train-SVM.png)


##### Detect Objects
- Within the perception pipeline in pr2_perception.py, the function 
  detect_objects() makes a prediction by comparing histograms of the classifier
  and the histograms of the incoming point_cloud data. 
- It then outputs a the label and the object cluster to detected_objects 
  which is then published to RViz. For the first testXXX this can be viewed in 
  [image 9: below]


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), 

As you can see, each test and the normalised confusion matrix is shown below.

![image_9](./photos/Image_9-Object-Recognition-Pick-List-1.png)
![image_10](./photos/Image_9-Object-Recognition-Pick-List-2.png)
![image_11](./photos/Image_9-Object-Recognition-Pick-List-3.png)

#### Outputting PickPlace request to YAML file
- The first stage is to perform the object recognition in the pr2_perception.py 
  pcl_callback function. Just before this function ends, pr2_mover() is called.

- pr2_mover() is able to compare the items that are known to be in the scene 
  (pick_list objects), with the objects that pr2_perception has detected 
  (detected_objects). 
- For the objects that match the description, pr2_mover() then outputs a yaml 
  file which contains all the information attributed to the item in the 
  pick_list including location, dropbox, which arm to use etc.

#### Code critique and improvements


The objects were correctly identified in XXX of occassions which passes this
criteria XXX. 

The critiques of the code and suggested improvements are:

- Critique 1: Not removing the dropboxes from the pointcloud; This leads to 
  them being attributed labels which are not included in the pick_list files
  and therefore identifying wrongly; soap2 to be specific.

- Improvement 1: filter and segment these objects from the point cloud data

- Critique 2: The model is weakest at differentiating between objects XXX and 
  XXX. 

- Improvement 2: Increase the quality of the training set data by either: 
  - increasing the number of iterations in captured_features
  - Increase the number of point_cloud data points by decreasing the voxel-grid 
    downsampling

- Critique 3: The object 'biscuits' consistently is labelled twice, notably
  in the environment XXX.

- Improvement 3: Similar methods as explained in Improvement 2 also including
  optimising the HSV values for greater robustness of histogram for biscuits
  model.






