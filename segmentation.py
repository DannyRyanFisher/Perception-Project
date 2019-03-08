import pcl

def ransac_point_filtered_point_cloud(cloud_filtered):

  # Make segmenter object
  seg = cloud_filtered.make_segmenter()

  # Identify table model and set segmentation method to RANSAC
  seg.set_model_type(pcl.SACMODEL_PLANE)
  seg.set_method_type(pcl.SAC_RANSAC)

  # Set maximum distance
  max_inlier_distance = 0.01
  seg.set_distance_threshold(max_inlier_distance)

  inliers, coefficients = seg.segment()

  return inliers


#  extracted_inliers = cloud_filtered.extract(inliers, negative=False)
#  extracted_outliers = cloud_filtered.extract(inliers, negative=True)


def segment_inliers_and_outliers_using_ransac(cloud_filtered):
  inliers = ransac_point_filtered_point_cloud(cloud_filtered)

  extracted_inliers = cloud_filtered.extract(inliers, negative=False)
  extracted_outliers = cloud_filtered.extract(inliers, negative=True)


  return extracted_inliers, extracted_outliers


def remove_outliers_with_SOF(cloud_filtered):
  outlier_filter = cloud_filtered.make_statistical_outlier_filter()

  # Set the number of neighboring points to analyze for any given point
  outlier_filter.set_mean_k(50)

  # Set threshold scale factor
  x = 1.0

  # Any point with a mean distance larger than global
  # (mean distance+x*std_dev)
  outlier_filter.set_std_dev_mul_thresh(x)

  # Finally call the filter function for magic
  cloud_filtered = outlier_filter.filter()

  return cloud_filtered

def clean_data(inliers, outliers):
  final_inliers = remove_outliers_with_SOF(inliers)
  final_outliers = remove_outliers_with_SOF(outliers)

  return final_inliers, final_outliers

def segment_inliers_and_outliers_completely(cloud_filtered):

  a, b = segment_inliers_and_outliers_using_ransac(cloud_filtered)

  i, e = clean_data(a,b)

  return i,e
