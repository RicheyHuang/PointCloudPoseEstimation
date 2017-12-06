/***********************************************************
Copyright: Huang Xiaohang / Piec
Author: Huang Xiaohang
Date: 2017-12-06
Description: Declaration of the classes used in the project
************************************************************/

#ifndef POINTCLOUDPOSEESTIMATION_POINTCLOUDTOOLBOX_H
#define POINTCLOUDPOSEESTIMATION_POINTCLOUDTOOLBOX_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

typedef pcl::PointXYZ                PointT;
typedef pcl::PointCloud<PointT>      PointCloudT;
typedef pcl::PointNormal             PointN;
typedef pcl::PointCloud<PointN>      PointCloudN;
typedef pcl::Normal                  Normal;
typedef pcl::PointCloud<Normal>      Normals;
typedef pcl::FPFHSignature33         DescriptorT;
typedef pcl::PointCloud<DescriptorT> FeaturesT;


class PIPointCloud
{
public:

    /*************************************************
    Function:       PIPointCloud
    Description:    Constructor
    Calls:          None
    Input:          None
    Output:         None
    Return:         None
    Others:         Initialize the class member cloud
    *************************************************/

    PIPointCloud();

    /*************************************************
    Function:       printTransformationMatrix
    Description:    Display the rotation matrix and translation vector of the point cloud transformed respectively
    Calls:          None
    Input:          None
    Output:         Showing transformation matrix and illustration on the console
    Return:         None
    Others:         The transformation matrix is stored by the class member transform_matrix
    *************************************************/

    void printTransformationMatrix ();

    /*************************************************
    Function:       pointCloudRotate
    Description:    Apply a rotation on a point cloud using a rotation matrix
    Calls:          None
    Input:          A 3 * 3 matrix
    Output:         A transformed point cloud
    Return:         None
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    void pointCloudRotate(Eigen::Matrix3f rotate_matrix);

    /*************************************************
    Function:       pointCloudTranslate
    Description:    Apply a translation on a point cloud using a translation vector
    Calls:          None
    Input:          A 3 * 1 vector with float variables
    Output:         A transformed point cloud
    Return:         None
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    void pointCloudTranslate(Eigen::Vector3f translate_matrix);

    /*************************************************
    Function:       pointCloudRotateX
    Description:    Apply a rotation around the x-axis on a point cloud by a specific angle
    Calls:          None
    Input:          Rotation angle in degree
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudRotateX(float theta);

    /*************************************************
    Function:       pointCloudRotateY
    Description:    Apply a rotation around the y-axis on a point cloud by a specific angle
    Calls:          None
    Input:          Rotation angle in degree
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudRotateY(float theta);

    /*************************************************
    Function:       pointCloudRotateZ
    Description:    Apply a rotation around the z-axis on a point cloud by a specific angle
    Calls:          None
    Input:          Rotation angle in degree
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudRotateZ(float theta);

    /*************************************************
    Function:       pointCloudTranslateX
    Description:    Apply a translation along the x-axis on a point cloud by a specific distance
    Calls:          None
    Input:          Translation distance in millimeter
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudTranslateX(float sigma);

    /*************************************************
    Function:       pointCloudTranslateY
    Description:    Apply a translation along the y-axis on a point cloud by a specific distance
    Calls:          None
    Input:          Translation distance in millimeter
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudTranslateY(float sigma);

    /*************************************************
    Function:       pointCloudTranslateZ
    Description:    Apply a translation along the z-axis on a point cloud by a specific distance
    Calls:          None
    Input:          Translation distance in millimeter
    Output:         A transformed point cloud
    Return:         matrix: the transformation matrix used
    Others:         The transformed point cloud will cover the original one,
                    the point cloud to be transformed is stored by the class member cloud
    *************************************************/

    Eigen::Matrix4f pointCloudTranslateZ(float sigma);

    /*************************************************
    Function:       pointCloudSlicingX
    Description:    Remove the parts of point cloud beyond the specific range along x-axis
    Calls:          None
    Input:          The start and end of the range
    Output:         A sliced point cloud
    Return:         None
    Others:         Use the coordinate value to indicate the starting and ending position,
                    the point cloud to be sliced is stored by the class member cloud
    *************************************************/

    void pointCloudSlicingX(float starting_position, float ending_position);

    /*************************************************
    Function:       pointCloudSlicingY
    Description:    Remove the parts of point cloud beyond the specific range along y-axis
    Calls:          None
    Input:          The start and end of the range
    Output:         A sliced point cloud
    Return:         None
    Others:         Use the coordinate value to indicate the starting and ending position,
                    the point cloud to be sliced is stored by the class member cloud
    *************************************************/

    void pointCloudSlicingY(float starting_position, float ending_position);

    /*************************************************
    Function:       pointCloudSlicingZ
    Description:    Remove the parts of point cloud beyond the specific range along z-axis
    Calls:          None
    Input:          The start and end of the range
    Output:         A sliced point cloud
    Return:         None
    Others:         Use the coordinate value to indicate the starting and ending position,
                    the point cloud to be sliced is stored by the class member cloud
    *************************************************/

    void pointCloudSlicingZ(float starting_position, float ending_position);

    /*************************************************
    Function:       computeFeatures
    Description:    Acquire the features for the point clouds to apply alignment
    Calls:          None
    Input:          normal_search_range: the parameter to provide an unit range for estimator to search the normals of point cloud, in millimeter
                    feature_search_range: the parameter to provide an unit range for estimator to calculate the features of point cloud, in millimeter
    Output:         The features of the point cloud
    Return:         features: the acquired features of the point cloud
    Others:         The features is used in the process of ransac
    *************************************************/

    FeaturesT::Ptr computeFeatures(float normal_search_range, float feature_search_range);

    PointCloudT::Ptr cloud;

    Eigen::Matrix4d transform_matrix;
};

class PointCloudRegistrator
{
public:

    /*************************************************
    Function:       setParametersForRANSAC
    Description:    Setup the parameters to apply a ransac initial alignment
    Calls:          None
    Input:          max_iteration: the maximum number of iterations
                    min_sample_distance: the minimum distances between samples, in millimeter
                    max_correspondence_distance: the maximum distance threshold between a point and its nearest neighbor, in millimeter
                    sample_num: the number of samples to use during each iteration
                    correspondence_randomness: the number of neighbors to use when selecting a random feature correspondence
                    ransac_outlier_reject_threshold:
                    the inlier distance threshold; considers a point to be an inlier, if the distance between the target data
                    index and the transformed source index is smaller than the given inlier distance threshold, in millimeter
    Output:         None
    Return:         None
    Others:         None
    *************************************************/

    void setParametersForRANSAC(int max_iteration, float min_sample_distance,     float max_correspondence_distance,
                                int sample_num,    int correspondence_randomness, float ransac_outlier_reject_threshold);

    /*************************************************
    Function:       alignUsingRANSAC
    Description:    Apply ransac on a point cloud to give it a proper initial position to implement the registration later
    Calls:          None
    Input:          cloud_source: the source point cloud
                    features_source: the features of source point cloud
                    cloud_target: the target point cloud
                    features_target: the features of target point cloud
    Output:         A transformed point cloud
    Return:         None
    Others:         A proper initial position can make registration faster and achieve more precise results
    *************************************************/

    void alignUsingRANSAC(PointCloudT::Ptr cloud_source, FeaturesT::Ptr features_source,
                          PointCloudT::Ptr cloud_target, FeaturesT::Ptr features_target);

    /*************************************************
    Function:       getRANSACTransformationMatrix
    Description:    Acquire the transformation matrix in the process of ransac
    Calls:          None
    Input:          The matrix to store the transformation result
    Output:         The transformation matrix
    Return:         None
    Others:         None
    *************************************************/

    void getRANSACTransformationMatrix(Eigen::Matrix4d& transform_matrix);

    /*************************************************
    Function:       showRANSANInformation
    Description:    Display the time cost in the process of ransac
    Calls:          None
    Input:          None
    Output:         Showing the information of cost time on console
    Return:         None
    Others:         None
    *************************************************/

    void showRANSANInformation();

    /*************************************************
    Function:       setParametersForICP
    Description:    Setup the parameters to apply a icp registration
    Calls:          None
    Input:          max_iteration: the maximum number of iterations
                    ransac_outlier_reject_threshold:
                    the inlier distance threshold; considers a point to be an inlier, if the distance between the target data
                    index and the transformed source index is smaller than the given inlier distance threshold, in millimeter
                    max_correspondence_distance: the maximum distance threshold between a point and its nearest neighbor, in millimeter
    Output:         None
    Return:         None
    Others:         None
    *************************************************/

    void setParametersForICP(int max_iteration, float ransac_outlier_reject_threshold, float max_correspondence_distance);

    /*************************************************
    Function:       alignUsingICP
    Description:    Apply icp on a point cloud to implement the registration to the target point cloud
    Calls:          None
    Input:          cloud_source: the source point cloud
                    cloud_target: the target point cloud
    Output:         A transformed point cloud
    Return:         None
    Others:         An ideal registration result requires a proper initial position of the source point cloud
    *************************************************/

    void alignUsingICP(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target);

    /*************************************************
    Function:       getICPTransformationMatrix
    Description:    Acquire the transformation matrix in the process of icp
    Calls:          None
    Input:          The matrix to store the transformation result
    Output:         The transformation matrix
    Return:         None
    Others:         None
    *************************************************/

    void getICPTransformationMatrix(Eigen::Matrix4d& transform_matrix);

    /*************************************************
    Function:       showICPInformation
    Description:    Display the time cost in the process of icp
    Calls:          None
    Input:          None
    Output:         Showing the information of cost time on console
    Return:         None
    Others:         None
    *************************************************/

    void showICPInformation();

    /*************************************************
    Function:       getFinalTransformationMatrix
    Description:    Acquire the transformation matrix between the referential object and the detected object
    Calls:          None
    Input:          The matrix to store the transformation information
    Output:         The transformation matrix
    Return:         None
    Others:         From the acquired transformation matrix and the known pose of the referential object, the pose of
                    the detected object can be inferred
    *************************************************/

    void getFinalTransformationMatrix(Eigen::Matrix4d& transform_matrix);

private:
    pcl::SampleConsensusInitialAlignment<PointT, PointT, DescriptorT> ransac;

    pcl::IterativeClosestPoint<PointT, PointT> icp;

    pcl::console::TicToc time;

    double ransac_time;

    double icp_time;
};

class PointCloudVisualizer
{
public:

    /*************************************************
    Function:       PointCloudVisualizer
    Description:    Constructor
    Calls:          None
    Input:          None
    Output:         None
    Return:         None
    Others:         Initialize the class member viewer
    *************************************************/

    PointCloudVisualizer();

    /*************************************************
    Function:       displaySimpleCloud
    Description:    Visualization of a single point cloud
    Calls:          None
    Input:          The point cloud to be displayed
    Output:         A visualization window showing the point cloud
    Return:         None
    Others:         For checking out what a point cloud looks like
    *************************************************/

    void displaySimpleCloud(PointCloudT::Ptr cloud);

    /*************************************************
    Function:       displayComparingClouds
    Description:    Visualization of two point clouds to be compared
    Calls:          None
    Input:          The two point clouds to be displayed
    Output:         A visualization window respectively showing two point clouds in two viewports
    Return:         None
    Others:         For checking out the differences between two point clouds or the changes of a point cloud
    *************************************************/

    void displayComparingClouds(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2);

    /*************************************************
    Function:       displayCloudsRegistration
    Description:    Visualization of two point clouds to be aligned
    Calls:          None
    Input:          original_source_cloud: the source point cloud before applying registration
                    target_cloud: the target point cloud
                    final_source_cloud: the source point cloud after applying registration
    Output:         A visualization window respectively showing two point clouds before and after the registration in two viewports
    Return:         None
    Others:         For checking out the registration process of two point clouds
    *************************************************/

    void displayCloudsRegistration(PointCloudT::Ptr original_source_cloud,
                                   PointCloudT::Ptr target_cloud, PointCloudT::Ptr final_source_cloud);

    /*************************************************
    Function:       displayCloudsMerge
    Description:    Visualization of the merge of point clouds from three different views
    Calls:          None
    Input:          The three point clouds to be merged
    Output:         A visualization window respectively showing the merged point cloud and the 3D reconstructed point cloud in two viewports
    Return:         None
    Others:         For checking out the result of the merge
    *************************************************/

    void displayCloudsMerge(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, PointCloudT::Ptr cloud3);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};

class PointCloudProcessor
{
public:

    /*************************************************
    Function:       upSampling
    Description:    Applying upsampling to a point cloud
    Calls:          None
    Input:          cloud: the point cloud to be upsampled
                    sampling_range: the sphere radius that is to be used for determining the k-nearest neighbors used for fitting, in millimeter
                    sampling_radius: the radius of the circle in the local point plane that will be sampled, in millimeter
                    sampling_interval: the step size for the local plane sampling, in millimeter
    Output:         The upsampled point cloud
    Return:         None
    Others:         Upsampling will make a point cloud denser
    *************************************************/

    void upSampling(PointCloudT::Ptr cloud, double sampling_range, double sampling_radius, double sampling_interval);

    /*************************************************
    Function:       downSampling
    Description:    Applying downsampling to a point cloud
    Calls:          None
    Input:          cloud: the point cloud to be downsampled
                    sampling_grid_x: the voxel grid leaf size for x, in millimeter
                    sampling_grid_y: the voxel grid leaf size for y, in millimeter
                    sampling_grid_z: the voxel grid leaf size for z, in millimeter
    Output:         The downsampled point cloud
    Return:         None
    Others:         Downsampling will make a point cloud sparser
    *************************************************/

    void downSampling(PointCloudT::Ptr cloud, float sampling_grid_x, float sampling_grid_y, float sampling_grid_z);

    /*************************************************
    Function:       removeRandomNoise
    Description:    Use point neighborhood statistics to filter outlier data
    Calls:          None
    Input:          cloud: the point cloud to be filtered
                    unit_size: the number of points to use for mean distance estimation
                    deviation_coefficient: the standard deviation multiplier
    Output:         The processed point cloud
    Return:         None
    Others:         For removing the background noise generated randomly
    *************************************************/

    void removeRandomNoise(PointCloudT::Ptr cloud, int unit_size, float deviation_coefficient);

    /*************************************************
    Function:       smoothing
    Description:    Smoothen a point cloud
    Calls:          None
    Input:          cloud: the point cloud to be smoothened
                    smooth_range: the sphere radius that is to be used for determining the k-nearest neighbors used for fitting, in millimeter
    Output:         The processed point cloud
    Return:         None
    Others:         None
    *************************************************/

    void smoothing(PointCloudT::Ptr cloud, double smooth_range);

private:
    pcl::search::KdTree<PointT>::Ptr kdtree;
};

class PointCloudSimulator
{
public:

    /*************************************************
    Function:       addGaussianNoiseToPoints
    Description:    Add artificial gaussian noise to the point cloud data
    Calls:          None
    Input:          cloud: the point cloud to be processed
                    mean: the mean value of the normal distribution used for generating gaussian noise
                    standard_deviation: the standard deviation of the normal distribution used for generating gaussian noise
    Output:         The point cloud with gaussian noise
    Return:         None
    Others:         For simulating the noise caused in the process of acquiring the point cloud data
    *************************************************/

    void addGaussianNoiseToPoints(PointCloudT::Ptr cloud, double mean, double standard_deviation);

    /*************************************************
    Function:       addGaussianNoiseToMerge
    Description:    Add artificial gaussian noise to the transformation matrix that is used for the merge of point clouds
    Calls:          None
    Input:          rotation_angle_x: the rotation angle around x-axis that is used for the merge of point clouds, in degree
                    rotation_angle_y: the rotation angle around y-axis that is used for the merge of point clouds, in degree
                    rotation_angle_z: the rotation angle around z-axis that is used for the merge of point clouds, in degree
                    translation_x: the translation distance along x-axis that is used for the merge of point clouds, in millimeter
                    translation_y: the translation distance along y-axis that is used for the merge of point clouds, in millimeter
                    translation_z: the translation distance along z-axis that is used for the merge of point clouds, in millimeter
                    mean: the mean value of the normal distribution used for generating gaussian noise
                    standard_deviation: the standard deviation of the normal distribution used for generating gaussian noise
                    seed: the time seed to generate the random number
    Output:         The transformation parameters with gaussian noise
    Return:         None
    Others:         For simulating the noise caused in the process of merging the point clouds
    *************************************************/

    void addGaussianNoiseToMerge(float& rotation_angle_x, float& rotation_angle_y,   float& rotation_angle_z,
                                 float& translation_x,    float& translation_y,      float& translation_z,
                                 double mean,             double standard_deviation, double seed);
};

    /*************************************************
    Function:       readRotationMatrixFromXml
    Description:    Read the rotation matrix from the xml file
    Calls:          None
    Input:          rotate_matrix: the matrix to store the content of file
                    name: the name of file
                    format: the format of file
                    path: the file path
    Output:         The rotation matrix read from file
    Return:         None
    Others:         The xml file is created using opencv
    *************************************************/

void readRotationMatrixFromXml(Eigen::Matrix3f& rotate_matrix,string name,string format,string path);

    /*************************************************
    Function:       readTranslationVectorFromXml
    Description:    Read the translation vector from the xml file
    Calls:          None
    Input:          translate_vector: the vector to store the content of file
                    name: the name of file
                    format: the format of file
                    path: the file path
    Output:         The translation vector read from file
    Return:         None
    Others:         The xml file is created using opencv
    *************************************************/

void readTranslationVectorFromXml(Eigen::Vector3f& translate_vector,string name,string format,string path);

#endif  //POINTCLOUDPOSEESTIMATION_POINTCLOUDTOOLBOX_H
