/*********************************************************************
Copyright: Huang Xiaohang / Piec
Author: Huang Xiaohang
Date: 2017-12-06
Description: Point cloud 3D reconstruction and object pose estimation
**********************************************************************/

#include <iostream>
#include "PointCloudPoseEstimation.h"


int main()
{
    pcl::io::loadPCDFile(cam1_data_filepath,  *cloud_cam1.cloud);
    pcl::io::loadPCDFile(cam2_data_filepath,  *cloud_cam2.cloud);
    pcl::io::loadPCDFile(cam3_data_filepath,  *cloud_cam3.cloud);
    pcl::io::loadPCDFile(model_data_filepath, *cloud_model.cloud);
    pcl::io::loadPCDFile(model_data_filepath, *cloud_reference.cloud);

    ///  read the xml files to acquire the geometric relationship between cameras  ///

    readRotationMatrixFromXml   (rotation_matrix_2_to_1,    filename_rotate_2_to_1,    file_format, file_path);
    readRotationMatrixFromXml   (rotation_matrix_3_to_1,    filename_rotate_3_to_1,    file_format, file_path);
    readTranslationVectorFromXml(translation_vector_2_to_1, filename_translate_2_to_1, file_format, file_path);
    readTranslationVectorFromXml(translation_vector_3_to_1, filename_translate_3_to_1, file_format, file_path);

    ///  point clouds merge  ///

    cloud_cam2.pointCloudRotate(rotation_matrix_2_to_1);
    cloud_cam3.pointCloudRotate(rotation_matrix_3_to_1);

//    cloud_cam2.pointCloudTranslate(translation_vector_2_to_1);
//    cloud_cam3.pointCloudTranslate(translation_vector_3_to_1);

    *cloud_merged.cloud  = *cloud_cam1.cloud + *cloud_cam2.cloud;
    *cloud_merged.cloud += *cloud_cam3.cloud;


    FeaturesT::Ptr features_merged = cloud_merged.computeFeatures(10, 1);
    FeaturesT::Ptr features_model  = cloud_model.computeFeatures(10, 1);

    PointCloudRegistrator registrator;

    ///  set parameters for ransac  ///

    int   max_iteration_ransac               = 2;
    float min_sample_distance                = 3;
    float max_correspondence_distance_ransac = 3;
    int   sample_num                         = 20;
    int   correspondence_randomness          = 1;
    float ransac_outlier_reject_threshold    = 5e-5;

    ///  apply ransac to acquire an initial position  ///

    registrator.setParametersForRANSAC(max_iteration_ransac, min_sample_distance,        max_correspondence_distance_ransac,
                                       sample_num,           correspondence_randomness,  ransac_outlier_reject_threshold);
    registrator.alignUsingRANSAC(cloud_model.cloud, features_model, cloud_merged.cloud, features_merged);
    registrator.showRANSANInformation();
    registrator.getRANSACTransformationMatrix(cloud_model.transform_matrix);
    cout<<"ransac:"<<endl;
    cloud_model.printTransformationMatrix();

    ///  set parameters for icp  ///

    int   max_iteration_icp               = 70;
    float icp_outlier_reject_threshold    = 5e-4;
    float max_correspondence_distance_icp = 20;

    ///  apply icp for registration  ///

    registrator.setParametersForICP(max_iteration_icp, icp_outlier_reject_threshold, max_correspondence_distance_icp);
    registrator.alignUsingICP(cloud_model.cloud, cloud_merged.cloud);
    registrator.showICPInformation();
    registrator.getICPTransformationMatrix(cloud_model.transform_matrix);
    cout<<"icp:"<<endl;
    cloud_model.printTransformationMatrix();

    ///  get the pose of the object  ///

    registrator.getFinalTransformationMatrix(cloud_model.transform_matrix);
    cout<<"pose of the object:"<<endl;
    cloud_model.printTransformationMatrix();

    ///  visualization  ///

    PointCloudVisualizer viewer;
    viewer.displayCloudsRegistration(cloud_reference.cloud, cloud_merged.cloud, cloud_model.cloud);

    return (0);
}