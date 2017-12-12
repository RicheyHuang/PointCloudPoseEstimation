/************************************************************
Copyright: Huang Xiaohang / Piec
Author: Huang Xiaohang
Date: 2017-12-06
Description: Declaration of the variables used in the project
*************************************************************/

#ifndef POINTCLOUDPOSEESTIMATION_POINTCLOUDPOSEESTIMATION_H
#define POINTCLOUDPOSEESTIMATION_POINTCLOUDPOSEESTIMATION_H

#include "PointCloudToolbox.h"

PIPointCloud cloud_cam1;
PIPointCloud cloud_cam2;
PIPointCloud cloud_cam3;
PIPointCloud cloud_model;
PIPointCloud cloud_reference;
PIPointCloud cloud_merged;

string cam1_data_filepath  = "..//data//fraction1.pcd";
string cam2_data_filepath  = "..//data//fraction2.pcd";
string cam3_data_filepath  = "..//data//fraction3.pcd";
string model_data_filepath = "..//data//model.pcd";

string filepath_rotate_2_to_1    = "..//data//Cam21RotationMatrix.xml";
string filepath_rotate_3_to_1    = "..//data//Cam31RotationMatrix.xml";
string filepath_translate_2_to_1 = "..//data//Cam21TranslationMatrix.xml";
string filepath_translate_3_to_1 = "..//data//Cam31TranslationMatrix.xml";

Eigen::Matrix3f rotation_matrix_2_to_1 = Eigen::Matrix3f::Identity();
Eigen::Matrix3f rotation_matrix_3_to_1 = Eigen::Matrix3f::Identity();


Eigen::Vector3f translation_vector_2_to_1 = Eigen::Vector3f::Identity();
Eigen::Vector3f translation_vector_3_to_1 = Eigen::Vector3f::Identity();


#endif //POINTCLOUDPOSEESTIMATION_POINTCLOUDPOSEESTIMATION_H
