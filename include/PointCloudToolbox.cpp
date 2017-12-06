/***************************************************************************
Copyright: Huang Xiaohang / Piec
Author: Huang Xiaohang
Date: 2017-12-06
Description: Implementation of the classes and functions used in the project
****************************************************************************/

#include "PointCloudToolbox.h"


///  Implementation of Class PIPointCloud  ///

PIPointCloud::PIPointCloud() : cloud(new PointCloudT){}

void PIPointCloud::printTransformationMatrix ()
{
    printf ("Transformation information:\n");
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n",    transform_matrix (0, 0), transform_matrix (0, 1), transform_matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n",    transform_matrix (1, 0), transform_matrix (1, 1), transform_matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n",    transform_matrix (2, 0), transform_matrix (2, 1), transform_matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", transform_matrix (0, 3), transform_matrix (1, 3), transform_matrix (2, 3));
}

void PIPointCloud::pointCloudRotate(Eigen::Matrix3f rotate_matrix)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (0, 0) =  rotate_matrix (0, 0);
    matrix (0, 1) =  rotate_matrix (0, 1);
    matrix (0, 2) =  rotate_matrix (0, 2);
    matrix (1, 0) =  rotate_matrix (1, 0);
    matrix (1, 1) =  rotate_matrix (1, 1);
    matrix (1, 2) =  rotate_matrix (1, 2);
    matrix (2, 0) =  rotate_matrix (2, 0);
    matrix (2, 1) =  rotate_matrix (2, 1);
    matrix (2, 2) =  rotate_matrix (2, 2);
    pcl::transformPointCloud (*cloud, *cloud, matrix);
}

void PIPointCloud::pointCloudTranslate(Eigen::Vector3f translate_matrix)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (0, 3) =  translate_matrix (0, 0);
    matrix (1, 3) =  translate_matrix (0, 1);
    matrix (2, 3) =  translate_matrix (0, 2);
    pcl::transformPointCloud (*cloud, *cloud, matrix);
}

Eigen::Matrix4f PIPointCloud::pointCloudRotateX(float theta)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (1, 1) =  cos (theta * M_PI / 180);
    matrix (1, 2) = -sin (theta * M_PI / 180);
    matrix (2, 1) =  sin (theta * M_PI / 180);
    matrix (2, 2) =  cos (theta * M_PI / 180);
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

Eigen::Matrix4f PIPointCloud::pointCloudRotateY(float theta)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (0, 0) =  cos (theta * M_PI / 180);
    matrix (0, 2) =  sin (theta * M_PI / 180);
    matrix (2, 0) = -sin (theta * M_PI / 180);
    matrix (2, 2) =  cos (theta * M_PI / 180);
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

Eigen::Matrix4f PIPointCloud::pointCloudRotateZ(float theta)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (0, 0) =  cos (theta * M_PI / 180);
    matrix (0, 1) = -sin (theta * M_PI / 180);
    matrix (1, 0) =  sin (theta * M_PI / 180);
    matrix (1, 1) =  cos (theta * M_PI / 180);
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

Eigen::Matrix4f PIPointCloud::pointCloudTranslateX(float sigma)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (0, 3) = sigma;
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

Eigen::Matrix4f PIPointCloud::pointCloudTranslateY(float sigma)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (1, 3) = sigma;
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

Eigen::Matrix4f PIPointCloud::pointCloudTranslateZ(float sigma)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix (2, 3) = sigma;
    pcl::transformPointCloud (*cloud, *cloud, matrix);
    return matrix;
}

void PIPointCloud::pointCloudSlicingX(float starting_position, float ending_position)
{
    pcl::PassThrough<PointT> slicer;
    slicer.setInputCloud     (cloud);
    slicer.setFilterFieldName("x");
    slicer.setFilterLimits   (starting_position, ending_position);
    slicer.filter            (*cloud);
}

void PIPointCloud::pointCloudSlicingY(float starting_position, float ending_position)
{
    pcl::PassThrough<PointT> slicer;
    slicer.setInputCloud     (cloud);
    slicer.setFilterFieldName("y");
    slicer.setFilterLimits   (starting_position, ending_position);
    slicer.filter            (*cloud);
}

void PIPointCloud::pointCloudSlicingZ(float starting_position, float ending_position)
{
    pcl::PassThrough<PointT> slicer;
    slicer.setInputCloud     (cloud);
    slicer.setFilterFieldName("z");
    slicer.setFilterLimits   (starting_position, ending_position);
    slicer.filter            (*cloud);
}

FeaturesT::Ptr PIPointCloud::computeFeatures(float normal_search_range, float feature_search_range)
{
    Normals::Ptr point_cloud_normal       (new Normals ());
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    pcl::NormalEstimation<PointT, Normal> normal_estimator;
    normal_estimator.setRadiusSearch      (normal_search_range);
    normal_estimator.setInputCloud        (cloud);
    normal_estimator.compute              (*point_cloud_normal);

    pcl::FPFHEstimation<PointT, Normal, DescriptorT> fpfh_estimator;
    fpfh_estimator.setInputCloud             (cloud);
    fpfh_estimator.setInputNormals           (point_cloud_normal);
    fpfh_estimator.setSearchMethod           (tree);
    FeaturesT::Ptr features = FeaturesT::Ptr (new FeaturesT);
    fpfh_estimator.setRadiusSearch           (feature_search_range);
    fpfh_estimator.compute                   (*features);

    return features;
}


///  Implementation of Class PointCloudRegistrator  ///

void PointCloudRegistrator::setParametersForRANSAC(int max_iteration, float min_sample_distance, float max_correspondence_distance,
                                                   int sample_num, int correspondence_randomness, float ransac_outlier_reject_threshold)
{
    ransac.setMaximumIterations              (max_iteration);
    ransac.setMinSampleDistance              (min_sample_distance);
    ransac.setMaxCorrespondenceDistance      (max_correspondence_distance);
    ransac.setNumberOfSamples                (sample_num);
    ransac.setCorrespondenceRandomness       (correspondence_randomness);
    ransac.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold);
}

void PointCloudRegistrator::alignUsingRANSAC(PointCloudT::Ptr cloud_source, FeaturesT::Ptr features_source,
                                             PointCloudT::Ptr cloud_target, FeaturesT::Ptr features_target)
{
    ransac.setInputSource    (cloud_source);
    ransac.setSourceFeatures (features_source);
    ransac.setInputTarget    (cloud_target);
    ransac.setTargetFeatures (features_target);
    time.tic();
    ransac.align (*cloud_source);
    ransac_time = time.toc();
}

void PointCloudRegistrator::getRANSACTransformationMatrix(Eigen::Matrix4d& transform_matrix)
{
    transform_matrix = ransac.getFinalTransformation().cast<double>();
}

void PointCloudRegistrator::showRANSANInformation()
{
    cout << "Applied ransac initial alignment in " << ransac_time << " ms" << endl;
}

void PointCloudRegistrator::setParametersForICP(int max_iteration, float ransac_outlier_reject_threshold, float max_correspondence_distance)
{
    icp.setMaximumIterations              (max_iteration);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_reject_threshold);
    icp.setMaxCorrespondenceDistance      (max_correspondence_distance);
}

void PointCloudRegistrator::alignUsingICP(PointCloudT::Ptr cloud_source, PointCloudT::Ptr cloud_target)
{
    icp.setInputSource (cloud_source);
    icp.setInputTarget (cloud_target);
    time.tic();
    icp.align (*cloud_source);
    icp_time = time.toc();
}

void PointCloudRegistrator::getICPTransformationMatrix(Eigen::Matrix4d& transform_matrix)
{
    transform_matrix = icp.getFinalTransformation().cast<double>();
}

void PointCloudRegistrator::showICPInformation()
{
    cout << "Applied icp registration in " << icp_time << " ms" << endl;
}

void PointCloudRegistrator::getFinalTransformationMatrix(Eigen::Matrix4d& transform_matrix)
{
    transform_matrix = icp.getFinalTransformation().cast<double>() * ransac.getFinalTransformation().cast<double>();
}


///  Implementation of Class PointCloudVisualizer  ///

PointCloudVisualizer::PointCloudVisualizer() : viewer(new pcl::visualization::PCLVisualizer){}

void PointCloudVisualizer::displaySimpleCloud(PointCloudT::Ptr cloud)
{
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h1 (cloud, 120, 0, 120);
    viewer->addPointCloud<PointT>(cloud, cloud_color_h1, "cloud");
    viewer->addCoordinateSystem(5);
    viewer->spin();
}

void PointCloudVisualizer::displayComparingClouds(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2)
{
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h1  (cloud1,  120, 0, 120);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h2  (cloud2,  0, 120, 120);
    viewer->addPointCloud<PointT>(cloud1,  cloud_color_h1, "cloud1", v1);
    viewer->addPointCloud<PointT>(cloud2, cloud_color_h2,  "cloud2", v2);
    viewer->addCoordinateSystem(5);
    viewer->spin();
}

void PointCloudVisualizer::displayCloudsRegistration(PointCloudT::Ptr original_source_cloud,
                               PointCloudT::Ptr target_cloud, PointCloudT::Ptr final_source_cloud)
{
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color_h1 (original_source_cloud,  120, 0, 120);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> src_color_h2 (final_source_cloud,     120, 120, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_color_h  (target_cloud,           0, 120, 120);
    viewer->addPointCloud<PointT>(original_source_cloud,  src_color_h1, "origin_src", v1);
    viewer->addPointCloud<PointT>(target_cloud,           tgt_color_h,  "tgt_v1",     v1);
    viewer->addPointCloud<PointT>(final_source_cloud,     src_color_h2, "final_src",  v2);
    viewer->addPointCloud<PointT>(target_cloud,           tgt_color_h,  "tgt_v2",     v2);
    viewer->addCoordinateSystem(5);
    viewer->spin();
}

void PointCloudVisualizer::displayCloudsMerge(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, PointCloudT::Ptr cloud3)
{
    int v1(0);
    int v2(1);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0,0,0);
    PointCloudT::Ptr cloud_merge(new PointCloudT);
    *cloud_merge  = *cloud1 + *cloud2;
    *cloud_merge += *cloud3;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud1_color_h (cloud1,       120, 0, 120);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud2_color_h (cloud2,       120, 120, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud3_color_h (cloud3,       0, 120, 120);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> merge_color_h  (cloud_merge,  120, 90, 60);
    viewer->addPointCloud<PointT>(cloud1,       cloud1_color_h,  "cloud1",  v1);
    viewer->addPointCloud<PointT>(cloud2,       cloud2_color_h,  "cloud2",  v1);
    viewer->addPointCloud<PointT>(cloud3,       cloud3_color_h,  "cloud3",  v1);
    viewer->addPointCloud<PointT>(cloud_merge,  merge_color_h,   "merge",   v2);
    viewer->addCoordinateSystem(5);
    viewer->spin();
}


///  Implementation of Class PointCloudProcessor  ///

void PointCloudProcessor::upSampling(PointCloudT::Ptr cloud, double sampling_range, double sampling_radius, double sampling_interval)
{
    pcl::MovingLeastSquares<PointT, PointT> upsampler;
    upsampler.setInputCloud    (cloud);
    upsampler.setSearchRadius  (sampling_range);
    upsampler.setPolynomialFit (true);
    upsampler.setComputeNormals(true);
    upsampler.setSearchMethod  (kdtree);

    upsampler.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
    ///  other methods: DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY  ///

    upsampler.setUpsamplingRadius  (sampling_radius);
    upsampler.setUpsamplingStepSize(sampling_interval);
    upsampler.process              (*cloud);
}

void PointCloudProcessor::downSampling(PointCloudT::Ptr cloud, float sampling_grid_x, float sampling_grid_y, float sampling_grid_z)
{
     pcl::VoxelGrid<pcl::PointXYZ> downsampler;
     downsampler.setInputCloud(cloud);
     downsampler.setLeafSize  (sampling_grid_x, sampling_grid_y, sampling_grid_z);
     downsampler.filter       (*cloud);
}

void PointCloudProcessor::removeRandomNoise(PointCloudT::Ptr cloud, int unit_size, float deviation_coefficient)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> denoiser;
    denoiser.setInputCloud      (cloud);
    denoiser.setMeanK           (unit_size);
    denoiser.setStddevMulThresh (deviation_coefficient);
    denoiser.filter             (*cloud);
}

void PointCloudProcessor::smoothing(PointCloudT::Ptr cloud, double smooth_range)
{
    PointCloudN cloud_smooth;
    pcl::MovingLeastSquares<PointT, PointN> smoother;
    smoother.setComputeNormals (true);
    smoother.setInputCloud     (cloud);
    smoother.setPolynomialFit  (true);
    smoother.setSearchMethod   (kdtree);
    smoother.setSearchRadius   (smooth_range);
    smoother.process           (cloud_smooth);
    pcl::copyPointCloud        (cloud_smooth, *cloud);
}


///  Implementation of Class PointCloudSimulator  ///

void PointCloudSimulator::addGaussianNoiseToPoints(PointCloudT::Ptr cloud, double mean, double standard_deviation)
{
    boost::mt19937 rng;
    rng.seed (static_cast<unsigned int> (time (NULL)));
    boost::normal_distribution<> nd     (mean, standard_deviation);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t i = 0; i < cloud->points.size (); i++)
    {
        cloud->points[i].x = cloud->points[i].x + static_cast<float> (var_nor());
        cloud->points[i].y = cloud->points[i].y + static_cast<float> (var_nor());
        cloud->points[i].z = cloud->points[i].z + static_cast<float> (var_nor());
    }
}

void PointCloudSimulator::addGaussianNoiseToMerge(float& rotation_angle_x, float& rotation_angle_y,   float& rotation_angle_z,
                             float& translation_x,    float& translation_y,      float& translation_z,
                             double mean,             double standard_deviation, double seed)
{
    boost::mt19937 rng;
    rng.seed (static_cast<unsigned int> (seed));
    boost::normal_distribution<> nd (mean, standard_deviation);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
    rotation_angle_x += static_cast<float> (var_nor());
    rotation_angle_y += static_cast<float> (var_nor());
    rotation_angle_z += static_cast<float> (var_nor());
    translation_x    += static_cast<float> (var_nor());
    translation_y    += static_cast<float> (var_nor());
    translation_z    += static_cast<float> (var_nor());
}


///  Implementation of Other Functions  ///

void readRotationMatrixFromXml(Eigen::Matrix3f& rotate_matrix, string name, string format, string path)
{
    string str = path + name + format;
    FileStorage file(str, FileStorage::READ);
    if(!file.isOpened())
        cout<<"Open File Error!"<<endl;
    Mat Matrix;
    file[name]>>Matrix;
    cv2eigen(Matrix, rotate_matrix);
    file.release();
}

void readTranslationVectorFromXml(Eigen::Vector3f& translate_vector, string name, string format, string path)
{
    string str = path + name + format;
    FileStorage file(str, FileStorage::READ);
    if(!file.isOpened())
        cout<<"Open File Error!"<<endl;
    Mat Matrix;
    file[name]>>Matrix;
    cv2eigen(Matrix, translate_vector);
    file.release();
}
