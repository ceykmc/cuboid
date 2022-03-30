#include <iostream>
#include <thread>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include "utility.hpp"

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << argv[0] << " pcd_file_path" << std::endl;
        return -1;
    }
    const char *str_pcd_file_path = argv[1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr p_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader pcd_reader;
    int res = pcd_reader.read<pcl::PointXYZ>(str_pcd_file_path, *p_point_cloud);
    if (res != 0) {
        std::cerr << "can not read the pcd file: " << str_pcd_file_path << std::endl;
        return -1;
    }

    // step 1: filter by z value
    float z_top = 13000, z_bottom = 16000;
    pcl::PassThrough<pcl::PointXYZ> pass_filter;
    pass_filter.setInputCloud(p_point_cloud);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(z_top, z_bottom);
    pass_filter.filter(*p_point_cloud);

    // step 2: remove noize point
    remove_outlier(p_point_cloud, 50, 1.0);

    // step 3: use ransac to find the plane
    double distance_threshold = 10.;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setInputCloud(p_point_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "could not estimate a planar model for the given dataset" << std::endl;
        return -1;
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(p_point_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*p_point_cloud);

    // step 4: remove outlier again, with different parameters
    remove_outlier(p_point_cloud, 10, 10);

    // show corner points
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*p_point_cloud, min_pt, max_pt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_origin_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcd_reader.read<pcl::PointXYZ>(str_pcd_file_path, *p_origin_point_cloud);

    pcl::visualization::PCLVisualizer::Ptr p_viewer(
        new pcl::visualization::PCLVisualizer("viewer"));
    p_viewer->setBackgroundColor(0, 0, 0);
    p_viewer->addPointCloud<pcl::PointXYZ>(p_origin_point_cloud, "xyz point cloud");
    p_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "xyz point cloud");
    p_viewer->addCoordinateSystem(1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_key_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    p_key_point_cloud->points.push_back(min_pt);
    p_key_point_cloud->points.push_back(max_pt);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(
        p_key_point_cloud, 255, 0, 0);
    p_viewer->addPointCloud<pcl::PointXYZ>(p_key_point_cloud, keypoints_color_handler, "corner");
    p_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "corner");

    while (!p_viewer->wasStopped()) {
        p_viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
