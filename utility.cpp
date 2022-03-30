#include <thread>

#include <pcl/filters/statistical_outlier_removal.h>

#include "utility.hpp"

void remove_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& p_point_cloud, int nr_k, double stddev_mult)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(p_point_cloud);
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*p_point_cloud);
}

void show_point_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr p_point_cloud)
{
    pcl::visualization::PCLVisualizer::Ptr p_viewer(
        new pcl::visualization::PCLVisualizer("viewer"));
    p_viewer->setBackgroundColor(0, 0, 0);
    p_viewer->addPointCloud<pcl::PointXYZ>(p_point_cloud, "xyz point cloud");
    p_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "xyz point cloud");
    p_viewer->addCoordinateSystem(1.0);

    while (!p_viewer->wasStopped()) {
        p_viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void show_rgb_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr p_rgb_point_cloud)
{
    pcl::visualization::PCLVisualizer::Ptr p_viewer(
        new pcl::visualization::PCLVisualizer("rgb viewer"));
    p_viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(p_rgb_point_cloud);
    p_viewer->addPointCloud<pcl::PointXYZRGB>(p_rgb_point_cloud, rgb, "rgb point cloud");
    p_viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rgb point cloud");
    p_viewer->addCoordinateSystem(1.0);
    p_viewer->initCameraParameters();

    while (!p_viewer->wasStopped()) {
        p_viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}