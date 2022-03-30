#pragma once

#include <pcl/visualization/pcl_visualizer.h>

void remove_outlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& p_point_cloud,
                    int nr_k,
                    double stddev_mult);

void show_point_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr p_point_cloud);

void show_rgb_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr p_rgb_point_cloud);
