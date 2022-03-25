#include <iostream>
#include <thread>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

int main(int argc, char **argv)
{
    if (argc == 1)
    {
        std::cout << argv[0] << " ply_file_path" << std::endl;
        return -1;
    }
    const char *str_box_file_path = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::io::loadPLYFile(str_box_file_path, *p_cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cuboid"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(p_cloud, "box");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "box");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}