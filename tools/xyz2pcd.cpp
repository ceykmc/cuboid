#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void string_split(
    const std::string &s,
    std::vector<std::string> &tokens,
    const std::string &delimiters = "")
{
    std::string::size_type last_pos = s.find_first_not_of(delimiters, 0);
    std::string::size_type pos = s.find_first_of(delimiters, last_pos);
    while (std::string::npos != pos || std::string::npos != last_pos)
    {
        tokens.push_back(s.substr(last_pos, pos - last_pos));
        last_pos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, last_pos);
    }
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cout << argv[0] << "input_xyz_file_path output_pcd_file_path" << std::endl;
        return -1;
    }
    std::string str_xyz_file_path = argv[1];
    std::string str_pcd_file_path = argv[2];

    std::ifstream in_file(str_xyz_file_path.c_str());
    if (!in_file.is_open())
    {
        std::cerr << "can not open the file: " << str_xyz_file_path << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::string str_line_content;
    while (std::getline(in_file, str_line_content, '\n'))
    {
        std::vector<std::string> tokens;
        std::string delimiters = ",";

        string_split(str_line_content, tokens, delimiters);
        assert(tokens.size() == 3 && "tokens size is not 3");

        pcl::PointXYZ point;
        point.x = stof(tokens[0]);
        point.y = stof(tokens[1]);
        point.z = stof(tokens[2]);
        cloud.points.push_back(point);
    }
    in_file.close();

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cloud.is_dense = false;

    pcl::io::savePCDFileASCII(str_pcd_file_path.c_str(), cloud);

    return 0;
}
