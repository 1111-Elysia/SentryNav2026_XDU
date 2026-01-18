#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// --- 旋转函数（绕 roll, pitch, yaw） ---
void rotatePoint(pcl::PointXYZ& point, double roll_rad, double pitch_rad, double yaw_rad) {
    // Roll 绕 X
    double y1 = std::cos(roll_rad) * point.y - std::sin(roll_rad) * point.z;
    double z1 = std::sin(roll_rad) * point.y + std::cos(roll_rad) * point.z;
    point.y = y1;
    point.z = z1;

    // Pitch 绕 Y
    double x2 = std::cos(pitch_rad) * point.x + std::sin(pitch_rad) * point.z;
    double z2 = -std::sin(pitch_rad) * point.x + std::cos(pitch_rad) * point.z;
    point.x = x2;
    point.z = z2;

    // Yaw 绕 Z
    double x3 = std::cos(yaw_rad) * point.x - std::sin(yaw_rad) * point.y;
    double y3 = std::sin(yaw_rad) * point.x + std::cos(yaw_rad) * point.y;
    point.x = x3;
    point.y = y3;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入.pcd> <配置.json>" << std::endl;
        return -1;
    }

    std::string pcd_file = argv[1];
    std::string config_file = argv[2];

    // --- 读取 JSON 配置 ---
    json config;
    try {
        std::ifstream config_stream(config_file);
        if (!config_stream.is_open()) throw std::runtime_error("无法打开配置文件: " + config_file);
        config = json::parse(config_stream);
    } catch (const std::exception& e) {
        std::cerr << "JSON读取/解析失败: " << e.what() << std::endl;
        return -1;
    }

    // --- 参数 ---
    std::string output_prefix = config.at("output_prefix").get<std::string>();
    double resolution = config.at("resolution").get<double>();
    double min_z_height = config.at("min_z_height").get<double>();
    double max_z_height = config.at("max_z_height").get<double>();
    double voxel_leaf_size = config.at("voxel_leaf_size").get<double>();
    int occupied_value = config.value("occupied_value", 0);
    int free_value = config.value("free_value", 255);
    int unknown_value = config.value("unknown_value", 205);
    double occupied_thresh = config.at("occupied_thresh").get<double>();
    double free_thresh = config.at("free_thresh").get<double>();
    double map_padding = config.at("map_padding").get<double>();

    double roll_deg = config.value("lidar_roll_deg", 0.0);
    double pitch_deg = config.value("lidar_pitch_deg", 0.0);
    double yaw_deg = config.value("lidar_yaw_deg", 0.0);
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;

    std::string pgm_file = output_prefix + ".pgm";
    std::string yaml_file = output_prefix + ".yaml";

    // --- 1. 加载 PCD ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_raw) == -1) {
        std::cerr << "无法读取文件 " << pcd_file << std::endl;
        return -1;
    }
    std::cout << "原始点云大小: " << cloud_raw->size() << std::endl;
    if (cloud_raw->empty()) return -1;

    // --- 2. 雷达旋转补偿（先旋转点云） ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_rotated = *cloud_raw;
    for (auto& point : cloud_rotated->points) {
        rotatePoint(point, roll_rad, pitch_rad, yaw_rad);
    }
    std::cout << "已进行雷达三轴旋转补偿" << std::endl;

    // --- 3. 高度过滤（使用旋转后的点云 Z） ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_rotated);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(min_z_height, max_z_height);
    pass_z.filter(*cloud_filtered_z);
    std::cout << "高度过滤后: " << cloud_filtered_z->size() << std::endl;
    if (cloud_filtered_z->empty()) return -1;

    // --- 4. 体素下采样 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered_z);
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vg.filter(*cloud_downsampled);
    std::cout << "下采样后: " << cloud_downsampled->size() << std::endl;
    if (cloud_downsampled->empty()) return -1;

    // --- 5. 确定地图边界 ---
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_downsampled, min_pt, max_pt);
    double origin_x = min_pt.x - map_padding;
    double origin_y = min_pt.y - map_padding;
    double map_width_m = (max_pt.x + map_padding) - origin_x;
    double map_height_m = (max_pt.y + map_padding) - origin_y;
    int map_width_px = static_cast<int>(std::ceil(map_width_m / resolution));
    int map_height_px = static_cast<int>(std::ceil(map_height_m / resolution));
    if (map_width_px <=0 || map_height_px <=0) return -1;
    std::cout << "地图尺寸: " << map_width_px << " x " << map_height_px << std::endl;

    // --- 6. 初始化地图 ---
    std::vector<unsigned char> map_data(map_width_px * map_height_px, static_cast<unsigned char>(unknown_value));

    // --- 7. 栅格化点云 ---
    for (const auto& point : cloud_downsampled->points) {
        int px = static_cast<int>(std::floor((point.x - origin_x) / resolution));
        int py = static_cast<int>(std::floor((point.y - origin_y) / resolution));
        if (px >=0 && px < map_width_px && py >=0 && py < map_height_px) {
            int idx = px + (map_height_px - 1 - py) * map_width_px;
            if (idx >=0 && idx < map_data.size()) {
                map_data[idx] = static_cast<unsigned char>(occupied_value);
            }
        }
    }

    // --- 8. 输出 PGM ---
    std::ofstream pgm_out(pgm_file, std::ios::binary);
    if (!pgm_out) return -1;
    pgm_out << "P5\n" << map_width_px << " " << map_height_px << "\n255\n";
    pgm_out.write(reinterpret_cast<const char*>(map_data.data()), map_data.size());
    pgm_out.close();
    std::cout << "已保存 PGM: " << pgm_file << std::endl;

    // --- 9. 输出 YAML ---
    std::ofstream yaml_out(yaml_file);
    if (!yaml_out) return -1;
    std::filesystem::path pgm_path(pgm_file);
    std::string pgm_abs_path = std::filesystem::absolute(pgm_path).string();
    yaml_out << "image: " << pgm_abs_path << "\n";
    yaml_out << "mode: trinary\n";
    yaml_out << "resolution: " << resolution << "\n";
    yaml_out << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml_out << "negate: 0\n";
    yaml_out << "occupied_thresh: " << occupied_thresh << "\n";
    yaml_out << "free_thresh: " << free_thresh << "\n";
    yaml_out.close();
    std::cout << "已保存 YAML: " << yaml_file << std::endl;

    return 0;
}
