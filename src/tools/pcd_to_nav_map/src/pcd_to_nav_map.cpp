#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <filesystem>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// ------------------- 工具函数 -------------------
void rotatePoint(pcl::PointXYZ& point, double roll, double pitch, double yaw) {
    double y1 = cos(roll) * point.y - sin(roll) * point.z;
    double z1 = sin(roll) * point.y + cos(roll) * point.z;
    point.y = y1; point.z = z1;
    double x2 = cos(pitch) * point.x + sin(pitch) * point.z;
    double z2 = -sin(pitch) * point.x + cos(pitch) * point.z;
    point.x = x2; point.z = z2;
    double x3 = cos(yaw) * point.x - sin(yaw) * point.y;
    double y3 = sin(yaw) * point.x + cos(yaw) * point.y;
    point.x = x3; point.y = y3;
}

inline bool worldToMap(double wx, double wy, double origin_x, double origin_y,
                       double resolution, int width, int height, int& mx, int& my) {
    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return (mx >= 0 && mx < width && my >= 0 && my < height);
}

void bresenham(int x0, int y0, int x1, int y1, std::vector<unsigned char>& map,
               int width, int height, unsigned char free_val, unsigned char occ_val) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    while (true) {
        int idx = x + (height - 1 - y) * width;
        if (idx >= 0 && idx < static_cast<int>(map.size())) {
            // 不允许“自由空间射线”覆盖已确认的占据格，否则点多时会把障碍洗掉
            if (map[idx] != occ_val) map[idx] = free_val;
        }
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
}

// ------------------- 主程序 -------------------
int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "用法: " << argv[0] << " <PCD> <config.json>\n"; return -1; }
    std::string pcd_file = argv[1], config_file = argv[2];

    // --- 读取 JSON ---
    json config;
    std::ifstream f(config_file);
    config = json::parse(f);

    std::string output_prefix = config["output_prefix"];
    double resolution = config["resolution"];
    double min_z = config["min_z_height"];
    double max_z = config["max_z_height"];
    double voxel_size = config["voxel_leaf_size"];
    int occ_val = config.value("occupied_value", 0);
    int free_val = config.value("free_value", 255);
    int unknown_val = config.value("unknown_value", free_val);
    double occupied_thresh = config.value("occupied_thresh", 0.65);
    double free_thresh = config.value("free_thresh", 0.196);
    double map_pad = config["map_padding"];
    double roll = config.value("lidar_roll_deg", 0.0) * M_PI/180.0;
    double pitch = config.value("lidar_pitch_deg", 0.0) * M_PI/180.0;
    double yaw = config.value("lidar_yaw_deg", 0.0) * M_PI/180.0;
    double lidar_x = config.value("lidar_origin_x", 0.0);
    double lidar_y = config.value("lidar_origin_y", 0.0);

    std::string pgm_file = output_prefix + ".pgm";
    std::string yaml_file = output_prefix + ".yaml";

    // --- 加载点云 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(pcd_file, *cloud);
    if (cloud->empty()) return -1;

    // --- 旋转补偿 ---
    for (auto& pt : cloud->points) rotatePoint(pt, roll, pitch, yaw);

    // --- 高度分层 + 体素下采样 ---
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto& pt : cloud->points)
        if (pt.z >= min_z && pt.z <= max_z) cloud_filtered->push_back(pt);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(voxel_size, voxel_size, voxel_size);
    vg.filter(*cloud_voxel);
    if (cloud_voxel->empty()) return -1;

    // --- 地图边界 ---
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_voxel, min_pt, max_pt);
    double origin_x = min_pt.x - map_pad, origin_y = min_pt.y - map_pad;
    int width = static_cast<int>(std::ceil((max_pt.x + map_pad - origin_x)/resolution));
    int height = static_cast<int>(std::ceil((max_pt.y + map_pad - origin_y)/resolution));

    std::vector<unsigned char> map_data(width * height, static_cast<unsigned char>(unknown_val));

    // --- 雷达 origin 像素 ---
    int lx, ly;
    if (!worldToMap(lidar_x, lidar_y, origin_x, origin_y, resolution, width, height, lx, ly)) {
        // 防御性处理：若配置的雷达原点不在地图内，则将其夹紧到地图边界内
        lx = static_cast<int>(std::floor((lidar_x - origin_x) / resolution));
        ly = static_cast<int>(std::floor((lidar_y - origin_y) / resolution));
        lx = std::max(0, std::min(width - 1, lx));
        ly = std::max(0, std::min(height - 1, ly));
        std::cerr << "[WARN] lidar_origin 不在地图边界内，已夹紧到 (" << lx << "," << ly << ")\n";
    }

    // --- 投射 + 占据 ---
    for (auto& pt : cloud_voxel->points) {
        int px, py;
        if (!worldToMap(pt.x, pt.y, origin_x, origin_y, resolution, width, height, px, py)) continue;
        bresenham(lx, ly, px, py, map_data, width, height,
                  static_cast<unsigned char>(free_val), static_cast<unsigned char>(occ_val));
        int idx = px + (height-1 - py)*width;
        if (idx >=0 && idx < map_data.size()) map_data[idx] = occ_val;
    }

    // --- 输出 PGM ---
    std::ofstream pgm(pgm_file, std::ios::binary);
    pgm << "P5\n" << width << " " << height << "\n255\n";
    pgm.write(reinterpret_cast<char*>(map_data.data()), map_data.size());
    pgm.close();

    // --- 输出 YAML ---
    std::ofstream yaml(yaml_file);
    yaml << "image: " << std::filesystem::canonical(pgm_file).string() << "\n";
    yaml << "mode: trinary\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: " << occupied_thresh << "\n";
    yaml << "free_thresh: " << free_thresh << "\n";
    yaml.close();

    return 0;
}
