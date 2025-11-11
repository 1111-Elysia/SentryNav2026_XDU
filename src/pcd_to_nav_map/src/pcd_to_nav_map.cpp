#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits> // Required for std::numeric_limits
#include <stdexcept> // For exception handling

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> // For pcl::getMinMax3D
#include <pcl/filters/passthrough.h> // 用于高度过滤
#include <pcl/filters/voxel_grid.h> // 用于体素下采样

#include <nlohmann/json.hpp> // JSON库头文件

// 使用 nlohmann::json 命名空间
using json = nlohmann::json;

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "用法: " << argv[0] << " <输入.pcd文件> <配置文件.json>" << std::endl;
        std::cerr << "示例: " << argv[0] << " my_cloud.pcd config.json" << std::endl;
        return -1;
    }
    std::string pcd_file = argv[1];
    std::string config_file = argv[2];

    // --- 从JSON文件读取配置 ---
    json config;
    std::string output_prefix;
    double resolution;
    double min_z_height;
    double max_z_height;
    double voxel_leaf_size;
    int occupied_value;
    int free_value;
    int unknown_value;
    double occupied_thresh;
    double free_thresh;
    double map_padding;

    try {
        std::ifstream config_stream(config_file);
        if (!config_stream.is_open()) {
            throw std::runtime_error("无法打开配置文件: " + config_file);
        }
        config = json::parse(config_stream);

        output_prefix = config.at("output_prefix").get<std::string>();
        resolution = config.at("resolution").get<double>();
        min_z_height = config.at("min_z_height").get<double>();
        max_z_height = config.at("max_z_height").get<double>();
        voxel_leaf_size = config.at("voxel_leaf_size").get<double>();
        occupied_value = config.value("occupied_value", 0);
        free_value = config.value("free_value", 255);
        unknown_value = config.value("unknown_value", 205);
        occupied_thresh = config.at("occupied_thresh").get<double>();
        free_thresh = config.at("free_thresh").get<double>();
        map_padding = config.at("map_padding").get<double>();

        std::cout << "从 " << config_file << " 加载配置成功。" << std::endl;

    } catch (const json::parse_error& e) {
        std::cerr << "JSON解析错误: " << e.what() << std::endl;
        return -1;
    } catch (const json::exception& e) {
        std::cerr << "JSON配置错误: " << e.what() << std::endl;
        return -1;
    } catch (const std::runtime_error& e) {
        std::cerr << "运行时错误: " << e.what() << std::endl;
        return -1;
    }

    std::string pgm_file = output_prefix + ".pgm";
    std::string yaml_file = output_prefix + ".yaml";

    // 1. 加载PCD文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_raw) == -1) {
        std::cerr << "无法读取文件 " << pcd_file << std::endl;
        return -1;
    }
    std::cout << "从 " << pcd_file << " 加载了 " << cloud_raw->width * cloud_raw->height << " 个原始数据点" << std::endl;

    if (cloud_raw->empty()) {
         std::cerr << "错误：输入的点云为空！" << std::endl;
         return -1;
    }

    // 2. 高度过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_raw);
    pass_z.setFilterFieldName("z");
    // 设置过滤范围为 [min_z_height, max_z_height]
    pass_z.setFilterLimits(min_z_height, max_z_height);
    // pass_z.setFilterLimitsNegative(true); // 如果想移除此范围内的点，取消注释
    pass_z.filter(*cloud_filtered_z);
    // 更新输出信息以反映新的范围
    std::cout << "高度过滤后剩余 " << cloud_filtered_z->size() << " 个点 ("
              << min_z_height << " <= Z <= " << max_z_height << " 米)" << std::endl;

    if (cloud_filtered_z->empty()) {
         std::cerr << "错误：高度过滤后点云为空！请检查 Z 轴过滤范围和输入点云。" << std::endl;
         return -1;
    }

    // 3. 体素下采样
    // ... (后续代码保持不变) ...
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered_z);
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size); // 使用从JSON读取的值
    vg.filter(*cloud_downsampled);
    std::cout << "体素下采样后剩余 " << cloud_downsampled->size() << " 个点 (叶子大小: " << voxel_leaf_size << " 米)" << std::endl;

     if (cloud_downsampled->empty()) {
         std::cerr << "错误：下采样后点云为空！" << std::endl;
         return -1;
    }

    // 4. 确定地图边界和原点 (使用下采样后的点云)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud_downsampled, min_pt, max_pt);

    // 计算地图的原点和尺寸（以米为单位）
    double origin_x = min_pt.x - map_padding; // 使用从JSON读取的值
    double origin_y = min_pt.y - map_padding; // 使用从JSON读取的值
    double map_width_meters = (max_pt.x + map_padding) - origin_x;
    double map_height_meters = (max_pt.y + map_padding) - origin_y;

    // 计算地图的像素尺寸
    int map_width_pixels = static_cast<int>(std::ceil(map_width_meters / resolution)); // 使用从JSON读取的值
    int map_height_pixels = static_cast<int>(std::ceil(map_height_meters / resolution)); // 使用从JSON读取的值

    if (map_width_pixels <= 0 || map_height_pixels <= 0) {
        std::cerr << "错误：计算出的地图尺寸无效 (" << map_width_pixels << "x" << map_height_pixels << ")" << std::endl;
        return -1;
    }


    std::cout << "地图尺寸: " << map_width_pixels << "x" << map_height_pixels << " 像素" << std::endl;
    std::cout << "地图原点: [" << origin_x << ", " << origin_y << "] 米" << std::endl;

    // 5. 创建栅格地图数据结构 (初始化为未知)
    std::vector<unsigned char> map_data(map_width_pixels * map_height_pixels, static_cast<unsigned char>(unknown_value)); // 使用从JSON读取的值

    // 6. 栅格化点云 (使用下采样后的点云)
    for (const auto& point : cloud_downsampled->points) {
        // 将世界坐标转换为像素坐标
        int pixel_x = static_cast<int>(std::floor((point.x - origin_x) / resolution)); // 使用从JSON读取的值
        int pixel_y = static_cast<int>(std::floor((point.y - origin_y) / resolution)); // 使用从JSON读取的值

        // 检查坐标是否在地图边界内
        if (pixel_x >= 0 && pixel_x < map_width_pixels && pixel_y >= 0 && pixel_y < map_height_pixels) {
            int map_index = pixel_x + (map_height_pixels - 1 - pixel_y) * map_width_pixels;
             if (map_index >= 0 && map_index < map_data.size()) {
                 map_data[map_index] = static_cast<unsigned char>(occupied_value); // 使用从JSON读取的值
             } else {
                 std::cerr << "警告：计算出的地图索引无效: " << map_index
                           << " (像素: " << pixel_x << "," << pixel_y << ")" << std::endl;
             }
        }
    }

    // 7. 生成 PGM 文件
    std::ofstream pgm_out(pgm_file, std::ios::binary);
    if (!pgm_out) {
        std::cerr << "无法打开 " << pgm_file << " 进行写入。" << std::endl;
        return -1;
    }
    pgm_out << "P5\n";
    pgm_out << map_width_pixels << " " << map_height_pixels << "\n";
    pgm_out << "255\n"; // PGM 最大灰度值通常是 255，即使 free_value 不同
    pgm_out.write(reinterpret_cast<const char*>(map_data.data()), map_data.size());
    pgm_out.close();
    std::cout << "已将 PGM 地图保存到 " << pgm_file << std::endl;


    // 8. 生成 YAML 文件
    std::ofstream yaml_out(yaml_file);
     if (!yaml_out) {
        std::cerr << "无法打开 " << yaml_file << " 进行写入。" << std::endl;
        return -1;
    }
    yaml_out << "image: " << pgm_file << "\n";
    yaml_out << "mode: trinary\n";
    yaml_out << "resolution: " << resolution << "\n"; // 使用从JSON读取的值
    yaml_out << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n";
    yaml_out << "negate: 0\n";
    yaml_out << "occupied_thresh: " << occupied_thresh << "\n"; // 使用从JSON读取的值
    yaml_out << "free_thresh: " << free_thresh << "\n"; // 使用从JSON读取的值
    yaml_out.close();
    std::cout << "已将 YAML 元数据保存到 " << yaml_file << std::endl;

    return 0;
}
