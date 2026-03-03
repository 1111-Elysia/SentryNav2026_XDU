#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <filesystem>
#include <cmath>
#include <string>

class InitPosChecker : public rclcpp::Node
{
public:
    InitPosChecker() : Node("init_pos_checker"), checked_(false), frame_count_(0)
    {
        this->declare_parameter<std::string>("map_pcd_path", "");
        this->declare_parameter<std::string>("cloud_topic", "/lio/cloud_world");
        this->declare_parameter<int>("accumulate_frames", 5);
        this->declare_parameter<double>("voxel_size", 0.3);
        this->declare_parameter<int>("icp_max_iter", 50);
        this->declare_parameter<double>("pos_threshold", 0.5);
        this->declare_parameter<double>("rot_threshold", 5.0);

        map_pcd_path_      = this->get_parameter("map_pcd_path").as_string();
        cloud_topic_       = this->get_parameter("cloud_topic").as_string();
        accumulate_frames_ = this->get_parameter("accumulate_frames").as_int();
        voxel_size_        = this->get_parameter("voxel_size").as_double();
        icp_max_iter_      = this->get_parameter("icp_max_iter").as_int();
        pos_threshold_     = this->get_parameter("pos_threshold").as_double();
        rot_threshold_     = this->get_parameter("rot_threshold").as_double();

        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        accum_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // 加载地图
        if (!loadMapPCD()) {
            RCLCPP_ERROR(this->get_logger(), "地图加载失败，节点退出");
            rclcpp::shutdown();
            return;
        }

        // 发布者
        map_pub_     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/map_cloud", 1);
        current_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/current_cloud", 1);
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/aligned_cloud", 1);

        // 订阅者
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, 10,
            std::bind(&InitPosChecker::cloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  初始位姿检查器已启动");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  点云话题:  %s", cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  地图路径:  %s", map_pcd_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  地图点数:  %zu", map_cloud_->points.size());
        RCLCPP_INFO(this->get_logger(), "  累积帧数:  %d", accumulate_frames_);
        RCLCPP_INFO(this->get_logger(), "  位置阈值:  %.2f m", pos_threshold_);
        RCLCPP_INFO(this->get_logger(), "  旋转阈值:  %.2f°", rot_threshold_);
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "原理:");
        RCLCPP_INFO(this->get_logger(), "  %s 已在 map 坐标系下", cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  直接与地图 PCD 做 ICP，修正量 = 定位误差");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "等待点云数据...");
    }

private:
    bool loadMapPCD()
    {
        if (map_pcd_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "✗ 未指定 map_pcd_path 参数！");
            return false;
        }

        if (!std::filesystem::exists(map_pcd_path_)) {
            RCLCPP_ERROR(this->get_logger(), "✗ 路径不存在: %s", map_pcd_path_.c_str());
            return false;
        }

        // 支持目录（多个 PCD 拼接）或单个文件
        if (std::filesystem::is_directory(map_pcd_path_)) {
            int file_count = 0;
            for (auto& entry : std::filesystem::directory_iterator(map_pcd_path_)) {
                if (entry.path().extension() == ".pcd") {
                    pcl::PointCloud<pcl::PointXYZ> tmp;
                    if (pcl::io::loadPCDFile(entry.path().string(), tmp) == 0) {
                        *map_cloud_ += tmp;
                        file_count++;
                        RCLCPP_INFO(this->get_logger(), "  加载: %s (%zu 点)",
                            entry.path().filename().c_str(), tmp.points.size());
                    }
                }
            }
            if (file_count == 0) {
                RCLCPP_ERROR(this->get_logger(), "✗ 目录中没有 .pcd 文件: %s", map_pcd_path_.c_str());
                return false;
            }
        } else {
            if (pcl::io::loadPCDFile(map_pcd_path_, *map_cloud_) != 0) {
                RCLCPP_ERROR(this->get_logger(), "✗ 无法加载: %s", map_pcd_path_.c_str());
                return false;
            }
        }

        RCLCPP_INFO(this->get_logger(), "地图原始点数: %zu", map_cloud_->points.size());

        // 降采样
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(map_cloud_);
        vox.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*filtered);
        map_cloud_ = filtered;

        RCLCPP_INFO(this->get_logger(), "✓ 地图加载完成: %zu 点（降采样后）", map_cloud_->points.size());
        return true;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (checked_) return;

        pcl::PointCloud<pcl::PointXYZ> frame;
        pcl::fromROSMsg(*msg, frame);
        *accum_cloud_ += frame;
        frame_count_++;

        RCLCPP_INFO(this->get_logger(), "累积点云: %d/%d", frame_count_, accumulate_frames_);

        if (frame_count_ < accumulate_frames_) return;

        // 降采样当前点云
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(accum_cloud_);
        vox.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        vox.filter(*filtered);

        RCLCPP_INFO(this->get_logger(), "当前点云: %zu 点（降采样后）", filtered->points.size());

        // ========== ICP 对齐 ==========
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "  ICP 对齐验证");
        RCLCPP_INFO(this->get_logger(), "========================================");

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(filtered);
        icp.setInputTarget(map_cloud_);
        icp.setMaximumIterations(icp_max_iter_);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-4);
        icp.setMaxCorrespondenceDistance(2.0);

        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        bool converged = icp.hasConverged();
        double fitness = icp.getFitnessScore();
        Eigen::Matrix4f T = icp.getFinalTransformation();

        Eigen::Vector3f trans = T.block<3, 1>(0, 3);
        Eigen::Matrix3f rot = T.block<3, 3>(0, 0);

        float sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
        float roll  = std::atan2(rot(2, 1), rot(2, 2));
        float pitch = std::atan2(-rot(2, 0), sy);
        float yaw   = std::atan2(rot(1, 0), rot(0, 0));

        float pos_err = trans.norm();
        float rot_err = std::sqrt(roll * roll + pitch * pitch + yaw * yaw) * 180.0f / M_PI;

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "收敛:       %s", converged ? "✓ 是" : "✗ 否");
        RCLCPP_INFO(this->get_logger(), "适应度:     %.6f", fitness);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "位置修正:");
        RCLCPP_INFO(this->get_logger(), "  dx = %.4f m", trans.x());
        RCLCPP_INFO(this->get_logger(), "  dy = %.4f m", trans.y());
        RCLCPP_INFO(this->get_logger(), "  dz = %.4f m", trans.z());
        RCLCPP_INFO(this->get_logger(), "  总计 = %.4f m", pos_err);
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "旋转修正:");
        RCLCPP_INFO(this->get_logger(), "  roll  = %.3f°", roll * 180.0f / M_PI);
        RCLCPP_INFO(this->get_logger(), "  pitch = %.3f°", pitch * 180.0f / M_PI);
        RCLCPP_INFO(this->get_logger(), "  yaw   = %.3f°", yaw * 180.0f / M_PI);
        RCLCPP_INFO(this->get_logger(), "  总计  = %.3f°", rot_err);
        RCLCPP_INFO(this->get_logger(), "");

        // 判定
        bool pos_ok = pos_err < pos_threshold_;
        bool rot_ok = rot_err < rot_threshold_;

        RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        if (pos_ok && rot_ok) {
            RCLCPP_INFO(this->get_logger(), "✅ 定位准确！点云与地图对齐良好");
            RCLCPP_INFO(this->get_logger(), "   位置误差 %.4f m < 阈值 %.2f m", pos_err, pos_threshold_);
            RCLCPP_INFO(this->get_logger(), "   旋转误差 %.3f° < 阈值 %.2f°", rot_err, rot_threshold_);
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ 定位存在偏差！");
            if (!pos_ok)
                RCLCPP_WARN(this->get_logger(), "   位置: %.4f m > 阈值 %.2f m", pos_err, pos_threshold_);
            if (!rot_ok)
                RCLCPP_WARN(this->get_logger(), "   旋转: %.3f° > 阈值 %.2f°", rot_err, rot_threshold_);
            RCLCPP_WARN(this->get_logger(), "");
            RCLCPP_WARN(this->get_logger(), "   可能原因:");
            RCLCPP_WARN(this->get_logger(), "   1. loc_start_pose.yaml 中的初始位姿不准确");
            RCLCPP_WARN(this->get_logger(), "   2. lidar.yaml 中的外参不正确");
            RCLCPP_WARN(this->get_logger(), "   3. 地图与当前环境不匹配");
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        // 发布到 RViz
        publishDebug(msg->header.stamp, filtered, aligned);

        checked_ = true;
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "检查完成。Ctrl+C 退出。");
    }

    void publishDebug(const rclcpp::Time& stamp,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& current,
                      const pcl::PointCloud<pcl::PointXYZ>& aligned)
    {
        sensor_msgs::msg::PointCloud2 out;

        pcl::toROSMsg(*map_cloud_, out);
        out.header.frame_id = "map";
        out.header.stamp = stamp;
        map_pub_->publish(out);

        pcl::toROSMsg(*current, out);
        out.header.frame_id = "map";
        out.header.stamp = stamp;
        current_pub_->publish(out);

        pcl::toROSMsg(aligned, out);
        out.header.frame_id = "map";
        out.header.stamp = stamp;
        aligned_pub_->publish(out);

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "RViz 调试话题已发布:");
        RCLCPP_INFO(this->get_logger(), "  /debug/map_cloud      绿色 - 地图");
        RCLCPP_INFO(this->get_logger(), "  /debug/current_cloud   红色 - 当前定位点云");
        RCLCPP_INFO(this->get_logger(), "  /debug/aligned_cloud   白色 - ICP对齐后");
    }

    // 成员变量
    std::string map_pcd_path_;
    std::string cloud_topic_;
    int accumulate_frames_ = 5;
    double voxel_size_ = 0.3;
    int icp_max_iter_ = 50;
    double pos_threshold_ = 0.5;
    double rot_threshold_ = 5.0;

    bool checked_ = false;
    int frame_count_ = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accum_cloud_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitPosChecker>());
    rclcpp::shutdown();
    return 0;
}