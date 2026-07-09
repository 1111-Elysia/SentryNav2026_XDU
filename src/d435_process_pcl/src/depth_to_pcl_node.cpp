#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <algorithm>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DepthToPclStable : public rclcpp::Node {
public:
    DepthToPclStable() : Node("depth_to_pcl_stable") {

        this->declare_parameter<std::string>("output_topic", "d435_pointcloud");
        this->declare_parameter<std::string>("depth_image_topic", "/camera/camera/depth/image_rect_raw");
        this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/depth/camera_info");
        this->declare_parameter<std::string>("frame_id", "d435_frame");
        this->declare_parameter<int>("step", 2);
        this->declare_parameter<int>("edge_margin", 5);
        this->declare_parameter<float>("min_distance", 0.2f);
        this->declare_parameter<float>("max_distance", 3.0f);

        this->declare_parameter<int>("median_kernel_size", 5);
        this->declare_parameter<int>("temporal_frames", 5);
        this->declare_parameter<double>("voxel_leaf_size", 0.03);
        this->declare_parameter<int>("sor_mean_k", 20);
        this->declare_parameter<double>("sor_stddev_thresh", 1.0);

        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("depth_image_topic", depth_image_topic_);
        this->get_parameter("camera_info_topic", camera_info_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("step", step_);
        this->get_parameter("edge_margin", edge_margin_);
        this->get_parameter("min_distance", min_distance_);
        this->get_parameter("max_distance", max_distance_);
        this->get_parameter("median_kernel_size", median_kernel_size_);
        this->get_parameter("temporal_frames", temporal_frames_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("sor_mean_k", sor_mean_k_);
        this->get_parameter("sor_stddev_thresh", sor_stddev_thresh_);

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        depth_sub_.subscribe(this, depth_image_topic_);
        info_sub_.subscribe(this, camera_info_topic_);

        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        sync_ = std::make_shared<Sync>(MySyncPolicy(10));

        sync_->connectInput(depth_sub_, info_sub_);

        sync_->registerCallback(
            std::bind(&DepthToPclStable::callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)
        );

        // ===== Publish Static TF =====
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        double tf_x = this->declare_parameter<double>("tf_x", 0.0);
        double tf_y = this->declare_parameter<double>("tf_y", 0.0);
        double tf_z = this->declare_parameter<double>("tf_z", 0.0);
        double tf_yaw = this->declare_parameter<double>("tf_yaw_deg", 0.0) * M_PI / 180.0;
        double tf_pitch = this->declare_parameter<double>("tf_pitch_deg", 0.0) * M_PI / 180.0;
        double tf_roll = this->declare_parameter<double>("tf_roll_deg", 0.0) * M_PI / 180.0;
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = frame_id_;
        
        t.transform.translation.x = tf_x;
        t.transform.translation.y = tf_y;
        t.transform.translation.z = tf_z;
        
        tf2::Quaternion q;
        q.setRPY(tf_roll, tf_pitch, tf_yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "Stable Depth PCL node started.");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {

        // ===== camera model =====
        image_geometry::PinholeCameraModel cam;
        cam.fromCameraInfo(info_msg);

        double fx = cam.fx();
        double fy = cam.fy();
        double cx = cam.cx();
        double cy = cam.cy();

        // ===== image =====
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(depth_msg);
        } catch (...) {
            return;
        }

        cv::Mat depth = cv_ptr->image;

        // ===== 1. Spatial Filter (空间滤波，去散粒噪点) =====
        cv::Mat depth_filtered;
        if (depth.type() == CV_16U || depth.type() == CV_8U) {
            cv::medianBlur(depth, depth_filtered, median_kernel_size_);
        } else {
            depth_filtered = depth;
        }

        bool is32f = (depth_msg->encoding == "32FC1");
        float scale = is32f ? 1.0f : 0.001f;

        cv::Mat current_float;
        if (!is32f) {
            depth_filtered.convertTo(current_float, CV_32FC1, scale);
        } else {
            current_float = depth_filtered.clone();
        }

        // ===== 1.5 Temporal Median Filter (多帧中值去水面波动) =====
        int N = temporal_frames_;
        if (N < 1) N = 1;

        // 帧数或尺寸变化时重建历史队列
        if (history_frames_.empty() || history_frames_.front().size() != current_float.size()) {
            history_frames_.clear();
            for (int k = 0; k < N; ++k) {
                history_frames_.push_back(current_float.clone());
            }
        } else {
            // 滑动窗口：丢弃最老帧，压入最新帧
            history_frames_.erase(history_frames_.begin());
            history_frames_.push_back(current_float.clone());

            // 逐像素取中值
            std::vector<float> vals(N);
            for (int i = 0; i < current_float.rows; ++i) {
                float* out_row = current_float.ptr<float>(i);
                for (int j = 0; j < current_float.cols; ++j) {
                    int valid = 0;
                    for (int k = 0; k < N; ++k) {
                        float v = history_frames_[k].at<float>(i, j);
                        if (v > 0.05f && v < 15.0f) {
                            vals[valid++] = v;
                        }
                    }
                    if (valid > 0) {
                        std::nth_element(vals.begin(), vals.begin() + valid / 2, vals.begin() + valid);
                        out_row[j] = vals[valid / 2];
                    } else {
                        out_row[j] = 0.0f;  // 无有效值则清零
                    }
                }
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        cloud->header.frame_id = frame_id_;
        pcl_conversions::toPCL(depth_msg->header.stamp, cloud->header.stamp);

        int rows = current_float.rows;
        int cols = current_float.cols;

        cloud->points.reserve(rows * cols / (step_ * step_));

        // ===== 2. projection (跳过图像边缘 margin 像素，避免边界伪影) =====
        int margin = std::max(0, edge_margin_);
        int v_start = margin;
        int v_end = rows - margin;
        int u_start = margin;
        int u_end = cols - margin;

        for (int v = v_start; v < v_end; v += step_) {
            for (int u = u_start; u < u_end; u += step_) {

                float d = current_float.at<float>(v, u);

                // ===== 3. 强过滤（核心稳定性与距离限制）=====
                if (!std::isfinite(d) || d < min_distance_ || d > max_distance_)
                    continue;

                // ===== 4. pinhole projection =====
                pcl::PointXYZ pt;
                pt.x = (u - cx) * d / fx;
                pt.y = (v - cy) * d / fy;
                pt.z = d;

                cloud->points.push_back(pt);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;

        // ===== 5. voxel grid（稳定墙面）=====
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        float vls = static_cast<float>(voxel_leaf_size_);
        vg.setLeafSize(vls, vls, vls);
        vg.filter(*voxel);

        // ===== 6. statistical outlier removal（去飞点）=====
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(voxel);
        sor.setMeanK(sor_mean_k_);
        sor.setStddevMulThresh(sor_stddev_thresh_);
        sor.filter(*filtered);

        // ===== 7. publish =====
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered, output);

        output.header.frame_id = frame_id_;
        output.header.stamp = depth_msg->header.stamp;

        pub_->publish(output);
    }

private:
    std::string output_topic_;
    std::string depth_image_topic_;
    std::string camera_info_topic_;
    std::string frame_id_;
    int step_;
    int edge_margin_;
    float min_distance_;
    float max_distance_;
    int median_kernel_size_;
    int temporal_frames_;
    double voxel_leaf_size_;
    int sor_mean_k_;
    double sor_stddev_thresh_;
    
    std::vector<cv::Mat> history_frames_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo
    > MySyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPclStable>());
    rclcpp::shutdown();
    return 0;
}