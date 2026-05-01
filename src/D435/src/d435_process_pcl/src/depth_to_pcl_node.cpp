#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class DepthToPclStable : public rclcpp::Node {
public:
    DepthToPclStable() : Node("depth_to_pcl_stable") {

        this->declare_parameter<std::string>("output_topic", "d435_pointcloud");
        this->declare_parameter<std::string>("frame_id", "d435_frame");
        this->declare_parameter<int>("step", 2);
        this->declare_parameter<float>("min_distance", 0.3f);
        this->declare_parameter<float>("max_distance", 6.0f);

        this->get_parameter("output_topic", output_topic_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("step", step_);
        this->get_parameter("min_distance", min_distance_);
        this->get_parameter("max_distance", max_distance_);

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

        depth_sub_.subscribe(this, "/camera/camera/depth/image_rect_raw");
        info_sub_.subscribe(this, "/camera/camera/depth/camera_info");

        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        sync_ = std::make_shared<Sync>(MySyncPolicy(10));

        sync_->connectInput(depth_sub_, info_sub_);

        sync_->registerCallback(
            std::bind(&DepthToPclStable::callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2)
        );

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
            cv::medianBlur(depth, depth_filtered, 5);
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

        // ===== 1.5 Temporal Filter (时域平滑，解决水面波动) =====
        if (history_depth_.empty() || history_depth_.size() != current_float.size()) {
            history_depth_ = current_float.clone();
        } else {
            float alpha = 0.4f;        // 当前帧权重：越小抗水波纹能力越强，会有类似残影的效应
            float delta_thresh = 0.05f; // 阈值5cm：深度变化大于此值说明是物体真实移动

            for (int i = 0; i < current_float.rows; ++i) {
                float* curr_row = current_float.ptr<float>(i);
                float* hist_row = history_depth_.ptr<float>(i);
                for (int j = 0; j < current_float.cols; ++j) {
                    float c = curr_row[j];
                    float h = hist_row[j];

                    if (c > 0.1f && c < 10.0f) { 
                        if (h > 0.1f && std::abs(c - h) < delta_thresh) {
                            float smoothed = alpha * c + (1.0f - alpha) * h;
                            curr_row[j] = smoothed;
                            hist_row[j] = smoothed;
                        } else {
                            hist_row[j] = c;
                        }
                    } else {
                        hist_row[j] = 0.0f;
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

        // ===== 2. projection =====
        for (int v = 0; v < rows; v += step_) {
            for (int u = 0; u < cols; u += step_) {

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
        vg.setLeafSize(0.03f, 0.03f, 0.03f);
        vg.filter(*voxel);

        // ===== 6. statistical outlier removal（去飞点）=====
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(voxel);
        sor.setMeanK(20);
        sor.setStddevMulThresh(1.0);
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
    std::string frame_id_;
    int step_;
    float min_distance_;
    float max_distance_;
    
    cv::Mat history_depth_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo
    > MySyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPclStable>());
    rclcpp::shutdown();
    return 0;
}