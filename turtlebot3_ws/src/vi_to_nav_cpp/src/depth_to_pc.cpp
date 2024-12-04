#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "depth_image_proc/conversions.hpp"
#include "image_transport/image_transport.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthToPCNode : public rclcpp::Node
{
    private: std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
    private: image_transport::CameraSubscriber sub_depth_;
    private: image_geometry::PinholeCameraModel model_;

    public: DepthToPCNode(): Node("depth_to_pc")
    {
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth/pointcloud2", 10);
        sub_depth_ = image_transport::create_camera_subscription(this, "/depth/image_raw", std::bind(&DepthToPCNode::callback, this, std::placeholders::_1, std::placeholders::_2), "raw", rmw_qos_profile_system_default);


    }

    private:void callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Callback");
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = depth_msg->header;
        cloud_msg->height = depth_msg->height;
        cloud_msg->width = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;
        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        // Update camera model
        model_.fromCameraInfo(info_msg);

        // Convert Depth Image to Pointcloud
        if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || depth_msg->encoding == sensor_msgs::image_encodings::MONO16) {
            depth_image_proc::convertDepth<uint16_t>(depth_msg, cloud_msg, model_);
        } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            depth_image_proc::convertDepth<float>(depth_msg, cloud_msg, model_);
        } else {
            RCLCPP_ERROR(
            get_logger(), "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
            return;
        }
        pub_->publish(*cloud_msg);
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPCNode>());
    rclcpp::shutdown();
    return 0;
}