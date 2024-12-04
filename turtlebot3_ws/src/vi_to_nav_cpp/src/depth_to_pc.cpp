#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "depth_image_proc/conversions.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthToPCNode : public rclcpp::Node
{
    private: std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;

    public: DepthToPCNode(): Node("depth_to_pc")
    {
        this->create_subscription<sensor_msgs::msg::Image>("/depth/image_raw", 10, std::bind(&DepthToPCNode::depth_image_callback, this, _1));
        this->create_subscription<sensor_msgs::msg::CameraInfo>("/depth/camera_info", 10, std::bind(&DepthToPCNode::info_callback, this, _1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth/pointcloud2", 10);

    }

    private:void depth_image_callback(const sensor_msgs::msg::Image &msg)
    {

    }

    private: void info_callback(const sensor_msgs::msg::CameraInfo &msg)
    {

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPCNode>());
    rclcpp::shutdown();
    return 0;
}