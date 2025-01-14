#include "rclcpp/rclcpp.hpp"
#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"
#include "nbv_mapping/vdb_lifecycle.hpp"

#include "nbv_calculator/nbv_action_server.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=vdb_mapping_lifecycle"});
    std::shared_ptr<VDBMappingLifecycle> node = std::make_shared<VDBMappingLifecycle>(options, exec);
    options.arguments({"--ros-args", "-r", "__node:=nbv_action_server"});
    auto action_server = std::make_shared<nbv::NBVActionServer>(options);
    action_server->set_vdb_node(node);
    exec->add_node(node->get_node_base_interface());
    exec->add_node(action_server);
    exec->spin();
    rclcpp::shutdown();
    return 0;

}


