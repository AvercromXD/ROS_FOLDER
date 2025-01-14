#include "rclcpp/rclcpp.hpp"
#include "nbv_mapping/vdb_lifecycle.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"--ros_args", "-r", "__node:=vdb_mapping_lifecycle"});
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  
  VDBMappingLifecycle node(options, exec);
  exec->add_node(node.get_node_base_interface());

  exec->spin();
  rclcpp::shutdown();
  return 0;
}
