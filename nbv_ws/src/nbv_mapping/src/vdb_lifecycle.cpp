#include "nbv_mapping/vdb_lifecycle.hpp"

VDBMappingLifecycle::VDBMappingLifecycle(const rclcpp::NodeOptions& options, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec) : LifecycleNode("vdb_mapping_lifecycle", options), m_exec(exec)
{
    RCLCPP_INFO(this->get_logger(), "Init vdb_mapping lifecycle");
}

VDBMappingLifecycle::CallbackReturn VDBMappingLifecycle::on_configure(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "Configuring mapping lifecycle");
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=vdb_mapping_lifecycle"});
    m_vdb_map = std::make_shared<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> >(options);
    return CallbackReturn::SUCCESS; 
}

VDBMappingLifecycle::CallbackReturn VDBMappingLifecycle::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "Activating mapping");
    m_exec->add_node(m_vdb_map);
    return CallbackReturn::SUCCESS;
}

VDBMappingLifecycle::CallbackReturn VDBMappingLifecycle::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "Deactivating mapping");
    m_exec->remove_node(m_vdb_map);
    return CallbackReturn::SUCCESS;
}

VDBMappingLifecycle::CallbackReturn VDBMappingLifecycle::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    return CallbackReturn::SUCCESS;
}

VDBMappingLifecycle::CallbackReturn VDBMappingLifecycle::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    return CallbackReturn::SUCCESS;
}