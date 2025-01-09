#ifndef NBV_MAPPING_H
#define NBV_MAPPING_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"

class VDBMappingLifecycle : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit VDBMappingLifecycle(const rclcpp::NodeOptions& options, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);
        virtual ~VDBMappingLifecycle() {};

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor() {return m_exec;}
        std::shared_ptr<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping>> vdb_map() {return m_vdb_map;}

        void reset() { m_vdb_map->resetMap(); }

    private:
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> m_exec;
        std::shared_ptr<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> > m_vdb_map;

};

#endif NBV_MAPPING_H