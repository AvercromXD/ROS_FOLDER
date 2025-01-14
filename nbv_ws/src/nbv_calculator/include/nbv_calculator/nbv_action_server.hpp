#ifndef NBV_ACTION_SERVER_H
#define NBV_ACTION_SERVER_H

#include "nbv_interfaces/action/nbv.hpp"
#include "nbv_interfaces/srv/view_point_sampling.hpp"
#include "nbv_interfaces/msg/candidate_view.hpp"
#include "nbv_interfaces/msg/centroid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "vdb_mapping_interfaces/msg/update_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>
#include <chrono>

#include "rclcpp_components/register_node_macro.hpp"
#include "vdb_mapping/OccupancyVDBMapping.h"
#include "vdb_mapping_ros2/VDBMappingROS2.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <unordered_set>
#include <iostream>
#include "image_geometry/pinhole_camera_model.h"
#include <cmath>
#include <algorithm>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include "lifecycle_msgs/msg/transition.hpp"

#include "nbv_mapping/vdb_lifecycle.hpp"

namespace nbv {

    struct CoordinateHash {
        size_t operator()(const openvdb::Coord coord) const {
            return coord.hash();
        }
    };

    struct CandidateView {
        nbv_interfaces::msg::CandidateView view;
        int num_surface_vox;
        int num_unknown_vox;
        double eval;
    };

    bool compare_candidate(const CandidateView& a, const CandidateView &b)
    {
        return a.eval < b.eval;
    }

    class CoordQueue
    {
        private:
            std::queue<openvdb::Coord> queue;
            std::unordered_set<openvdb::Coord, CoordinateHash> set;


        public:
            void push(openvdb::Coord c)
            {
                queue.push(c);
                set.insert(c);
            }

            bool empty()
            {
                return set.empty();
            }

            bool contains(openvdb::Coord c)
            {
                return (set.find(c) != set.end());
            }

            bool pop(openvdb::Coord &res)
            {
                if (set.empty())
                {
                    return false;
                }

                do
                {
                    res = queue.front();
                    queue.pop();
                } while (set.find(res)==set.end());

                set.erase(res);
                return true;
            }

            void remove(openvdb::Coord c)
            {
                if (set.find(c)!=set.end())
                {
                    set.erase(c);
                }
            }

            size_t size()
            {
                return set.size();
            }
    };

class NBVActionServer : public rclcpp::Node {

    using NBV = nbv_interfaces::action::Nbv;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NBV>;
    using PointCloud = sensor_msgs::msg::PointCloud2;
    using UpdateGrid = vdb_mapping_interfaces::msg::UpdateGrid;

    using RayT  = openvdb::math::Ray<double>;
    using DDAT  = openvdb::math::DDA<RayT, 0>;

    public:
        explicit NBVActionServer(const rclcpp::NodeOptions &options);
        ~NBVActionServer() override = default;
        
        void set_map(std::shared_ptr<vdb_mapping::OccupancyVDBMapping> map_ptr)
        {
            m_map_ptr = map_ptr;
        }

        void set_vdb_node(std::shared_ptr<VDBMappingLifecycle> node)
        {
            m_vdb_lifecycle = node;
        }

    private:
        bool m_updated;
        bool m_canceled;
        double m_resolution;
        double m_near;
        double m_far;
        double m_fov_samp;
        double m_min_overlap;
        sensor_msgs::msg::CameraInfo m_cam_info;
        sensor_msgs::msg::Image m_depth;

        image_geometry::PinholeCameraModel m_camera;

        float m_thresh_min; //below, then free
        float m_thresh_max; //above then occupied
        float m_z_limit; //height to segment floor

        std::mutex m_mutex;
        std::condition_variable m_cond_v;

        std::shared_ptr<vdb_mapping::OccupancyVDBMapping> m_map_ptr;
        std::shared_ptr<VDBMappingLifecycle> m_vdb_lifecycle;

        rclcpp_action::Server<NBV>::SharedPtr m_action_server;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_info_sub;
        rclcpp::Client<nbv_interfaces::srv::ViewPointSampling>::SharedPtr m_candidate_client;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr m_lifecycle_client;

        rclcpp::Subscription<UpdateGrid>::SharedPtr m_updates;

        std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_;



        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NBV::Goal> goal)
        {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
        {
            m_canceled = true;
            m_cond_v.notify_all();
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
            m_canceled = false;
            using namespace std::placeholders;
            std::thread{std::bind(&NBVActionServer::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandle> goal_handle);

        void info_callback(const sensor_msgs::msg::CameraInfo &msg)
        {
            m_cam_info = msg; // Different for camera and depth camera
            m_camera.fromCameraInfo(msg);
        }
        void update_callback(const UpdateGrid &msg)
        {
            m_updated = true;
            m_cond_v.notify_all();
            RCLCPP_INFO(this->get_logger(), "Map updated");
        }

        std::vector<openvdb::Coord> get_neighbors(openvdb::Coord p)
        {
            std::vector<openvdb::Coord> res = {};
            for(int i = -1; i <= 1; i++)
            {
                for (int j = -1; j <= 1; j ++)
                {
                    for (int k = -1; k <= 1; k ++)
                    {
                        if (i == j && j == k && k == 0) {
                            continue;
                        }
                        res.push_back(openvdb::Coord(p[0] + i, p[1] + j, p[2] + k));
                    }
                }
            }
            return res;
        }

        bool change_lifecycle_state(uint8_t transition)
        {
            auto start_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> timeout(10.0);

            while (!m_lifecycle_client->wait_for_service(std::chrono::seconds(1))) 
            {
                auto current = std::chrono::steady_clock::now();
                std::chrono::duration<double> elapsed_seconds = current - start_time;
                if (elapsed_seconds > timeout) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout, waited 10 seconds...");
                    return false;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "lifecycle service not available, waiting again...");
            }
            auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            req->transition.id = transition;
            auto future = m_lifecycle_client->async_send_request(req);
            return true;
        }

        double logodds_to_prob(double logodds)
        {
            return 1.0 / (1.0 + std::exp(-logodds));
        }

        bool raytrace_in_range(const openvdb::Vec3d& ray_origin, const openvdb::Vec3d& ray_direction, const double max_ray_length, const openvdb::Vec3d& surface, const double dist, openvdb::Coord& end_point)
        {
            auto vdb_grid = m_map_ptr->getGrid();
            auto acc = vdb_grid->getAccessor();
            openvdb::Vec3d ray_origin_index = vdb_grid->worldToIndex(ray_origin);
            openvdb::Vec3d surface_index = vdb_grid->worldToIndex(surface);
            RayT ray(ray_origin_index, ray_direction);
            DDAT dda(ray);
            while (true)
            {
                double distance = 
                m_resolution * std::sqrt(std::pow((dda.voxel().x() - ray_origin_index.x()), 2) +
                                        std::pow((dda.voxel().y() - ray_origin_index.y()), 2) +
                                        std::pow((dda.voxel().z() - ray_origin_index.z()), 2));
                double dist_to_surface = 
                m_resolution * std::sqrt(std::pow((dda.voxel().x() - surface_index.x()), 2) +
                                        std::pow((dda.voxel().y() - surface_index.y()), 2) +
                                        std::pow((dda.voxel().z() - surface_index.z()), 2));
                if (distance < max_ray_length)
                {
                    if (dist_to_surface < dist)
                    {
                        double val = logodds_to_prob(acc.getValue(dda.voxel()));
                        if (val > m_thresh_max) // Occupied
                        {
                            end_point = dda.voxel();
                            return true;
                        } else if (val >= m_thresh_min)
                        {
                            end_point = dda.voxel();
                            return true;
                        }

                    }
                    dda.step();
                } else 
                {
                    return false;
                }
            }
        }

        std::tuple<double, double> raytrace_partial_frustum(int i_start, int j_start, int i_stop, int j_stop, Eigen::Matrix<double, 4, 4>& cam_to_map, Eigen::Matrix<double, 4, 4>& t_new_cam_pose, const openvdb::Vec3d& cam_pos, openvdb::Vec3d& origin_point, double max_dist_to_origin, std::unordered_set<openvdb::Coord, CoordinateHash> occupied)
        {
            double num_occupied = 0.0;
            double num_unknown = 0.0;
            for (int i = i_start; (i < i_stop && !m_canceled); i++)
            {
                for (int j = j_start; (j < j_stop && !m_canceled); j++)
                {
                    cv::Vec3d ray = m_camera.fullIntrinsicMatrix().inv() * cv::Vec3d((double) i, (double) j, 1.0);

                    Eigen::Matrix<double, 4, 1> direction;
                    direction << ray[0], ray[1], ray[2], 0.0;
                    direction = cam_to_map * direction;
                    direction = t_new_cam_pose * direction;
                    openvdb::Coord end_point;
                    if (raytrace_in_range(cam_pos, openvdb::Vec3d(direction.x(), direction.y(), direction.z()), m_far, origin_point, max_dist_to_origin, end_point))
                    {
                        if (occupied.find(end_point) != occupied.end())
                        {
                            num_occupied++;
                        } else 
                        {
                            num_unknown++;
                        }
                    }
                }
            }
            std::tuple<double, double>res(num_unknown,num_occupied);
            return res;
        }

        std::vector<std::vector<openvdb::Coord>> group_frontiers(CoordQueue &frontier_queue)
        {
            std::vector<std::vector<openvdb::Coord>> frontiers;
            while (!frontier_queue.empty() && !m_canceled)
            {
                openvdb::Coord f; 
                if (frontier_queue.pop(f))
                {
                    std::vector<openvdb::Coord> frontier;
                    frontier.push_back(f);
                    std::queue<openvdb::Coord> local_queue;
                    local_queue.push(f);
                    while (!local_queue.empty())
                    {
                        auto local_f = local_queue.front();
                        local_queue.pop();
                        for (openvdb::Coord n : NBVActionServer::get_neighbors(local_f))
                        {
                            if (frontier_queue.contains(n))
                            {
                                frontier.push_back(n);
                                frontier_queue.remove(n);
                                local_queue.push(n);
                            }
                        }
                    }
                    frontiers.push_back(frontier);
                }
            }
            return frontiers;
        }

        std::vector<nbv_interfaces::msg::Centroid> calculate_centroids(std::vector<std::vector<openvdb::Coord>> frontiers)
        {
            auto vdb_grid = m_map_ptr->getGrid();
            std::vector<nbv_interfaces::msg::Centroid> centroids;
            for (std::vector<openvdb::Coord> frontier : frontiers)
            {
                geometry_msgs::msg::Point centroid;
                for (openvdb::Coord coord : frontier)
                {
                    auto coord_world = vdb_grid->indexToWorld(coord);
                    centroid.x += coord_world[0];
                    centroid.y += coord_world[1];
                    centroid.z += coord_world[2];
                }
                centroid.x /= frontier.size();
                centroid.y /= frontier.size();
                centroid.z /= frontier.size();
                nbv_interfaces::msg::Centroid centroid_msg;
                centroid_msg.pos = centroid;
                centroids.push_back(centroid_msg);
            }
            return centroids;
        }
        
        std::vector<nbv::CandidateView> evaluate_candidates(std::vector<nbv_interfaces::msg::CandidateView> candidate_views, geometry_msgs::msg::TransformStamped optical_to_map_tf, geometry_msgs::msg::TransformStamped cam_to_map_tf, double& max_unknown_found, openvdb::Vec3d& origin_point, double max_dist_to_origin, std::unordered_set<openvdb::Coord, CoordinateHash> occupied)
        {
            auto vdb_grid = m_map_ptr->getGrid();
            std::vector<nbv::CandidateView> valid_views;
            geometry_msgs::msg::Pose cam_frame_pose, current_map_pose;
            Eigen::Matrix<double, 4, 4> optical_to_map_mat = tf2::transformToEigen(optical_to_map_tf).matrix();

            tf2::doTransform(cam_frame_pose, current_map_pose, cam_to_map_tf);
            geometry_msgs::msg::Vector3 up_camera, look_camera, map_look_dir, map_up;
            up_camera.y = -1.0;
            look_camera.z = 1.0;

            tf2::doTransform(up_camera, map_up, optical_to_map_tf);
            tf2::doTransform(look_camera, map_look_dir, optical_to_map_tf);

            double half_fov_x = atan2(m_cam_info.width/2.0, m_cam_info.k[0]);
            double half_fov_y = atan2(m_cam_info.height/2.0, m_cam_info.k[4]);
            double tan_fov_x = tan(half_fov_x);
            double tan_fov_y = tan(half_fov_y);

            for (nbv_interfaces::msg::CandidateView c : candidate_views)
            {
                geometry_msgs::msg::Pose cam_pose = c.cam_pose;
                CandidateView candidate;
                candidate.view = c;
                candidate.num_surface_vox = 0;
                candidate.num_unknown_vox = 0;

                //Transform look and up vectors to new candidate pose
                geometry_msgs::msg::Vector3 up, look_dir;
                tf2::Quaternion source_quat, target_quat, relative_quat;
                tf2::fromMsg(current_map_pose.orientation, source_quat);
                tf2::fromMsg(cam_pose.orientation, target_quat);
                relative_quat = target_quat * source_quat.inverse();
                geometry_msgs::msg::TransformStamped new_pose_tf;
                new_pose_tf.transform.rotation = tf2::toMsg(relative_quat);
                tf2::doTransform(map_up, up, new_pose_tf);
                tf2::doTransform(map_look_dir, look_dir, new_pose_tf);

                double norm = std::sqrt(std::pow(up.x, 2) + std::pow(up.y, 2) + std::pow(up.z, 2));
                up.x /= norm;
                up.y /= norm;
                up.z /= norm;
                norm = std::sqrt(std::pow(look_dir.x, 2) + std::pow(look_dir.y, 2) + std::pow(look_dir.z, 2));
                look_dir.x /= norm;
                look_dir.y /= norm;
                look_dir.z /= norm;

                //cross product
                std::vector<double> right = {0.0, 0.0, 0.0};
                right[0] = look_dir.y * up.z - look_dir.z * up.y;
                right[1] = look_dir.z * up.x - look_dir.x * up.z;
                right[2] = look_dir.x * up.y - look_dir.y * up.x;
                norm = std::sqrt(std::pow(right[0], 2) + std::pow(right[1], 2) + std::pow(right[2], 2));
                right[0] /= norm;
                right[1] /= norm;
                right[2] /= norm;

                // Counting occupied voxels in view frustum
                for (openvdb::Coord voxel : occupied)
                {
                    if (m_canceled) {
                        return valid_views;
                    }
                    auto point = vdb_grid->indexToWorld(voxel);
                    double dist = (point[0] - cam_pose.position.x) * look_dir.x + (point[1] - cam_pose.position.y) * look_dir.y + (point[2] - cam_pose.position.z) * look_dir.z;
                    if (dist < m_near || dist > m_far)
                    {
                        continue;
                    }
                    double width_at_dist = 2 * dist * tan_fov_x;
                    double height_at_dist = 2 * dist * tan_fov_y;
                    double right_dist = (point[0] - cam_pose.position.x) * right[0] + (point[1] - cam_pose.position.y) * right[1] + (point[2] - cam_pose.position.z) * right[2];

                    double up_dist = (point[0] - cam_pose.position.x) * up.x + (point[1] - cam_pose.position.y) * up.y + (point[2] - cam_pose.position.z) * up.z;
                    if (abs(right_dist) > width_at_dist / 2.0 || abs(up_dist) > height_at_dist / 2.0)
                    {
                        continue;
                    }
                    candidate.num_surface_vox++;
                }
                // Sampling view frustum with raytracing
                std::tuple<double, double> ray_res = raytrace_partial_frustum(0, 0, m_cam_info.width, m_cam_info.height, optical_to_map_mat, tf2::transformToEigen(new_pose_tf).matrix(), openvdb::Vec3d(cam_pose.position.x, cam_pose.position.y, cam_pose.position.z), origin_point, max_dist_to_origin, occupied);

                if (m_canceled) {
                    return valid_views;
                }

                candidate.num_unknown_vox = std::get<1>(ray_res);
                max_unknown_found = max_unknown_found < candidate.num_unknown_vox ? candidate.num_unknown_vox : max_unknown_found;
                RCLCPP_INFO(this->get_logger(), "number unknown: %d", candidate.num_unknown_vox);

                if (std::get<0>(ray_res) > occupied.size() * m_min_overlap)
                {
                    RCLCPP_INFO(this->get_logger(), "Valid view");
                    valid_views.push_back(candidate);
                } 

            }
            return valid_views;
        }
    
};

}

#endif NBV_ACTION_SERVER_H