#include "nbv_calculator/nbv_action_server.hpp"

namespace nbv {
    NBVActionServer::NBVActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("nbv_action_server", options)
    {
        using namespace std::placeholders;
        m_action_server = rclcpp_action::create_server<NBV>(
            this, 
            "nbv", 
            std::bind(&NBVActionServer::handle_goal, this, _1, _2),
            std::bind(&NBVActionServer::handle_cancel, this, _1),
            std::bind(&NBVActionServer::handle_accepted, this, _1)
            );
        m_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_);
        
        m_updates = this->create_subscription<UpdateGrid>("/vdb_mapping_lifecycle/vdb_map_updates", rclcpp::QoS(1).durability_volatile().best_effort(), std::bind(&NBVActionServer::update_callback, this, std::placeholders::_1));
        m_candidate_client = this->create_client<nbv_interfaces::srv::ViewPointSampling>("vi_to_nav/view_point_sampling");
        m_lifecycle_client = this->create_client<lifecycle_msgs::srv::ChangeState>("vdb_mapping_lifecycle/change_state");
        if (!change_lifecycle_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
            RCLCPP_ERROR(this->get_logger(), "Init failed, could not configure mapping lifecycle");
        }
        m_z_limit = 0.01;
        m_thresh_min = 0.3;
        m_thresh_max = 0.7;
        m_near = 0.01;
        m_far = 5.0;
        m_fov_samp = 0.01;
        m_resolution = 0.01;
        m_min_overlap = 0.1;
    }


    void NBVActionServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        set_map(m_vdb_lifecycle->vdb_map()->getMap());
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        m_updated = false;
        auto action_res = std::make_shared<NBV::Result>();
        m_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(goal->camera_name + "/camera_info", 10, std::bind(&NBVActionServer::info_callback, this, std::placeholders::_1));
        if(!change_lifecycle_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout, lifecycle could not be activated");
            action_res->result_msg = "Timeout, lifecycle could not be activated";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        std::unique_lock<std::mutex> lock(m_mutex);

        // Wait for vdb to update
        m_cond_v.wait_for(lock, std::chrono::minutes(2) , [this]() { return m_updated || m_canceled; }); 
        m_info_sub.reset();

        if(!change_lifecycle_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
        {
            RCLCPP_ERROR(this->get_logger(), "Timeout, lifecycle could not be activated");
            action_res->result_msg = "Timeout, lifecycle could not be activated";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        if (m_canceled) {
            RCLCPP_ERROR(this->get_logger(), "Goal canceled");
            action_res->result_msg = "Goal canceled";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        if (!m_updated) {
            RCLCPP_ERROR(this->get_logger(), "Timeout, map was not updated yet!");
            action_res->result_msg = "Timeout while waiting for vdb to update";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        } 
        
        RCLCPP_INFO(this->get_logger(), "Waiting done!");

        // Calculate ray to pixel
        cv::Vec3d ray = m_camera.fullIntrinsicMatrix().inv() * cv::Vec3d(goal->origin_x, goal->origin_y, 1.0);
 
        //Raytrace crom camera to clicked surface point
        geometry_msgs::msg::TransformStamped optical_to_map_tf;
        openvdb::Vec3d origin_point;
        try
        {
            optical_to_map_tf = m_tf_buffer_->lookupTransform("map", goal->optical_frame_name.c_str(), tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Transform optical frame to map frame failed: " << ex.what());
            action_res->success = false;
            action_res->result_msg = "Transform to map failed: " + std::string(ex.what());
            goal_handle->abort(action_res);
            return;
        }

        Eigen::Matrix<double, 4, 4> optical_to_map_mat = tf2::transformToEigen(optical_to_map_tf).matrix();
        Eigen::Matrix<double, 4, 1> direction;
        Eigen::Matrix<double, 3, 1> origin;
        origin = tf2::transformToEigen(optical_to_map_tf).translation();

        direction << ray[0], ray[1], ray[2], 0.0;

        direction = optical_to_map_mat * direction;

        bool success = m_map_ptr->raytrace(openvdb::Vec3d(origin.x(), origin.y(), origin.z()),
                                        openvdb::Vec3d(direction.x(), direction.y(), direction.z()),
                                        m_far,
                                        origin_point);
        if (success) 
        {
            RCLCPP_INFO(this->get_logger(), "Success");
        } else 
        {
            RCLCPP_INFO(this->get_logger(), "Fail");
        }
        RCLCPP_INFO(this->get_logger(), "End point x: %f, y: %f, z: %f", origin_point[0], origin_point[1], origin_point[2]);

        /*
        Start of loop
        */
        auto vdb_grid = m_map_ptr->getGrid();
        auto acc = vdb_grid->getAccessor();

        // Finding all surface and frontier voxels
        double max_dist_to_origin = 0;
        CoordQueue q;
        CoordQueue frontier_queue;
        std::unordered_set<openvdb::Coord, CoordinateHash> visited;
        std::unordered_set<openvdb::Coord, CoordinateHash> occupied;
        q.push(openvdb::Coord::floor(vdb_grid->worldToIndex(origin_point)));

        RCLCPP_INFO(this->get_logger(), "Iterating from origin, getting surface");

        openvdb::Coord p;
        while(q.pop(p) && !m_canceled)    
        { 
            auto p_world = vdb_grid->indexToWorld(p);
            visited.insert(p);
            double val = logodds_to_prob(acc.getValue(p));
            //RCLCPP_INFO(this->get_logger(), "Value %f", val);
            if (p_world[2] < m_z_limit) //too low
            {
                continue;
            }
            if (val < m_thresh_min) //free
            {
                continue;
            } else if (val > m_thresh_max) //occupied
            {
                occupied.insert(p);
                double dist = std::sqrt(std::pow(p_world[0] - origin_point.x(),2) + std::pow(p_world[1] - origin_point.y(),2) + std::pow(p_world[2] - origin_point.z(),2));
                max_dist_to_origin = dist > max_dist_to_origin ? dist : max_dist_to_origin;
                for (openvdb::Coord n : NBVActionServer::get_neighbors(p))
                {
                    if (visited.find(n) == visited.end() && !q.contains(n)) // TODO Check if in queue
                    {
                        q.push(n);
                    }
                }
            } else //unknown
            {
                for (openvdb::Coord n : NBVActionServer::get_neighbors(p))
                {
                    if (acc.getValue(n) < m_thresh_min) //frontier
                    {
                        if (!frontier_queue.contains(p))
                        {
                            frontier_queue.push(p);
                        }
                    }
                }
            }
        }

        if (m_canceled) {
            RCLCPP_ERROR(this->get_logger(), "Goal canceled");
            action_res->result_msg = "Goal canceled";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Found %ld surface voxels", occupied.size());
        RCLCPP_INFO(this->get_logger(), "Grouping frontiers, with %ld number of frontier voxels", frontier_queue.size());

        //Grouping all frontier voxels
        std::vector<std::vector<openvdb::Coord>> frontiers = group_frontiers(frontier_queue);

        if (m_canceled) {
            RCLCPP_ERROR(this->get_logger(), "Goal canceled");
            action_res->result_msg = "Goal canceled";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Found %ld frontiers", frontiers.size());
        
        //Calculating the centroids of frontiers
        std::vector<nbv_interfaces::msg::Centroid> centroids = calculate_centroids(frontiers);

        // Requesting candidate views
        auto req = std::make_shared<nbv_interfaces::srv::ViewPointSampling::Request>();
        req->centroids = centroids;
        req->cam_info = m_cam_info;
        req->optical_frame = goal->optical_frame_name;

        while (!m_candidate_client->wait_for_service(std::chrono::seconds(1)) && !m_canceled) 
        {
            if (!rclcpp::ok() || m_canceled) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                action_res->result_msg = "Interrupted while waiting for service. Exiting.";
                action_res->success = false;
                goal_handle->abort(action_res);
                return; 
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        //Getting candidate views
        auto future = m_candidate_client->async_send_request(req);

        while(future.wait_for(std::chrono::milliseconds(100)) == std::future_status::timeout && !m_canceled) 
        {
            RCLCPP_INFO(this->get_logger(), "waiting for result");
        }
 
        if (m_canceled) {
            RCLCPP_ERROR(this->get_logger(), "Goal canceled");
            action_res->result_msg = "Goal canceled";
            action_res->success = false;
            goal_handle->abort(action_res);
            return;
        }
        auto res = future.get();

        // Evaluating candidates
        std::vector<nbv_interfaces::msg::CandidateView> candidate_views = res->view_points;
        geometry_msgs::msg::TransformStamped cam_to_map_tf;
        try
        {
            cam_to_map_tf = m_tf_buffer_->lookupTransform("map", m_cam_info.header.frame_id.c_str(), tf2::TimePointZero);
        }
        catch(tf2::TransformException &ex)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Transform camera to map frame failed: " << ex.what());
            action_res->success = false;
            action_res->result_msg = "Transform to map failed: " + std::string(ex.what());
            goal_handle->abort(action_res);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Evaluating candidates");
        double max_unknown_found = 0;
        std::vector<nbv::CandidateView> valid_views = evaluate_candidates(candidate_views, optical_to_map_tf, cam_to_map_tf, max_unknown_found, origin_point, max_dist_to_origin, occupied);
        CandidateView *max = &valid_views.front();
        for (CandidateView candidate : valid_views)
        {
            candidate.eval = (1.0 / (1.0 + candidate.view.d)) * ((double)(candidate.num_unknown_vox) / max_unknown_found) * std::pow((double)(candidate.num_surface_vox) / occupied.size(), 2);
            max = max->eval < candidate.eval ? &candidate : max;
        }
        RCLCPP_INFO(this->get_logger(), "Best pose %f, %f, %f, %f, %f, %f, %f", max->view.cam_pose.position.x, max->view.cam_pose.position.y, max->view.cam_pose.position.z, max->view.cam_pose.orientation.w, max->view.cam_pose.orientation.x, max->view.cam_pose.orientation.y, max->view.cam_pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), "Eval: %f", max->eval);

        action_res->result_msg = "Success";
        action_res->success = true;
        goal_handle->succeed(action_res);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto action_server_node = std::make_shared<nbv::NBVActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server_node);
  executor.spin();
  return 0;
}