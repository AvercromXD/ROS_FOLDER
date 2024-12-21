#include "spot_hand_mapping/spot_hand_mapping.hpp"
#include <iostream>



SpotHandMapping::SpotHandMapping(const rclcpp::NodeOptions& options, std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
  : LifecycleNode("spot_hand_mapping",options)
  , m_exec(exec)
  , m_last_valid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  RCLCPP_INFO(this->get_logger(), "Initializing hand mapping");
  this->declare_parameter<int>("hand_map_publish_frequency", 20);
  this->declare_parameter<int>("hand_map_creation_frequency", 2);
  this->declare_parameter<std::string>("map_frame", "");
  this->declare_parameter<std::string>("robot_frame", "");
  this->declare_parameter<double>("hand_tcp_offset.x", 0.0);
  this->declare_parameter<double>("hand_tcp_offset.y", 0.0);
  this->declare_parameter<double>("hand_tcp_offset.z", 0.0);
}

SpotHandMapping::CallbackReturn
SpotHandMapping::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Configuring hand mapping");
  (void)previous_state;
  // Set up mapping
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=spot_hand_mapping"});

  this->get_parameter("hand_map_publish_frequency", m_pub_freq);
  this->get_parameter("hand_map_creation_frequency", m_creation_freq);
  this->get_parameter("map_frame", m_map_frame);
  this->get_parameter("robot_frame", m_robot_frame);
  this->get_parameter("hand_tcp_offset.x", m_hand_tcp_offset_x);
  this->get_parameter("hand_tcp_offset.y", m_hand_tcp_offset_y);
  this->get_parameter("hand_tcp_offset.z", m_hand_tcp_offset_z);


  m_vdb_map = std::make_shared<VDBMappingROS2<vdb_mapping::OccupancyVDBMapping> >(options);
  m_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/hand_map", 1);

  m_creation_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_publishing_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
  m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  return CallbackReturn::SUCCESS;
}

SpotHandMapping::CallbackReturn
SpotHandMapping::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  // Add node to executor
  m_exec->add_node(m_vdb_map);

  m_map_publish_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / m_pub_freq)),
                                                std::bind(&SpotHandMapping::mapPublishTimerCallback, this),
                                                m_publishing_cb_group);
  m_map_creation_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / m_creation_freq)),
                                                 std::bind(&SpotHandMapping::mapCreationTimerCallback, this),
                                                m_creation_cb_group);

  RCLCPP_INFO(this->get_logger(), "Activating Hand Mapping");
  return CallbackReturn::SUCCESS;
}

SpotHandMapping::CallbackReturn
SpotHandMapping::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  m_map_publish_timer.reset();
  m_map_creation_timer.reset();
  // reset map
  m_vdb_map->resetMap();
  // Remove mapping instance from executor
  m_exec->remove_node(m_vdb_map);

  RCLCPP_INFO(this->get_logger(), "Deactivating Hand Mapping");
  return CallbackReturn::SUCCESS;
}

SpotHandMapping::CallbackReturn
SpotHandMapping::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

SpotHandMapping::CallbackReturn
SpotHandMapping::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

void SpotHandMapping::mapCreationTimerCallback(){
  std::shared_lock map_lock(*m_vdb_map->getMap()->getMapMutex());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for(vdb_mapping::OccupancyVDBMapping::GridT::ValueOnCIter iter = m_vdb_map->getMap()->getGrid()->cbeginValueOn(); iter; ++iter)
  {
    openvdb::Vec3d world_coord = m_vdb_map->getMap()->getGrid()->indexToWorld(iter.getCoord());
    pcl::PointXYZRGB p;
    p.x = world_coord.x();
    p.y = world_coord.y();
    p.z = world_coord.z();
    cloud->points.push_back(p);
  }
  map_lock.unlock();
  cloud->width = cloud->points.size();
  cloud->height = 1;

  m_cloud_mutex.lock();
  m_last_valid_cloud = cloud;
  m_cloud_mutex.unlock();
}

void SpotHandMapping::mapPublishTimerCallback(){
  // Check if cloud is already initialized
  if(m_last_valid_cloud->points.size() == 0)
  {
    return;
  }
  geometry_msgs::msg::TransformStamped map_to_hand_tf;
  try{
    map_to_hand_tf = m_tf_buffer->lookupTransform(
    m_robot_frame, m_map_frame, rclcpp::Time(0) , rclcpp::Duration(1,0));
  }
  catch(tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform map to robot_frame: %s",
                 ex.what());
    return;
  }

  Eigen::Matrix<double, 1, 4> x_transform = tf2::transformToEigen(map_to_hand_tf).matrix().row(0);
  std::vector<double> transformed_x_coordinates;

  m_cloud_mutex.lock();

  double min_x=std::numeric_limits<double>::max();
  double max_x=std::numeric_limits<double>::min();
  for(auto& p : m_last_valid_cloud->points)
  {
    Eigen::Matrix<double, 4, 1> point_eigen;
    point_eigen << p.x,p.y,p.z,1.0;

    double x = x_transform * point_eigen;

    if(x < min_x)
    {
      min_x = x;
    }
    if(x > max_x)
    {
      max_x = x;
    }
    transformed_x_coordinates.push_back(x);
  }

  for(size_t i = 0; i < m_last_valid_cloud->points.size(); ++i)
  {
    std_msgs::msg::ColorRGBA color;
    double h = (1.0 - ((transformed_x_coordinates[i] - min_x) / (max_x - min_x)));
    color = VDBMappingTools<vdb_mapping::OccupancyVDBMapping>::heightColorCoding(h);
    m_last_valid_cloud->points[i].r = (int)(255* color.r);
    m_last_valid_cloud->points[i].g = (int)(255*color.g);
    m_last_valid_cloud->points[i].b = (int)(255*color.b);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*m_last_valid_cloud, cloud_msg);
  m_cloud_mutex.unlock();
  cloud_msg.header.frame_id = m_map_frame;
  m_map_pub->publish(cloud_msg);

  geometry_msgs::msg::TransformStamped gripper_tf;
  gripper_tf.header.stamp = this->get_clock()->now();
  gripper_tf.header.frame_id = m_robot_frame;
  gripper_tf.child_frame_id = "hand_tcp";
  gripper_tf.transform.translation.x = m_hand_tcp_offset_x;
  gripper_tf.transform.translation.y = m_hand_tcp_offset_y;
  gripper_tf.transform.translation.z = m_hand_tcp_offset_z; 
  gripper_tf.transform.rotation.x = 0;
  gripper_tf.transform.rotation.y = 0;
  gripper_tf.transform.rotation.z = 0;
  gripper_tf.transform.rotation.w = 1;

  m_tf_broadcaster->sendTransform(gripper_tf);

  gripper_tf.child_frame_id = "hand_tcp_rotated";
  tf2::Quaternion q;
  q.setRPY(3.14, 1.57, 0.0);
  gripper_tf.transform.rotation.x = q.x();
  gripper_tf.transform.rotation.y = q.y();
  gripper_tf.transform.rotation.z = q.z();
  gripper_tf.transform.rotation.w = q.w();
  m_tf_broadcaster->sendTransform(gripper_tf);


}
