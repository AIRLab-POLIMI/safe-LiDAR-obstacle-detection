#include <chrono>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

struct Vertex {
    double x;
    double y;
};

struct Polygon {
    std::vector<Vertex> vertices;

    // Function to get the size of the polygon
    int size() const {
        return vertices.size();
    }
};

double degreesToRadians(double degrees){
    return degrees * M_PI / 180.0;
}

class LidarFilteringNode : public rclcpp::Node
{
public:
  LidarFilteringNode() : Node("lidar_filtering_node")
  {
    // Retrieve the parameter values from the parameter server
    this->declare_parameter<std::string>("lidar_topic", "/robot/lidar/points");
    this->declare_parameter<double>("cube_edge_length", 12.0);
    this->declare_parameter<double>("z_lower_limit", -0.465);
    this->declare_parameter<double>("x_lower_limit", -7.0);
    this->declare_parameter<double>("x_upper_limit", 5.0);
    this->declare_parameter<std::string>("csv_file_path", "known_obs_coord.csv");
    this->declare_parameter<std::string>("obstacle_coordinates_frame_id", "map");
    this->declare_parameter<bool>("perform_downsampling", false);
    this->declare_parameter<std::string>("imu_topic", "/robot/imu/data");
    this->declare_parameter<bool>("perform_azimut_angle_filtering", false);
    this->declare_parameter<double>("minimal_azimut_angle", 5.0);
    this->declare_parameter<double>("maximal_azimut_angle", 5.0);

    lidar_topic_ = this->get_parameter("lidar_topic").as_string();
    cube_edge_length_ = this->get_parameter("cube_edge_length").as_double();
    z_lower_limit_ = this->get_parameter("z_lower_limit").as_double();
    x_lower_limit_ = this->get_parameter("x_lower_limit").as_double();
    x_upper_limit_ = this->get_parameter("x_upper_limit").as_double();
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    obstacle_coordinates_frame_id_ = this->get_parameter("obstacle_coordinates_frame_id").as_string();
    perform_downsampling_ = this->get_parameter("perform_downsampling").as_bool();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    perform_azimut_angle_filtering_ = this->get_parameter("perform_azimut_angle_filtering").as_bool();
    minimal_azimut_angle_ = degreesToRadians(this->get_parameter("minimal_azimut_angle").as_double());
    maximal_azimut_angle_ = degreesToRadians(this->get_parameter("maximal_azimut_angle").as_double());

    // LiDAR subscriber
    lidar_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, lidar_topic_);
    
    // IMU subscriber
    imu_subscriber_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, imu_topic_);

    // Synchronize LiDAR and IMU messages based on timestamps
    time_sync_ = std::make_shared<message_filters::Synchronizer<approximate_policy>>(
        approximate_policy(10),
        *lidar_subscriber_,
        *imu_subscriber_);
    time_sync_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(50000000));   // 100000000 ns = 0.1 seconds
    time_sync_->registerCallback(std::bind(&LidarFilteringNode::lidarImuCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Advertise the filtered point cloud topic
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);

    // TF Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Read obstacle coordinates from CSV file
    obstacle_polygons_ = readPolygonsFromFile(csv_file_path_);
    transformed_obstacle_polygons_ = obstacle_polygons_;  
  }

private:
  void lidarImuCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg, const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
  {
    // Measure the time before filtering
    auto total_start_time = std::chrono::high_resolution_clock::now();

    // Define the point type with Intensity field
    typedef pcl::PointXYZI PointType;

    // Convert ROS2 PointCloud2 to PCL PointCloud
    auto cloud = std::make_shared<pcl::PointCloud<PointType>>();
    pcl::fromROSMsg(*lidar_msg, *cloud);

    // Get the lidar frame for TF transforms
    std::string lidar_frame = cloud->header.frame_id; 

    // DEBUG
    auto partial_start_time = std::chrono::high_resolution_clock::now();

    auto rotated_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    if (!cloud->empty()){
        geometry_msgs::msg::QuaternionStamped imu_quaternion_msg;
        imu_quaternion_msg.header = imu_msg->header;
        imu_quaternion_msg.quaternion = imu_msg->orientation;
        try
        {
            // Transform the IMU message to the LiDAR frame
            imu_quaternion_msg = tf_buffer_->transform(imu_quaternion_msg, lidar_frame, tf2::durationFromSec(0.001));
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(get_logger(), "IMU msg transform failed: %s", ex.what());
            return;
        }
        // Extract roll and pitch angles from IMU data using
        // quaternion-to-Euler angle conversion formulas
        float roll, pitch;
        roll = atan2(2.0 * (imu_quaternion_msg.quaternion.w * imu_quaternion_msg.quaternion.x + 
                            imu_quaternion_msg.quaternion.y * imu_quaternion_msg.quaternion.z),
                    1.0 - 2.0 * (imu_quaternion_msg.quaternion.x * imu_quaternion_msg.quaternion.x +
                                imu_quaternion_msg.quaternion.y * imu_quaternion_msg.quaternion.y));
        pitch = asin(2.0 * (imu_quaternion_msg.quaternion.w * imu_quaternion_msg.quaternion.y - 
                            imu_quaternion_msg.quaternion.x * imu_quaternion_msg.quaternion.z));

        // RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f", (roll), (pitch));
        
        if (std::abs(roll) > 0.015 || pitch > 0.007){     // Avoid to perform rotation if roll and pitch are both small
            // Build a quaternion to align the LiDAR points with the ground
            Eigen::Quaternionf imu_quaternion = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
            // Apply rotation transformation based on IMU data
            pcl::transformPointCloud(*cloud, *rotated_cloud, Eigen::Vector3f::Zero(), imu_quaternion);
        } else {
            rotated_cloud = cloud;
        }
    } else {
        rotated_cloud = cloud;
    }

    // DEBUG
    auto partial_end_time = std::chrono::high_resolution_clock::now();

    auto filtered_cloud = std::make_shared<pcl::PointCloud<PointType>>();

    // Log the size of the original point cloud
    size_t original_size = cloud->size();

    // Create a Passthrough filter to remove points outside the cube
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(rotated_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_lower_limit_, x_upper_limit_);
    pass.filter(*filtered_cloud);

    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-cube_edge_length_ / 2.0, cube_edge_length_ / 2.0);
    pass.filter(*filtered_cloud);

    pass.setInputCloud(filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_lower_limit_, cube_edge_length_ / 2.0);
    pass.filter(*filtered_cloud);  

    // Perform downsampling using VoxelGrid
    auto downsampled_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    if (perform_downsampling_) {
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the voxel grid leaf size
        sor.filter(*downsampled_cloud);
    } else {
        downsampled_cloud = filtered_cloud;
    }

    // Filter points based on their azimut angle
    auto angle_filtered_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    if (perform_azimut_angle_filtering_){
        for (const auto& point : downsampled_cloud->points){
            double az_angle = std::atan2(point.y, point.x);
            if (az_angle >= minimal_azimut_angle_ && az_angle <= maximal_azimut_angle_){
                angle_filtered_cloud->points.push_back(point);
            }
        }
    } else {
        angle_filtered_cloud = downsampled_cloud;
    }

    // Transform obstacle coordinates to lidar frame
    geometry_msgs::msg::PointStamped obstacle_point_stamped;
    obstacle_point_stamped.header.frame_id = obstacle_coordinates_frame_id_;
    obstacle_point_stamped.header.stamp = lidar_msg->header.stamp;
    obstacle_point_stamped.point.z = 0.0;

    int n_polygons = obstacle_polygons_.size();
    for (int i = 0; i < n_polygons; i++) {
        int n_vertices = obstacle_polygons_[i].size();
        for (int j = 0; j < n_vertices; j++) {
            obstacle_point_stamped.point.x = obstacle_polygons_[i].vertices[j].x;
            obstacle_point_stamped.point.y = obstacle_polygons_[i].vertices[j].y;

            try {
                // Transform point to point cloud frame
                geometry_msgs::msg::PointStamped point_lidar_frame = tf_buffer_->transform(obstacle_point_stamped, lidar_frame, tf2::durationFromSec(0.05));
                transformed_obstacle_polygons_[i].vertices[j].x = point_lidar_frame.point.x;
                transformed_obstacle_polygons_[i].vertices[j].y = point_lidar_frame.point.y;
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(get_logger(), "Obstacle point transform error: %s", ex.what());
                return;
            }
        }
    }

    // Filter out lidar points inside the polygons
    auto no_known_obs_cloud = std::make_shared<pcl::PointCloud<PointType>>();

    for (const auto& point : angle_filtered_cloud->points) {
        Vertex vertex = {point.x, point.y};
        if (!isInsideAnyPolygon(vertex)) {
            no_known_obs_cloud->push_back(point);
        }
    }

    // Log the size of the filtered point cloud
    size_t filtered_size = no_known_obs_cloud->size();  

    // Convert PCL PointCloud to ROS2 PointCloud2
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*no_known_obs_cloud, *filtered_msg);

    // Fill in the missing PointCloud2 message fields
    filtered_msg->header.frame_id = cloud->header.frame_id;  // Copy the frame_id
    filtered_msg->header.stamp = this->get_clock()->now();  // Set the timestamp to the current ROS time in nanoseconds

    // Publish the filtered point cloud
    publisher_->publish(*filtered_msg);

    // Measure the time after filtering
    auto total_end_time = std::chrono::high_resolution_clock::now();
    // Calculate the duration
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(total_end_time - total_start_time);

    acc_total_duration += total_duration.count();
    ++n_total_duration;

    auto partial_duration = std::chrono::duration_cast<std::chrono::microseconds>(partial_end_time - partial_start_time);
    acc_partial_duration += partial_duration.count();
    ++n_partial_duration;

    RCLCPP_INFO(this->get_logger(), "Original size: %zu, Filtered size: %zu, Avg total callback duration: %lld microseconds",
             original_size, filtered_size, static_cast<long long>(acc_total_duration / n_total_duration));

    // RCLCPP_INFO(this->get_logger(), "Avg total callback duration: %lld microseconds, Avg partial callback duration: %lld microseconds",
    //        static_cast<long long>(acc_total_duration / n_total_duration), static_cast<long long>(acc_partial_duration / n_partial_duration));
  }

  std::vector<Polygon> readPolygonsFromFile(const std::string& filename) {
    std::vector<Polygon> polygons;
    std::ifstream file(filename);

    if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Error opening file: %s", filename.c_str());
        return polygons;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        Polygon polygon;
        double x, y;
        char comma;  // to capture the commas between values
        
        if (iss >> x >> comma >> y) {
            Vertex vertex = {x, y};
            polygon.vertices.push_back(vertex);
            
            // Read subsequent pairs with leading commas
            while (iss >> comma >> x >> comma >> y) {
                Vertex vertex = {x, y};
                polygon.vertices.push_back(vertex);
            }
        }

        if (polygon.size() >= 3) {
            polygons.push_back(polygon);
        } else {
            RCLCPP_ERROR(get_logger(), "Error: Each polygon must have at least three vertices.");
        }
    }

    file.close();
    return polygons;
  }

  bool isInsidePolygon(const Vertex& vertex, const Polygon& polygon) {
    // Algorithm from here: https://wrfranklin.org/Research/Short_Notes/pnpoly.html
    // Attention to how you specify the polygon coordinates; the order matters!
    int n = polygon.size();
    bool inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((polygon.vertices[i].y > vertex.y) != (polygon.vertices[j].y > vertex.y)) &&
            (vertex.x < (polygon.vertices[j].x - polygon.vertices[i].x) * (vertex.y- polygon.vertices[i].y) / 
                (polygon.vertices[j].y - polygon.vertices[i].y) + polygon.vertices[i].x)) {
            inside = !inside;
        }
    }

    return inside;
  }

  bool isInsideAnyPolygon(const Vertex& vertex) {
      for (const auto& polygon : transformed_obstacle_polygons_) {
          if (isInsidePolygon(vertex, polygon)) {
              return true;  // Point is inside at least one polygon
          }
      }
      return false;  // Point is outside all polygons
  }

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_subscriber_{nullptr};
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_subscriber_{nullptr};

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu> approximate_policy;
  std::shared_ptr<message_filters::Synchronizer<approximate_policy>> time_sync_{nullptr};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::vector<Polygon> obstacle_polygons_;
  std::vector<Polygon> transformed_obstacle_polygons_;
  Eigen::Matrix4f imu_rotation_matrix_;

  // Variables to store the parameters
  std::string lidar_topic_;
  double cube_edge_length_;
  double z_lower_limit_;
  double x_lower_limit_;
  double x_upper_limit_;
  std::string csv_file_path_;
  std::string obstacle_coordinates_frame_id_;
  bool perform_downsampling_;
  std::string imu_topic_;
  bool perform_azimut_angle_filtering_;
  double minimal_azimut_angle_;
  double maximal_azimut_angle_;

  // DEBUG
  float acc_roll;
  float acc_pitch;
  int n_roll;
  int n_pitch;
  double acc_total_duration;
  int n_total_duration;
  double acc_partial_duration;
  int n_partial_duration;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarFilteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
