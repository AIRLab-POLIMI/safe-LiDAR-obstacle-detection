#include <random>
#include <cmath>    // Include the cmath library for rounding functions

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/common/centroid.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ch2_msg_srv/srv/dump_centroids.hpp"    // Include my custom service header
#include "ch2_msg_srv/msg/obstacle_detection.hpp"    // Include my custom message header


struct Obstacle {
    int n_updates = 0;
    geometry_msgs::msg::PointStamped acc_position;  // variable to accumulate the position coordinates
                                                    // and perform the average dividing by n_updates
};


class ClusteringNode : public rclcpp::Node {
public:
    ClusteringNode() : Node("clustering_node") {
        // Retrieve the parameter values from the parameter server
        this->declare_parameter<std::string>("filtered_pointcloud_topic", "/filtered_pointcloud");
        this->declare_parameter<std::string>("obstacle_coordinates_frame_id", "map");
        this->declare_parameter<int>("min_cluster_size", 40);
        this->declare_parameter<bool>("perform_outlier_removal", false);
        this->declare_parameter<int>("sor_mean_K", 10);
        this->declare_parameter<double>("obs_radius_threshold", 0.3);
        this->declare_parameter<double>("cluster_tolerance", 0.3);

        filtered_pointcloud_topic_ = this->get_parameter("filtered_pointcloud_topic").as_string();
        obstacle_coordinates_frame_id_ = this->get_parameter("obstacle_coordinates_frame_id").as_string();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        perform_outlier_removal_ = this->get_parameter("perform_outlier_removal").as_bool();
        sor_mean_K_ = this->get_parameter("sor_mean_K").as_int();
        obs_radius_threshold_ = this->get_parameter("obs_radius_threshold").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();

        // Subscribe to the filtered point cloud topic
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            filtered_pointcloud_topic_, 1, std::bind(&ClusteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Advertise the clustered point cloud topic and centroids topic
        clustered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_pointcloud", 1);
        centroids_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("cluster_centroids", 1);
        obstacles_publisher_ = this->create_publisher<ch2_msg_srv::msg::ObstacleDetection>("/evaluation/obstacle_detection", 1);

        // TF Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Declare the service to dump centroids
        dump_centroids_service_ = this->create_service<ch2_msg_srv::srv::DumpCentroids>(
            "dump_centroids", std::bind(&ClusteringNode::dumpCentroidsCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud) {
        // Define the point type with Intensity field
        typedef pcl::PointXYZI PointType;

        // Convert ROS PointCloud2 to PCL PointCloud
        auto pcl_cloud = std::make_shared<pcl::PointCloud<PointType>>();
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        
        if (perform_outlier_removal_){
            // Perform Statistical Outlier Removal
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setInputCloud(pcl_cloud);
            sor.setMeanK(sor_mean_K_);               
            sor.setStddevMulThresh(1.0);    // Adjust as needed
            sor.filter(*pcl_cloud);
        }
        
        // Euclidean Cluster Extraction
        auto kd_tree = std::make_shared<pcl::search::KdTree<PointType>>();

        kd_tree->setInputCloud(pcl_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> euclidean_cluster;
        euclidean_cluster.setClusterTolerance(cluster_tolerance_);
        euclidean_cluster.setMinClusterSize(min_cluster_size_);        
        euclidean_cluster.setMaxClusterSize(10000);                 // Adjust max cluster size as needed
        euclidean_cluster.setSearchMethod(kd_tree);
        euclidean_cluster.setInputCloud(pcl_cloud);
        euclidean_cluster.extract(cluster_indices);

        // Print INFO message with the number of clusters
        RCLCPP_INFO(get_logger(), "Number of clusters found: %zu", cluster_indices.size());

        // Create a random number generator for cluster colors
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(0.0, 1.0);

        // Create a new point cloud for colored clusters
        auto colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        // Create a new message for centroids
        geometry_msgs::msg::PointStamped centroids_msg;
        centroids_msg.header = pcl_conversions::fromPCL(pcl_cloud->header);
        centroids_msg.header.frame_id = obstacle_coordinates_frame_id_;

        // Get the transform at the time the pointcloud was generated
        geometry_msgs::msg::TransformStamped pc_to_obstacle_frame_tf;
        try {
            pc_to_obstacle_frame_tf = tf_buffer_->lookupTransform(obstacle_coordinates_frame_id_, pcl_cloud->header.frame_id, centroids_msg.header.stamp);
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "Transform lookup failed: %s", ex.what());
            return;
        }

        // Iterate through each cluster
        std::vector<geometry_msgs::msg::PointStamped> current_centroids;   // vector to store centroids from a single scan
        for (std::size_t i = 0; i < cluster_indices.size(); ++i) {
            // Print INFO message with the size of each cluster
            RCLCPP_INFO(get_logger(), "Cluster %zu size: %zu", i, cluster_indices[i].indices.size());

            // Assign a random color to the cluster
            pcl::PointXYZRGB color;
            color.r = static_cast<uint8_t>(255 * distribution(generator));
            color.g = static_cast<uint8_t>(255 * distribution(generator));
            color.b = static_cast<uint8_t>(255 * distribution(generator));

            // Compute the centroid of the cluster
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_cloud, cluster_indices[i], centroid);

            // Add the centroid to the message
            geometry_msgs::msg::Point point;
            point.x = centroid[0];
            point.y = centroid[1];
            point.z = centroid[2];
            centroids_msg.point = point;

            geometry_msgs::msg::PointStamped detection_point_stamped;
            tf2::doTransform(centroids_msg, detection_point_stamped, pc_to_obstacle_frame_tf);
            
            // Publish the centroids message
            centroids_publisher_->publish(detection_point_stamped);
            current_centroids.push_back(detection_point_stamped);

            // Approximate the coordinates to centimeter accuracy
            detection_point_stamped.point.x = std::round(detection_point_stamped.point.x * 100.0) / 100.0;
            detection_point_stamped.point.y = std::round(detection_point_stamped.point.y * 100.0) / 100.0;
            detection_point_stamped.point.z = std::round(detection_point_stamped.point.z * 100.0) / 100.0;
            // Add the centroid to the set
            unique_centroids_.insert(detection_point_stamped);

            // Iterate through each point in the cluster
            for (const auto& index : cluster_indices[i].indices) {
                PointType point = pcl_cloud->points[index];

                // Create a new colored point
                pcl::PointXYZRGB colored_point;
                colored_point.x = point.x;
                colored_point.y = point.y;
                colored_point.z = point.z;
                colored_point.r = color.r;
                colored_point.g = color.g;
                colored_point.b = color.b;

                // Add the colored point to the new point cloud
                colored_cloud->points.push_back(colored_point);
            }
        }

        for (const auto& curr_centroid : current_centroids) {
                // Check if the obstacle is near any previously detected obstacles
                bool found_nearby = false;
                int id;
                Obstacle curr_obstacle;
                for (auto& prev_obstacle : detected_obstacles) {
                    // Check if the distance between obstacles is within a certain radius
                    geometry_msgs::msg::Point prev_point;
                    prev_point.x = prev_obstacle.second.acc_position.point.x / prev_obstacle.second.n_updates;
                    prev_point.y = prev_obstacle.second.acc_position.point.y / prev_obstacle.second.n_updates;
                    prev_point.z = prev_obstacle.second.acc_position.point.z / prev_obstacle.second.n_updates;
                    double distance = calculateObstacleDistance(curr_centroid.point, prev_point);
                    if (distance < obs_radius_threshold_) {
                        // Assign the same ID
                        id = prev_obstacle.first;
                        curr_obstacle.n_updates = prev_obstacle.second.n_updates + 1;
                        curr_obstacle.acc_position.header = curr_centroid.header;
                        curr_obstacle.acc_position.point.x = prev_obstacle.second.acc_position.point.x + curr_centroid.point.x;
                        curr_obstacle.acc_position.point.y = prev_obstacle.second.acc_position.point.y + curr_centroid.point.y;
                        curr_obstacle.acc_position.point.z = prev_obstacle.second.acc_position.point.z + curr_centroid.point.z;
                        found_nearby = true;
                        break;
                    }
                }

                // If not found nearby, assign a new ID
                if (!found_nearby) {
                    id = generateNewObstacleId();
                    curr_obstacle.n_updates = 1;
                    curr_obstacle.acc_position = curr_centroid;
                }

                // Update the detected obstacles map
                detected_obstacles[id] = curr_obstacle;

                // Publish an obstacle position only if it has been seen at least three times
                if (curr_obstacle.n_updates >= 3) {
                    ch2_msg_srv::msg::ObstacleDetection obstacle_detection_msg;
                    obstacle_detection_msg.header = curr_obstacle.acc_position.header;
                    obstacle_detection_msg.id = std::to_string(id);
                    geometry_msgs::msg::Point obstacle_position;
                    obstacle_position.x = curr_obstacle.acc_position.point.x / curr_obstacle.n_updates;
                    obstacle_position.y = curr_obstacle.acc_position.point.y / curr_obstacle.n_updates;
                    obstacle_position.z = curr_obstacle.acc_position.point.z / curr_obstacle.n_updates;
                    obstacle_detection_msg.position = obstacle_position;
                    obstacles_publisher_->publish(obstacle_detection_msg);
                }
        }

        // Convert PCL PointCloud to ROS PointCloud2
        auto clustered_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*colored_cloud, *clustered_cloud_msg);

        // Fill in the missing PointCloud2 message fields
        clustered_cloud_msg->header.frame_id = pcl_cloud->header.frame_id;  // Copy the frame_id
        clustered_cloud_msg->header.stamp = this->get_clock()->now();   // Set the timestamp to the current ROS time in nanoseconds

        // Publish the clustered point cloud
        clustered_pointcloud_publisher_->publish(*clustered_cloud_msg);
    }

    int generateNewObstacleId() {
        return ++current_obstacle_id;
    }

    double calculateObstacleDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2) {
        double x_diff = p2.x - p1.x;
        double y_diff = p2.y - p1.y;

        // Calculate Euclidean distance
        double distance = std::sqrt(x_diff * x_diff + y_diff * y_diff);

        return distance;
    }

    void dumpCentroidsCallback(
        const std::shared_ptr<ch2_msg_srv::srv::DumpCentroids::Request> request,
        const std::shared_ptr<ch2_msg_srv::srv::DumpCentroids::Response> response)
    {
        // Convert geometry_msgs::msg::PointStamped to pcl::PointXYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : unique_centroids_) {
            pcl::PointXYZ pcl_point;
            pcl_point.x = point.point.x;
            pcl_point.y = point.point.y;
            pcl_point.z = point.point.z;
            pcl_cloud->push_back(pcl_point);
        }

        // Create a KdTree object for the search method of the extraction
        auto kd_tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();

        kd_tree->setInputCloud(pcl_cloud);

        // Set up Euclidean clustering parameters
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.35);    // Set the distance threshold for cluster creation
        ec.setMinClusterSize(3);        // Set the minimum number of points that a cluster should have
        ec.setMaxClusterSize(1000);     // Set the maximum number of points that a cluster should have
                                        // TODO: there is the need to set an expire time for the
                                        // centroids, otherwise the cluster size grows indefinetly
                                        // if the obstacles are always in the same position and seen many
                                        // times. At a certain point the obstacle won't be detected anymore 
                                        // because the cluster will reach the MaxClusterSize.
                                        // Since the unique_centroids_ are PointStamped, it should be easy
                                        // to implement such an expriration. 
        ec.setSearchMethod(kd_tree);
        ec.setInputCloud(pcl_cloud);

         // Extract clusters
        std::vector<pcl::PointIndices> cluster_indices;
        ec.extract(cluster_indices);

        // Extract the centroids of the clusters of centroids
        std::set<geometry_msgs::msg::PointStamped_<std::allocator<void>>, PointStampedComparator> cluster_centroids;
        for (const auto& indices : cluster_indices) {
            // Compute the centroid of the cluster
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pcl_cloud, indices, centroid);

            // Add the centroid to the message
            geometry_msgs::msg::PointStamped centroids_msg;
            centroids_msg.point.x = centroid[0];
            centroids_msg.point.y = centroid[1];
            centroids_msg.point.z = centroid[2];
            cluster_centroids.insert(centroids_msg);
        }

        // Check if the 'centroid' parameter is true
        if (request->centroid) {
            // Dump cluster centroids to a CSV file
            std::ofstream file("centroids_dump.csv");

            if (file.is_open()) {
                for (const auto& centroid : cluster_centroids) {
                    file << centroid.point.x << "," << centroid.point.y << "," << centroid.point.z << "\n";
                }

                file.close();
                response->success = true;
                RCLCPP_INFO(get_logger(), "Centroids dumped to centroids_dump.csv");
            } else {
                response->success = false;
                RCLCPP_ERROR(get_logger(), "Failed to open centroids_dump.csv for writing");
            }
        } else {
            // Dump the vertices of the convex hull surrounding all points in the cluster
            std::ofstream file("cluster_convex_hull_vertices_dump.csv");

            // TODO: insert implementation
            response->success = false;
            RCLCPP_ERROR(get_logger(), "No implementation for the polygon area computation");
        }
    }

    // Custom comparison function
    struct PointStampedComparator {
        bool operator()(const geometry_msgs::msg::PointStamped_<std::allocator<void>>& lhs,
                        const geometry_msgs::msg::PointStamped_<std::allocator<void>>& rhs) const {
            // Custom comparison logic
            // Compare based on x, y, z coordinates
            if (lhs.point.x != rhs.point.x) {
                return lhs.point.x < rhs.point.x;
            }
            if (lhs.point.y != rhs.point.y) {
                return lhs.point.y < rhs.point.y;
            }
            return lhs.point.z < rhs.point.z;
        }
    };

    std::set<geometry_msgs::msg::PointStamped_<std::allocator<void>>, PointStampedComparator> unique_centroids_; // Set to store unique centroids
    std::unordered_map<int, Obstacle> detected_obstacles;   // Unordered map to store the detected obstacle positions with id
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pointcloud_publisher_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroids_publisher_{nullptr};
    rclcpp::Publisher<ch2_msg_srv::msg::ObstacleDetection>::SharedPtr obstacles_publisher_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Service<ch2_msg_srv::srv::DumpCentroids>::SharedPtr dump_centroids_service_{nullptr};    // Service to dump centroids to a file
    std::string filtered_pointcloud_topic_;
    std::string obstacle_coordinates_frame_id_;
    int min_cluster_size_;
    bool perform_outlier_removal_;
    int sor_mean_K_;
    int current_obstacle_id = 0;
    double obs_radius_threshold_;
    double cluster_tolerance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusteringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

