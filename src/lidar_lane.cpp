//기본
#include <rclcpp/rclcpp.hpp>

//자료형
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/prosac.h>
#include <pcl/sample_consensus/sac_model_line.h>

//시각화
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//계산
#include <cmath>
#include <vector>


using PointT = pcl::PointXYZI;
using namespace std;

class LiDAR_lane : public rclcpp::Node
{
public:
    LiDAR_lane() : Node("LiDAR_lane")
    {
        mission_flag_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/Planning/mission", 1,
        [this](const std_msgs::msg::Int16::SharedPtr msg) -> void
        {
            mission_flag_cb(msg);
        });

        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&LiDAR_lane::timer_callback, this));

        lane_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/center_lane", 1);
        lane_dot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/lane_dot", 1);
        coord_dot_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Marker/ERP_center_dot", 1);
        lane_dot_MA = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/lane", 1);
        center_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/center_lane", 1);
        wall_distance = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/wall_dist", 1);
        ceiling_end_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/LiDAR/ceiling_end", 1);
        pub_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_line", 1);
        pub_ceiling = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_ceiling", 1);
        wall_cluster = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Cluster/tunnel_wall", 1);
        zone_center = this->create_publisher<visualization_msgs::msg::Marker>("/Marker/orange_lane_zone", 1);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "velodyne_points", 10, std::bind(&LiDAR_lane::cloud_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "LiDAR_lane node has started.");
    }

private:
    int mission_flag = 0;
    float lane_intensity_threshold = 70.0;

    int mission_check = -1; // 미션 들어오는 확인하는 변수
    int mission_count = 0; // 몇 번 동안 띄울지 결정하는 함수

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ceiling;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lane_dot_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coord_dot_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lane_dot_MA;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wall_distance;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ceiling_end_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_cluster;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr zone_center;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_flag_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        if (mission_check == -1)
        {
            mission_count = 0;
            RCLCPP_INFO(this->get_logger(), "\n 🚨🚨 터널 아닌가 ? 터널 아닌가 ? 🚨🚨");
        }
        else if ( mission_check > 0 && mission_count < 1)
        {
            RCLCPP_INFO(this->get_logger(), "\n 터널 안 인가 ? 🥇 터널 안 인가 ? 🥇");
            mission_count = 1;
        }
        mission_check = -1;
    }

    void mission_flag_cb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        this->mission_flag = msg->data;
        if (mission_flag == 15 || mission_flag == 17)
        {
            mission_check = 1;
        }
    }

    void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        if (mission_flag == 15 || mission_flag == 17)
        {
            // sensor_msgs/PointCloud2 data 를 pcl/PointCloud로 변환
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*input, *cloud);

/////////////////////////////////<벽 보는 코드>////////////////////////////////////////////////////////////////////
            // VoxelGrid filter를 이용하여 다운샘플링
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>());
            down_sampling(cloud, cloud_downsampled);

            // crop the wall 
            pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            seethewall(cloud_downsampled, wall_filtered);

            // 벽을 정사영하여 x,y만 남김
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>());
            project_to_2d_plane(wall_filtered, cloud_projected);

            // 클러스터링 및 가장 긴 두 개의 클러스터 선택
            pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            find_two_largest_clusters(cloud_projected, largest_clusters_cloud);

            // // 클러스터를 이용하여 벽의 벡터를 RANSAC으로 생성
            vector_wall(largest_clusters_cloud);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////<중앙 차선 코드>////////////////////////////////////////////////////////////////////
            // crop the ground
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
            cropGround(cloud, cloud_filtered2);

            // Detect lanes using intensity values
            pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud(new pcl::PointCloud<pcl::PointXYZI>());
            extractLanesByIntensity(cloud_filtered2, lanes_cloud, lane_intensity_threshold);

            clusterLanes(lanes_cloud, 0.2, 3, 50); //여기 바꿔
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////<천장 보는 코드>//////////////////////////////////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            I_want_know_tunnel_end(cloud, ceiling_filtered);
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected(new pcl::PointCloud<pcl::PointXYZI>);
            clustering_tunnel_ceiling(ceiling_filtered, ceiling_projected);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            // 정사영 된 포인트 클라우드를 출력으로 바꾸어 publish
            sensor_msgs::msg::PointCloud2 output_wall;
            pcl::toROSMsg(*largest_clusters_cloud, output_wall);
            output_wall.header = input->header;
            wall_cluster->publish(output_wall);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.ns = "my_roi";
            marker.id = 0;
            marker.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = 8;
            marker.scale.y = 4;
            marker.scale.z = 1;
            marker.pose.position.x = 4;
            marker.pose.position.y = 2;
            marker.pose.position.z = -0.5;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3;
            zone_center->publish(marker);  
        }
    }

    void down_sampling(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled)
    {
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.filter(*cloud_downsampled);
    }

    void cropGround(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2)
    {

        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        
        cropFilter.setMin(Eigen::Vector4f(-0, -10, -1.7, 1));  // 여기 바꿔
        cropFilter.setMax(Eigen::Vector4f(8, 10, -0.45, 1));   
        
        cropFilter.filter(*cloud_filtered2);
    }

    void seethewall(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr wall_filtered)
    {
        // Remove points close to the ground plane
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        
        cropFilter.setMin(Eigen::Vector4f(-20, -20, 0.8, 0)); // x, y, z, min (m)
        cropFilter.setMax(Eigen::Vector4f(100, 20, 3.0, 0)); 
        // cropFilter.setMax(Eigen::Vector4f(100, 20, 2.0, 0)); // 1008 산학테스트
        
        cropFilter.filter(*wall_filtered);
    }

    void I_want_know_tunnel_end(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected)
    {

        // Remove points close to the ground plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud);
        
        cropFilter.setMin(Eigen::Vector4f(-0, -30, 4.5, 1)); 
        cropFilter.setMax(Eigen::Vector4f(100, 30, 10.0, 1));   
        
        cropFilter.filter(*ceiling_filtered);

        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
        projection_matrix(2, 2) = 0;

        // 정사영 변환을 적용합니다.
        pcl::transformPointCloud(*ceiling_filtered, *ceiling_projected, projection_matrix);
    }

    void clustering_tunnel_ceiling(pcl::PointCloud<pcl::PointXYZI>::Ptr ceiling_projected, pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_projected)
    {
         // 클러스터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(ceiling_projected);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> ceiling_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(20.0);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(ceiling_projected);
        ec.extract(ceiling_indices);

        int ii = 0; 
        pcl::PointCloud<PointT> TotalCloud;
        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (vector<pcl::PointIndices>::const_iterator it = ceiling_indices.begin(); it != ceiling_indices.end(); ++it, ++ii)
        { 
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){   
                cluster->points.push_back(ceiling_projected->points[*pit]); PointT pt = ceiling_projected->points[*pit]; 
                PointT pt2; pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z; pt2.intensity = (float)(ii + 1); TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size(); cluster->height = 1; cluster->is_dense = true; clusters.push_back(cluster);
        }

        pcl::PCLPointCloud2 cloud_p; pcl::toPCLPointCloud2(TotalCloud, cloud_p); 
        sensor_msgs::msg::PointCloud2 clu; pcl_conversions::fromPCL(cloud_p, clu); 
        clu.header.frame_id = "velodyne"; pub_ceiling->publish(clu);

        std_msgs::msg::Float64MultiArray ceiling_end;
        for (int i = 0; i < int(clusters.size()); i++)
        {
            Eigen::Vector4f centroid, min_p, max_p;
            pcl::compute3DCentroid(*clusters[i], centroid);
            pcl::getMinMax3D(*clusters[i], min_p, max_p); 
            
            geometry_msgs::msg::Point center_point; 
            center_point.x = centroid[0]; center_point.y = centroid[1]; center_point.z = centroid[2];
            
            geometry_msgs::msg::Point min_point; 
            min_point.x = min_p[0]; min_point.y = min_p[1]; min_point.z = min_p[2];
            
            geometry_msgs::msg::Point max_point; 
            max_point.x = max_p[0]; max_point.y = max_p[1]; max_point.z = max_p[2];
            float max_px = max_point.x;
            ceiling_end.data.push_back(max_px);
        }

        ceiling_end_->publish(ceiling_end);
        ceiling_end.data.clear();
    }

    void project_to_2d_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected)
    {
        // Z 축 값이 0이 되도록 변환 행렬을 생성합니다.
        Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
        projection_matrix(2, 2) = 0;

        // 정사영 변환을 적용합니다.
        pcl::transformPointCloud(*cloud, *cloud_projected, projection_matrix);
    }

    void find_two_largest_clusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    {
        // 클러스터링을 위한 KDTree 오브젝트 생성
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(cloud);

        // 클러스터링 결과를 저장할 벡터
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        // ec.setMaxClusterSize(25000);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        
        // 클러스터의 길이를 저장할 벡터
        std::vector<std::pair<size_t, size_t>> cluster_lengths;

        // 클러스터의 길이 계산
        for (size_t i = 0; i < cluster_indices.size(); ++i){
            cluster_lengths.emplace_back(cluster_indices[i].indices.size(), i);
        }

        // 클러스터 길이를 기준으로 정렬(내림차순)
        std::sort(cluster_lengths.begin(), cluster_lengths.end(), std::greater<>());
        size_t num_clusters_to_publish = std::min<size_t>(2, cluster_lengths.size());

        // 가장 긴 두 개의 클러스터에 속한 포인트를 하나의 포인트 클라우드로 병합
        for (size_t i = 0; i < num_clusters_to_publish; ++i) {
            size_t cluster_idx = cluster_lengths[i].second;
            for (const auto& point_idx : cluster_indices[cluster_idx].indices) {
                largest_clusters_cloud->push_back((*cloud)[point_idx]);
            }
        }
    }

    void vector_wall(pcl::PointCloud<pcl::PointXYZI>::Ptr largest_clusters_cloud)
    {
        // Create KD-tree object for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
        kdtree->setInputCloud(largest_clusters_cloud);

        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.5);
        ec.setMinClusterSize(10);
        //ec.setMaxClusterSize(50);
        ec.setSearchMethod(kdtree);
        ec.setInputCloud(largest_clusters_cloud);
        ec.extract(cluster_indices);

        std_msgs::msg::Float64MultiArray wall_d;

        // For each of the largest clusters...
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& index : indices.indices)
            {
                cluster->points.push_back(largest_clusters_cloud->points[index]);
            }

            // Find the min and max points along x-axis (or whichever axis represents your 'horizontal')
            float x_min = std::numeric_limits<float>::max();
            float y_at_x_min = 0;
            
            float x_max = std::numeric_limits<float>::min();
            float y_at_x_max = 0;

            
            for (const auto &point : cluster->points)
            {
                if(point.x < x_min) 
                {
                    x_min = point.x;
                    y_at_x_min = point.y;
                }
                
                if(point.x > x_max) 
                {
                    x_max = point.x;
                    y_at_x_max = point.y;
                }
                
            }
            wall_d.data.push_back(x_min);
            wall_d.data.push_back(y_at_x_min);
            wall_d.data.push_back(x_max);
            wall_d.data.push_back(y_at_x_max);
        }
        wall_distance->publish(wall_d);
        wall_d.data.clear();
    }

    void extractLanesByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2, pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud, float lane_intensity_threshold)
    {
        for (const auto &point : cloud_filtered2->points)
        {
            if (point.intensity >= lane_intensity_threshold)
            {
                lanes_cloud->points.push_back(point);
            }
        }

    }

    void clusterLanes(pcl::PointCloud<pcl::PointXYZI>::Ptr lanes_cloud, double tolerance, int min_size, int max_size)
    {
        // Create KD-tree for efficient clustering
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(lanes_cloud);

        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(lanes_cloud);
        
        // Extract the clusters out of the input data
        ec.extract(cluster_indices);

        int ii = 0; 
        pcl::PointCloud<PointT> TotalCloud;
        std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr>> clusters;
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it, ++ii)
        { 
            for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){   
                cluster->points.push_back(lanes_cloud->points[*pit]); PointT pt = lanes_cloud->points[*pit]; 
                PointT pt2; pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z; pt2.intensity = (float)(ii + 1); TotalCloud.push_back(pt2);
            }
            cluster->width = cluster->size(); cluster->height = 1; cluster->is_dense = true; clusters.push_back(cluster);
        }

        pcl::PCLPointCloud2 cloud_p; pcl::toPCLPointCloud2(TotalCloud, cloud_p); 
        sensor_msgs::msg::PointCloud2 clu; pcl_conversions::fromPCL(cloud_p, clu); 
        clu.header.frame_id = "velodyne"; pub_cluster->publish(clu);

        // Cluster centers and clusters for visualization
        pcl::PointCloud<pcl::PointXYZI>::Ptr centers(new pcl::PointCloud<pcl::PointXYZI>);
        
        visualization_msgs::msg::MarkerArray marker_array1;
        visualization_msgs::msg::MarkerArray marker_array2;
        visualization_msgs::msg::MarkerArray marker_array3;
        
        std_msgs::msg::ColorRGBA color;
        color.r = 1.f; 
        color.g = 0.f; 
        color.b = 0.f; 
        color.a = 1.f;

        std_msgs::msg::ColorRGBA color2;
        color2.r = 0.f; 
        color2.g = 0.f; 
        color2.b = 1.f; 
        color2.a = 1.f;

        std_msgs::msg::Float64MultiArray lane_coord;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*lanes_cloud, it->indices, centroid);

            pcl::PointXYZI center_point;
            center_point.x = centroid[0];
            center_point.y = centroid[1];
            center_point.z = centroid[2];

            lane_coord.data.push_back(center_point.x);
            lane_coord.data.push_back(center_point.y);

            centers->points.push_back(center_point);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id="velodyne";
            marker.ns="clusters";
            marker.id=it-cluster_indices.begin();
            marker.type=visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action=visualization_msgs::msg::Marker::ADD;
            marker.lifetime.nanosec = 100000000;

            geometry_msgs::msg::Point point_msg;
            point_msg.x=center_point.x;
            point_msg.y=center_point.y;
            point_msg.z=center_point.z;

            marker.points.push_back(point_msg);
            marker.colors.push_back(color);

            marker.scale.x=0.5;  
            marker.scale.y=0.5;  
            marker.scale.z=0.5;

            marker_array1.markers.push_back(marker);
        }

        lane_dot_MA->publish(lane_coord);
        lane_coord.data.clear();
        
        lane_dot_->publish(marker_array1);
        marker_array1.markers.clear(); 

        pcl::PointCloud<pcl::PointXYZI>::Ptr centercenters(new pcl::PointCloud<pcl::PointXYZI>); // 중앙선만 따로 뽑아낸 pointcloud
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(centers);
        
        cropFilter.setMin(Eigen::Vector4f(-0, 0, -1.7, 1));  // 여기 바꿔
        cropFilter.setMax(Eigen::Vector4f(8, 4, -0.45, 1));   
        
        cropFilter.filter(*centercenters);

        if (centercenters->points.size() > 3) // 여기 바꿔
        {
            std_msgs::msg::Float64MultiArray center_lane;
            std_msgs::msg::Float64MultiArray your_point;
            for(int i = 0; i < 1; ++i)
            {
                pcl::SampleConsensusModelLine<PointT>::Ptr line_model(new pcl::SampleConsensusModelLine<PointT>(centercenters));
                pcl::ProgressiveSampleConsensus<PointT> ransac(line_model); 
                ransac.setDistanceThreshold(0.5);  // 여기 바꿔
                std::vector<int> inliers_; 
                // printf("ransac 생성\n");
                if (ransac.computeModel()) 
                {
                    // printf("ransac 성공\n");
                    ransac.getInliers(inliers_);

                    bool isTooFar = false;

                    for (size_t j = 1; j < inliers_.size(); ++j)
                    {
                        const auto& prev_point = centercenters->points[inliers_[j-1]];
                        const auto& curr_point = centercenters->points[inliers_[j]];
                        
                        float segment_length = std::sqrt(std::pow(curr_point.x - prev_point.x, 2) +
                                                        std::pow(curr_point.y - prev_point.y, 2) +
                                                        std::pow(curr_point.z - prev_point.z, 2));

                        if (segment_length > 3.0)              
                        {
                            // printf("거리 너무 길어\n");
                            isTooFar = true;
                            break;
                        }
                    }
                    if (!isTooFar)
                    {
                        // printf("선 생성 해줄게\n");
                        // Create and add markers to visualize the line segment
                        visualization_msgs::msg::Marker line_marker;
                        line_marker.header.frame_id = "velodyne";
                        line_marker.ns ="clusters";
                        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP; 
                        line_marker.action = visualization_msgs::msg::Marker::ADD;
                        line_marker.scale.x = 0.2; 
                        line_marker.pose.orientation.w = 1.0;
                        line_marker.lifetime.nanosec = 100000000;
                                    
                        // Set the color of the line segment marker
                        std_msgs::msg::ColorRGBA color_line_segment;
                        color_line_segment.r=0.f;  
                        color_line_segment.g=1.f;  
                        color_line_segment.b=0.f;  
                        color_line_segment.a=1.0;

                        std::vector<geometry_msgs::msg::Point> points;
                        for (const auto &index : inliers_)
                        {
                            // printf("선에 좌표 부여 해줄게\n");
                            geometry_msgs::msg::Point point_msg;
                            point_msg.x = centercenters->points[index].x;
                            point_msg.y = centercenters->points[index].y;
                            point_msg.z = centercenters->points[index].z;

                            geometry_msgs::msg::Point point_yours;
                            point_yours.x = point_msg.x;
                            point_yours.y = point_msg.y - 2.0;
                            point_yours.z = point_msg.z; 

                            points.push_back(point_yours);

                            line_marker.points.push_back(point_msg);
                            line_marker.colors.push_back(color_line_segment);

                            // Assign a unique id for each cluster
                            static int marker_id_counter = 0;
                            line_marker.id = marker_id_counter++;

                            visualization_msgs::msg::Marker marker_yours;
                            marker_yours.header.frame_id="velodyne";
                            marker_yours.ns="coord";
                            marker_yours.id=marker_id_counter++;
                            marker_yours.type=visualization_msgs::msg::Marker::SPHERE_LIST;
                            marker_yours.action=visualization_msgs::msg::Marker::ADD;
                            marker_yours.lifetime.nanosec = 100000000;

                            geometry_msgs::msg::Point point_coord;
                            point_coord.x=point_yours.x;
                            point_coord.y=point_yours.y;
                            point_coord.z=point_yours.z;

                            marker_yours.points.push_back(point_coord);
                            marker_yours.colors.push_back(color2);

                            marker_yours.scale.x=0.5;  
                            marker_yours.scale.y=0.5;  
                            marker_yours.scale.z=0.5;

                            marker_array3.markers.push_back(marker_yours);
                        }

                        std::sort(points.begin(), points.end(), [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
                            return a.x < b.x;
                        });

                        for (const auto& sorted_point : points)
                        {
                            center_lane.data.push_back(sorted_point.x);
                            center_lane.data.push_back(sorted_point.y);
                        }
                        center_->publish(center_lane);
                        center_lane.data.clear();

                        marker_array2.markers.push_back(line_marker);
                        lane_->publish(marker_array2);
                        marker_array2.markers.clear();

                        coord_dot_->publish(marker_array3);
                        marker_array3.markers.clear();  

                    }
                }
            }
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDAR_lane>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}