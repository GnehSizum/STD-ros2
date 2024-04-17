#include <Eigen/Geometry>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "include/STDesc.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

std::mutex laser_mtx;
std::mutex odom_mtx;

PointCloud::Ptr current_cloud_body(new PointCloud);
PointCloud::Ptr current_cloud_world(new PointCloud);
PointCloud::Ptr key_cloud(new PointCloud);

ConfigSetting config_setting;

STDescManager *std_manager = new STDescManager(config_setting);

class STDOnlineNode : public rclcpp::Node {
 public:
  STDOnlineNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
    : Node("std_online_node", options), isam(parameters) {
    // pre-preocess
    this->declare_parameter<double>("ds_size", 0.5);
    this->declare_parameter<int>("maximum_corner_num", 100);
    // key points
    this->declare_parameter<double>("plane_merge_normal_thre", 0.1);
    this->declare_parameter<double>("plane_detection_thre", 0.01);
    this->declare_parameter<double>("voxel_size",  2.0);
    this->declare_parameter<int>("voxel_init_num",  10);
    this->declare_parameter<double>("proj_image_resolution", 0.5);
    this->declare_parameter<double>("proj_dis_min", 0);
    this->declare_parameter<double>("proj_dis_max", 2);
    this->declare_parameter<double>("corner_thre", 10);
    // std descriptor
    this->declare_parameter<int>("descriptor_near_num", 10);
    this->declare_parameter<double>("descriptor_min_len", 2);
    this->declare_parameter<double>("descriptor_max_len", 50);
    this->declare_parameter<double>("non_max_suppression_radius", 2.0);
    this->declare_parameter<double>("std_side_resolution", 0.2);
    // candidate search
    this->declare_parameter<int>("skip_near_num", 50);
    this->declare_parameter<int>("candidate_num", 50);
    this->declare_parameter<int>("sub_frame_num", 10);
    this->declare_parameter<double>("rough_dis_threshold", 0.01);
    this->declare_parameter<double>("vertex_diff_threshold", 0.5);
    this->declare_parameter<double>("icp_threshold", 0.5);
    this->declare_parameter<double>("normal_threshold", 0.2);
    this->declare_parameter<double>("dis_threshold", 0.5);

    // pre-preocess
    this->get_parameter<double>("ds_size", config_setting.ds_size_);
    this->get_parameter<int>("maximum_corner_num", config_setting.maximum_corner_num_);
    // key points
    this->get_parameter<double>("plane_merge_normal_thre", config_setting.plane_merge_normal_thre_);
    this->get_parameter<double>("plane_detection_thre", config_setting.plane_detection_thre_);
    this->get_parameter<double>("voxel_size", config_setting.voxel_size_);
    this->get_parameter<int>("voxel_init_num", config_setting.voxel_init_num_);
    this->get_parameter<double>("proj_image_resolution", config_setting.proj_image_resolution_);
    this->get_parameter<double>("proj_dis_min", config_setting.proj_dis_min_);
    this->get_parameter<double>("proj_dis_max", config_setting.proj_dis_max_);
    this->get_parameter<double>("corner_thre", config_setting.corner_thre_);
    // std descriptor
    this->get_parameter<int>("descriptor_near_num", config_setting.descriptor_near_num_);
    this->get_parameter<double>("descriptor_min_len", config_setting.descriptor_min_len_);
    this->get_parameter<double>("descriptor_max_len", config_setting.descriptor_max_len_);
    this->get_parameter<double>("non_max_suppression_radius", config_setting.non_max_suppression_radius_);
    this->get_parameter<double>("std_side_resolution", config_setting.std_side_resolution_);
    // candidate search
    this->get_parameter<int>("skip_near_num", config_setting.skip_near_num_);
    this->get_parameter<int>("candidate_num", config_setting.candidate_num_);
    this->get_parameter<int>("sub_frame_num", config_setting.sub_frame_num_);
    this->get_parameter<double>("rough_dis_threshold", config_setting.rough_dis_threshold_);
    this->get_parameter<double>("vertex_diff_threshold", config_setting.vertex_diff_threshold_);
    this->get_parameter<double>("icp_threshold", config_setting.icp_threshold_);
    this->get_parameter<double>("normal_threshold", config_setting.normal_threshold_);
    this->get_parameter<double>("dis_threshold", config_setting.dis_threshold_);

    std::cout << "Sucessfully load parameters:" << std::endl;
    std::cout << "----------------Main Parameters-------------------"
              << std::endl;
    std::cout << "voxel size:" << config_setting.voxel_size_ << std::endl;
    std::cout << "loop detection threshold: " << config_setting.icp_threshold_
              << std::endl;
    std::cout << "sub-frame number: " << config_setting.sub_frame_num_
              << std::endl;
    std::cout << "candidate number: " << config_setting.candidate_num_
              << std::endl;
    std::cout << "maximum corners size: " << config_setting.maximum_corner_num_
              << std::endl;
    
    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", 100, //rclcpp::SensorDataQoS(), 
        [this](const sensor_msgs::msg::PointCloud2::ConstPtr &msg) {
          this->laserCloudHandler(msg);
        });
    
    subOdom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 100, //rclcpp::SensorDataQoS(), 
        [this](const nav_msgs::msg::Odometry::ConstPtr &msg) {
          this->OdomHandler(msg);
        });
    
    pubCurrentCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_current", 100);
    pubCurrentCorner = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_key_points", 100);
    pubMatchedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_matched", 100);
    pubMatchedCorner = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_matched_key_points", 100);
    pubSTD = this->create_publisher<visualization_msgs::msg::MarkerArray>("/descriptor_line", 10);
    pubOriginCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_origin", 10000);
    pubCorrectCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_correct", 10000);
    pubCorrectPath = this->create_publisher<nav_msgs::msg::Path>("/correct_path", 100000);
    pubOdomOrigin = this->create_publisher<nav_msgs::msg::Odometry>("/odom_origin", 10);
    pubLoopConstraintEdge = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_closure_constraints", 10);

    auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    timer = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&STDOnlineNode::timerCallback, this));

    odometryNoise = gtsam::noiseModel::Diagonal::Variances(
                      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    priorNoise = gtsam::noiseModel::Diagonal::Variances(
                   (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
  
    double loopNoiseScore = 1e-1;
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
                          loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
                        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    
    last_pose.setIdentity();
  }

  ~STDOnlineNode() {}

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstPtr &msg) {
    std::unique_lock<std::mutex> lock(laser_mtx);
    laser_buffer.push(msg);
  }

  void OdomHandler(const nav_msgs::msg::Odometry::ConstPtr &msg) {
    std::unique_lock<std::mutex> lock(odom_mtx);
    odom_buffer.push(msg);
  }

  bool syncPackages(PointCloud::Ptr &cloud, Eigen::Affine3d &pose) {
    if (laser_buffer.empty() || odom_buffer.empty())
      return false;

    auto laser_msg = laser_buffer.front();
    double laser_timestamp = get_time_sec(laser_msg->header.stamp);

    auto odom_msg = odom_buffer.front();
    double odom_timestamp = get_time_sec(odom_msg->header.stamp);

    // check if timestamps are matched
    if (abs(odom_timestamp - laser_timestamp) < 1e-3) {
      pcl::fromROSMsg(*laser_msg, *cloud);

      Eigen::Quaterniond r(
          odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
          odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
      Eigen::Vector3d t(odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
                        odom_msg->pose.pose.position.z);

      pose = Eigen::Affine3d::Identity();
      pose.translate(t);
      pose.rotate(r);

      std::unique_lock<std::mutex> l_lock(laser_mtx);
      std::unique_lock<std::mutex> o_lock(odom_mtx);

      laser_buffer.pop();
      odom_buffer.pop();

    } else if (odom_timestamp < laser_timestamp) {
      printf("Current odometry is earlier than laser scan, discard one "
              "odometry data.");
      std::unique_lock<std::mutex> o_lock(odom_mtx);
      odom_buffer.pop();
      return false;
    } else {
      printf(
          "Current laser scan is earlier than odometry, discard one laser scan.");
      std::unique_lock<std::mutex> l_lock(laser_mtx);
      laser_buffer.pop();
      return false;
    }

    return true;
  }

  void update_poses(const gtsam::Values &estimates,
                    std::vector<Eigen::Affine3d> &poses) {
    assert(estimates.size() == poses.size());

    poses.clear();

    for (int i = 0; i < estimates.size(); ++i) {
      auto est = estimates.at<gtsam::Pose3>(i);
      Eigen::Affine3d est_affine3d(est.matrix());
      poses.push_back(est_affine3d);
    }
  }

  void visualizeLoopClosure(
      const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &publisher,
      const std::vector<std::pair<int, int>> &loop_container,
      const std::vector<Eigen::Affine3d> &key_pose_vec) {
    if (loop_container.empty())
      return;

    visualization_msgs::msg::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::msg::Marker markerNode;
    markerNode.header.frame_id = "camera_init"; // camera_init
    // markerNode.header.stamp = ros::Time().fromSec( keyframeTimes.back() );
    markerNode.action = visualization_msgs::msg::Marker::ADD;
    markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 闭环边
    visualization_msgs::msg::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    // markerEdge.header.stamp = ros::Time().fromSec( keyframeTimes.back() );
    markerEdge.action = visualization_msgs::msg::Marker::ADD;
    markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历闭环
    for (auto it = loop_container.begin(); it != loop_container.end(); ++it) {
      int key_cur = it->first;
      int key_pre = it->second;
      geometry_msgs::msg::Point p;
      p.x = key_pose_vec[key_cur].translation().x();
      p.y = key_pose_vec[key_cur].translation().y();
      p.z = key_pose_vec[key_cur].translation().z();
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
      p.x = key_pose_vec[key_pre].translation().x();
      p.y = key_pose_vec[key_pre].translation().y();
      p.z = key_pose_vec[key_pre].translation().z();
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    publisher->publish(markerArray);
  }

 private:
  void timerCallback()
  {
    if (syncPackages(current_cloud_body, pose)) {
      auto origin_estimate_affine3d = pose;
      pcl::transformPointCloud(*current_cloud_body, *current_cloud_world, pose);
      down_sampling_voxel(*current_cloud_world, config_setting.ds_size_);
      // down sample body cloud
      down_sampling_voxel(*current_cloud_body, 0.5);
      cloud_vec.push_back(current_cloud_body);
      pose_vec.push_back(pose);
      origin_pose_vec.push_back(pose);
      PointCloud origin_cloud;
      pcl::transformPointCloud(*current_cloud_body, origin_cloud,
                               origin_estimate_affine3d);
      sensor_msgs::msg::PointCloud2 pub_cloud;
      pcl::toROSMsg(origin_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubOriginCloud->publish(pub_cloud);

      Eigen::Quaterniond _r(origin_estimate_affine3d.rotation());
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "camera_init";
      odom.pose.pose.position.x = origin_estimate_affine3d.translation().x();
      odom.pose.pose.position.y = origin_estimate_affine3d.translation().y();
      odom.pose.pose.position.z = origin_estimate_affine3d.translation().z();
      odom.pose.pose.orientation.w = _r.w();
      odom.pose.pose.orientation.x = _r.x();
      odom.pose.pose.orientation.y = _r.y();
      odom.pose.pose.orientation.z = _r.z();
      pubOdomOrigin->publish(odom);

      *key_cloud += *current_cloud_world;
      initial.insert(cloudInd, gtsam::Pose3(pose.matrix()));

      if (!cloudInd) {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            0, gtsam::Pose3(pose.matrix()), odometryNoise));
      } else {
        auto prev_pose = gtsam::Pose3(origin_pose_vec[cloudInd - 1].matrix());
        auto curr_pose = gtsam::Pose3(pose.matrix());
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            cloudInd - 1, cloudInd, prev_pose.between(curr_pose),
            odometryNoise));
      }

      // RCLCPP_INFO(this->get_logger(), 
      //             "key cloud size: [%d]",
      //             (int)key_cloud->size());

      // check if keyframe
      if (cloudInd % config_setting.sub_frame_num_ == 0 && cloudInd != 0) {
        RCLCPP_INFO(this->get_logger(), 
                    "key frame idx: [%d], key cloud size: [%d]",
                    (int)keyCloudInd, (int)key_cloud->size());
        // step1. Descriptor Extraction
        std::vector<STDesc> stds_vec;
        std_manager->GenerateSTDescs(key_cloud, stds_vec);

        // step2. Searching Loop
        std::pair<int, double> search_result(-1, 0);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<STDesc, STDesc>> loop_std_pair;

        if (keyCloudInd > config_setting.skip_near_num_) {
          std_manager->SearchLoop(stds_vec, search_result, loop_transform,
                                  loop_std_pair);
        }

        // step3. Add descriptors to the database
        std_manager->AddSTDescs(stds_vec);

        // publish
        sensor_msgs::msg::PointCloud2 pub_cloud;
        pcl::toROSMsg(*key_cloud, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCloud->publish(pub_cloud);
        pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCurrentCorner->publish(pub_cloud);

        std_manager->key_cloud_vec_.push_back(key_cloud->makeShared());

        if (search_result.first > 0) {
          std::cout << "[Loop Detection] triggle loop: " << keyCloudInd << "--"
                    << search_result.first << ", score:" << search_result.second
                    << std::endl;

          has_loop_flag = true;
          int match_frame = search_result.first;
          // obtain optimal transform
          std_manager->PlaneGeomrtricIcp(
              std_manager->plane_cloud_vec_.back(),
              std_manager->plane_cloud_vec_[match_frame], loop_transform);

          // std::cout << "delta transform:" << std::endl;
          // std::cout << "translation: " << loop_transform.first.transpose() <<
          // std::endl;

          // auto euler = loop_transform.second.eulerAngles(2, 1, 0) * 57.3;
          // std::cout << "rotation(ypr): " << euler[0] << ' ' << euler[1] << '
          // ' << euler[2]
          //           << std::endl;

          /*
            add connection between loop frame.
            e.g. if src_key_frame_id 5 with sub frames 51~60 triggle loop with
                  tar_key_frame_id 1 with sub frames 11~20, add connection
            between each sub frame, 51-11, 52-12,...,60-20.

          */
          int sub_frame_num = config_setting.sub_frame_num_;
          for (size_t j = 1; j <= sub_frame_num; j++) {
            int src_frame = cloudInd + j - sub_frame_num;

            auto delta_T = Eigen::Affine3d::Identity();
            delta_T.translate(loop_transform.first);
            delta_T.rotate(loop_transform.second);
            Eigen::Affine3d src_pose_refined = delta_T * pose_vec[src_frame];

            int tar_frame = match_frame * sub_frame_num + j;
            // old
            // Eigen::Affine3d tar_pose = pose_vec[tar_frame];
            Eigen::Affine3d tar_pose = origin_pose_vec[tar_frame];

            loop_container.push_back({tar_frame, src_frame});

            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                tar_frame, src_frame,
                gtsam::Pose3(tar_pose.matrix())
                    .between(gtsam::Pose3(src_pose_refined.matrix())),
                robustLoopNoise));
          }

          pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCloud->publish(pub_cloud);

          pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first],
                        pub_cloud);
          pub_cloud.header.frame_id = "camera_init";
          pubMatchedCorner->publish(pub_cloud);
          publish_std_pairs(loop_std_pair, pubSTD);
        }

        key_cloud->clear();
        ++keyCloudInd;
      }
      isam.update(graph, initial);
      isam.update();

      if (has_loop_flag) {
        isam.update();
        isam.update();
        isam.update();
        isam.update();
        isam.update();
      }

      graph.resize(0);
      initial.clear();

      curr_estimate = isam.calculateEstimate();
      update_poses(curr_estimate, pose_vec);

      auto latest_estimate_affine3d = pose_vec.back();

      if (has_loop_flag) {
        // publish correct cloud map
        PointCloud full_map;
        for (int i = 0; i < pose_vec.size(); ++i) {
          PointCloud correct_cloud;
          pcl::transformPointCloud(*cloud_vec[i], correct_cloud, pose_vec[i]);
          full_map += correct_cloud;
        }
        sensor_msgs::msg::PointCloud2 pub_cloud;
        pcl::toROSMsg(full_map, pub_cloud);
        pub_cloud.header.frame_id = "camera_init";
        pubCorrectCloud->publish(pub_cloud);

        // publish corerct path
        nav_msgs::msg::Path correct_path;
        for (int i = 0; i < pose_vec.size(); i += 1) {

          geometry_msgs::msg::PoseStamped msg_pose;
          msg_pose.pose.position.x = pose_vec[i].translation()[0];
          msg_pose.pose.position.y = pose_vec[i].translation()[1];
          msg_pose.pose.position.z = pose_vec[i].translation()[2];
          Eigen::Quaterniond pose_q(pose_vec[i].rotation());
          msg_pose.header.frame_id = "camera_init";
          msg_pose.pose.orientation.x = pose_q.x();
          msg_pose.pose.orientation.y = pose_q.y();
          msg_pose.pose.orientation.z = pose_q.z();
          msg_pose.pose.orientation.w = pose_q.w();
          correct_path.poses.push_back(msg_pose);
        }
        correct_path.header.stamp = this->get_clock()->now();
        correct_path.header.frame_id = "camera_init";
        pubCorrectPath->publish(correct_path);
      }
      //   PointCloud correct_cloud;
      //   pcl::transformPointCloud(*current_cloud_body, correct_cloud,
      //                            latest_estimate_affine3d);
      //   sensor_msgs::msg::PointCloud2 pub_cloud;
      //   pcl::toROSMsg(correct_cloud, pub_cloud);
      //   pub_cloud.header.frame_id = "camera_init";
      //   pubCorrectCloud->publish(pub_cloud);

      visualizeLoopClosure(pubLoopConstraintEdge, loop_container, pose_vec);

      has_loop_flag = false;
      ++cloudInd;
    }
  }

  std::queue<sensor_msgs::msg::PointCloud2::ConstPtr> laser_buffer;
  std::queue<nav_msgs::msg::Odometry::ConstPtr> odom_buffer;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCurrentCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCurrentCorner;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMatchedCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMatchedCorner;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubSTD;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOriginCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCorrectCloud;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubCorrectPath;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomOrigin;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

  rclcpp::TimerBase::SharedPtr timer;

  size_t cloudInd = 0;
  size_t keyCloudInd = 0;

  std::vector<PointCloud::Ptr> cloud_vec;
  std::vector<Eigen::Affine3d> pose_vec;
  std::vector<Eigen::Affine3d> origin_pose_vec;
  std::vector<Eigen::Affine3d> key_pose_vec;
  std::vector<std::pair<int, int>> loop_container;

  bool has_loop_flag = false;
  gtsam::Values curr_estimate;

  gtsam::Values initial;
  gtsam::NonlinearFactorGraph graph;
  gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
  gtsam::noiseModel::Base::shared_ptr robustLoopNoise;
  
  gtsam::ISAM2Params parameters;
  gtsam::ISAM2 isam;

  Eigen::Affine3d pose;
  Eigen::Affine3d last_pose;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<STDOnlineNode>());

  if (rclcpp::ok())
      rclcpp::shutdown();

  // You can save full map with refined pose
  // assert(cloud_vec.size() == pose_vec.size());
  // PointCloud full_map;
  // for (int i = 0; i < pose_vec.size(); ++i) {
  //     PointCloud correct_cloud;
  //     pcl::transformPointCloud(*cloud_vec[i], correct_cloud, pose_vec[i]);
  //     full_map += correct_cloud;
  // }
  // down_sampling_voxel(full_map, 0.05);

  // std::cout << "saving map..." << std::endl;
  // pcl::io::savePCDFileBinary("/home/dustier/data/map.pcd", full_map);

  return 0;
}
