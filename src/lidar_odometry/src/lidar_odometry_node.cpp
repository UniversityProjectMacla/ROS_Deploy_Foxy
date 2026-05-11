/**
 * Lidar odometry using PCL Normal Distributions Transform (NDT).
 *
 * registration_mode = "scan_to_scan":
 *   Consecutive scans; same as original behavior.
 *
 * registration_mode = "scan_to_map":
 *   Maintains a voxel map in odom (accumulated aligned scans).
 *   Default high-performance path (scan_to_map_register_sensor_frame=true): source = filtered scan in
 *   **sensor** frame, target = map in **odom**, ndt.align(..., T_odom_sensor_pred) — full 3D T_ndt kept
 *   internally; global_pose_full_ is kept planar (SE2 in odom) after each scan_to_map update;
 *   global_pose_ matches it for publish/2D. Optional
 *   yaw blend (ndt_fuse_prior_planar_yaw + ndt_prior_yaw_blend) and corridor degeneracy check.
 *   Map merge uses transformPointCloud(..., T_odom_sensor_new). Legacy path: T_ndt * T_pred. Optional
 *   coarse NDT, keyframe merge, opposite-motion / tiny-correction fallbacks.
 *   Optional ndt_gate_until_prior_translation_m: skip NDT until planar EKF prior |xy| exceeds threshold
 *   (reduces bad startup alignment when TF is not yet stable).
 *
 * Motion prior for scan_to_map (parameter use_tf_initial_guess, default true):
 *   TF lookup odom_frame -> base_frame at the cloud stamp (e.g. from ekf_node). If that stamp is
 *   not in the buffer yet (NDT often receives the cloud before ekf_node's same-topic callback
 *   publishes odom->base_link at that time), falls back to latest TF, then to last integrated pose.
 *
 * Subscribes: sensor_msgs/PointCloud2
 * Publishes:
 *   - nav_msgs/Odometry : integrated planar pose in odom_frame -> base_frame
 *   - geometry_msgs/TwistStamped : planar step in **odom** (Δx, Δy, Δθ); header.frame_id = odom_frame
 *   - geometry_msgs/PoseStamped (scan_to_map only): NDT planar correction vs prediction
 */
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace lidar_odometry {

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Inline replacement for tf2::transformToEigen (not reliably shipped on Foxy).
static inline Eigen::Isometry3d transformToEigen(const geometry_msgs::msg::Transform &t) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() << t.translation.x, t.translation.y, t.translation.z;
  out.linear() = Eigen::Quaterniond(
      t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).toRotationMatrix();
  return out;
}

static float yawFrom2DBlock(const Eigen::Matrix4f &T) {
  return std::atan2(T.coeff(1, 0), T.coeff(0, 0));
}

static float wrapAnglePi(float a) {
  return std::atan2(std::sin(a), std::cos(a));
}

/**
 * PCL 1.12 NDT builds the target with VoxelGridCovariance but does not expose
 * setMinPointPerVoxel / setCovEigValueInflationRatio. Without those, fine resolution +
 * sparse Livox-style cells often yield singular covariances and repeated
 * "[VoxelGridCovariance::applyFilter] Invalid eigen value! (0, 0, 0)" warnings.
 */
class ConfigurableNdt : public pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> {
 public:
  using Base = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
  using PointCloudTargetConstPtr = Base::PointCloudTargetConstPtr;

  void setVoxelMinPointsPerVoxel(int n) { voxel_min_points_ = std::max(3, n); }
  void setVoxelCovEigInflationRatio(double r) { voxel_cov_eig_inflation_ratio_ = r; }

  void setInputTarget(const PointCloudTargetConstPtr &cloud) override {
    pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>::setInputTarget(cloud);
    rebuildTargetGrid();
  }

  void setResolution(float resolution) {
    if (resolution_ != resolution) {
      resolution_ = resolution;
      if (input_) {
        rebuildTargetGrid();
      }
    }
  }

 private:
  void rebuildTargetGrid() {
    if (!target_) {
      return;
    }
    target_cells_.setLeafSize(resolution_, resolution_, resolution_);
    target_cells_.setMinPointPerVoxel(voxel_min_points_);
    target_cells_.setCovEigValueInflationRatio(voxel_cov_eig_inflation_ratio_);
    target_cells_.setInputCloud(target_);
    target_cells_.filter(true);
  }

  int voxel_min_points_{10};
  double voxel_cov_eig_inflation_ratio_{0.05};
};

/**
 * NDT returns full 6-DOF; for planar driving, keep translation and replace rotation with Rz(yaw)
 * using the same yaw convention as yawFrom2DBlock (valid when roll/pitch are small).
 */
static Eigen::Affine3f ndtSensorPosePlanarSeal(const Eigen::Affine3f &T) {
  Eigen::Affine3f out = T;
  const float yaw = yawFrom2DBlock(T.matrix());
  out.linear() = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return out;
}

/** Planar SE2 pose (z=0, yaw only) extracted from a general rigid transform. */
static Eigen::Affine3f planarAffineFromFull(const Eigen::Affine3f &T) {
  const Eigen::Matrix4f M = T.matrix();
  const float x = M.coeff(0, 3);
  const float y = M.coeff(1, 3);
  const float yaw = yawFrom2DBlock(M);
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  Eigen::Affine3f out = Eigen::Affine3f::Identity();
  Eigen::Matrix3f R;
  R << c, -s, 0.0f, s, c, 0.0f, 0.0f, 0.0f, 1.0f;
  out.linear() = R;
  out.translation() << x, y, 0.0f;
  return out;
}

/** Build planar increment in odom (xy + yaw about z) from T_ndt^{-1} (scan-to-scan). */
static Eigen::Affine3f planarIncrementFromNdt(const Eigen::Matrix4f &T_source_to_target) {
  const Eigen::Matrix4f T_inc = T_source_to_target.inverse();
  const float dx = T_inc.coeff(0, 3);
  const float dy = T_inc.coeff(1, 3);
  const float yaw = std::atan2(T_inc.coeff(1, 0), T_inc.coeff(0, 0));
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  Eigen::Affine3f out = Eigen::Affine3f::Identity();
  Eigen::Matrix3f R;
  R << c, -s, 0.0f, s, c, 0.0f, 0.0f, 0.0f, 1.0f;
  out.linear() = R;
  out.translation() << dx, dy, 0.0f;
  return out;
}

class LidarOdometryNode : public rclcpp::Node {
 public:
  LidarOdometryNode() : Node("lidar_odometry_node") {
    topic_cloud_ = declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    topic_odom_ = declare_parameter<std::string>("odom_topic", "/lidar/odom");
    topic_delta_ = declare_parameter<std::string>("delta_topic", "/lidar/relative_motion");
    topic_pose_correction_ =
        declare_parameter<std::string>("pose_correction_topic", "/lidar/pose_correction");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const std::string mode = declare_parameter<std::string>("registration_mode", "scan_to_map");
    scan_to_map_ = (mode == "scan_to_map");

    leaf_size_ = static_cast<float>(declare_parameter<double>("voxel_leaf_size", 0.25));
    range_limit_ = static_cast<float>(declare_parameter<double>("crop_range_m", 40.0));
    ndt_resolution_ = static_cast<float>(declare_parameter<double>("ndt_resolution", 1.0));
    ndt_coarse_resolution_ =
        static_cast<float>(declare_parameter<double>("ndt_coarse_resolution", 0.0));
    ndt_step_size_ = static_cast<float>(declare_parameter<double>("ndt_step_size", 0.1));
    ndt_epsilon_ =
        static_cast<float>(declare_parameter<double>("ndt_transformation_epsilon", 0.01));
    ndt_voxel_min_points_ = declare_parameter<int>("ndt_voxel_min_points", 10);
    ndt_voxel_cov_eig_inflation_ratio_ =
        declare_parameter<double>("ndt_voxel_cov_eig_inflation_ratio", 0.05);
    ndt_max_iter_ = declare_parameter<int>("ndt_max_iterations", 35);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 3.0);
    min_points_ = declare_parameter<int>("min_points_per_cloud", 200);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    log_registration_debug_ = declare_parameter<bool>("log_registration_debug", false);
    log_ndt_relative_ = declare_parameter<bool>("log_ndt_relative", false);
    log_accumulated_pose_ = declare_parameter<bool>("log_accumulated_pose", false);

    use_tf_initial_guess_ = declare_parameter<bool>("use_tf_initial_guess", true);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_initial_guess_timeout_sec", 0.1);

    map_merge_leaf_ = static_cast<float>(declare_parameter<double>("map_merge_voxel_leaf_size", -1.0));
    if (map_merge_leaf_ <= 0.0f) {
      map_merge_leaf_ = leaf_size_;
    }
    map_merge_leaf_initial_ = map_merge_leaf_;
    map_max_points_ = declare_parameter<int>("map_max_points", 400000);
    // scan_to_map only: every N successful alignments, replace voxel map from a short ring of
    // recent aligned scans (drops accumulated bad geometry; global_pose_ unchanged). 0 = disabled.
    map_refresh_period_scans_ = declare_parameter<int>("scan_to_map_map_refresh_period", 0);
    {
      int keep = declare_parameter<int>("scan_to_map_refresh_keep_scans", 3);
      if (keep < 1) {
        keep = 1;
      }
      if (keep > 8) {
        keep = 8;
      }
      scan_to_map_ring_max_ = static_cast<size_t>(keep);
    }
    // scan_to_map: register sensor-frame source to odom map with align(..., T_odom_sensor_pred).
    scan_to_map_register_sensor_frame_ =
        declare_parameter<bool>("scan_to_map_register_sensor_frame", false);

    ndt_fuse_prior_planar_yaw_ = declare_parameter<bool>("ndt_fuse_prior_planar_yaw", false);
    ndt_prior_yaw_blend_ = static_cast<float>(declare_parameter<double>("ndt_prior_yaw_blend", 0.85));
    ndt_prior_yaw_blend_ = std::min(1.0f, std::max(0.0f, ndt_prior_yaw_blend_));
    ndt_corridor_degeneracy_check_ =
        declare_parameter<bool>("ndt_corridor_degeneracy_check", true);
    ndt_corridor_spin_yaw_min_rad_ = static_cast<float>(
        declare_parameter<double>("ndt_corridor_spin_yaw_min_rad", 0.5));
    ndt_corridor_spin_max_corr_xy_m_ = static_cast<float>(
        declare_parameter<double>("ndt_corridor_spin_max_corr_xy_m", 0.1));
    ndt_fallback_if_planar_correction_below_m_ = static_cast<float>(
        declare_parameter<double>("ndt_fallback_if_planar_correction_below_m", 0.0));
    ndt_reject_opposite_ekf_step_ =
        declare_parameter<bool>("ndt_reject_opposite_ekf_step", false);
    ndt_gate_until_prior_translation_m_ = static_cast<float>(
        declare_parameter<double>("ndt_gate_until_prior_translation_m", 0.0));
    ndt_gate_force_after_sec_ = declare_parameter<double>("ndt_gate_force_after_sec", 0.0);
    ndt_opposite_motion_min_ekf_step_m_ = static_cast<float>(
        declare_parameter<double>("ndt_opposite_motion_min_ekf_step_m", 0.05));
    ndt_opposite_motion_min_ndt_step_m_ = static_cast<float>(
        declare_parameter<double>("ndt_opposite_motion_min_ndt_step_m", 0.05));
    map_merge_keyframe_min_translation_m_ = static_cast<float>(
        declare_parameter<double>("map_merge_keyframe_min_translation_m", 0.0));
    map_merge_keyframe_min_yaw_rad_ = static_cast<float>(
        declare_parameter<double>("map_merge_keyframe_min_yaw_rad", 0.0));

    // base_link <- lidar frame: roll, pitch, yaw (rad), x, y, z (m). Default identity if cloud
    // is already in base_link; set for livox_frame offset from base.
    std::vector<double> ext = declare_parameter<std::vector<double>>(
        "sensor_extrinsic_rpy_xyz", std::vector<double>{0, 0, 0, 0, 0, 0});
    if (ext.size() != 6U) {
      RCLCPP_WARN(get_logger(), "sensor_extrinsic_rpy_xyz must have 6 elements; using zeros");
      ext = {0, 0, 0, 0, 0, 0};
    }
    T_base_sensor_ = Eigen::Affine3f::Identity();
    T_base_sensor_.linear() =
        (Eigen::AngleAxisf(static_cast<float>(ext[0]), Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(static_cast<float>(ext[1]), Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(static_cast<float>(ext[2]), Eigen::Vector3f::UnitZ()))
            .toRotationMatrix();
    T_base_sensor_.translation() << static_cast<float>(ext[3]), static_cast<float>(ext[4]),
        static_cast<float>(ext[5]);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_cloud_, rclcpp::SensorDataQoS(),
        std::bind(&LidarOdometryNode::onCloud, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(topic_odom_, 10);
    pub_delta_ = create_publisher<geometry_msgs::msg::TwistStamped>(topic_delta_, 10);
    if (scan_to_map_) {
      pub_pose_correction_ =
          create_publisher<geometry_msgs::msg::PoseStamped>(topic_pose_correction_, 10);
    }

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    global_pose_ = Eigen::Affine3f::Identity();
    global_pose_full_ = Eigen::Affine3f::Identity();

    RCLCPP_INFO(
        get_logger(),
        "lidar_odometry_node: mode=%s cloud=%s odom=%s delta=%s%s",
        scan_to_map_ ? "scan_to_map" : "scan_to_scan", topic_cloud_.c_str(),
        topic_odom_.c_str(), topic_delta_.c_str(),
        scan_to_map_ ? (" pose_correction=" + topic_pose_correction_).c_str() : "");
    if (scan_to_map_ && map_refresh_period_scans_ > 0) {
      RCLCPP_INFO(
          get_logger(),
          "scan_to_map map refresh: every %d scans, rebuild target from last %zu aligned scan(s)",
          map_refresh_period_scans_, scan_to_map_ring_max_);
    }
    if (scan_to_map_ && scan_to_map_register_sensor_frame_) {
      RCLCPP_INFO(
          get_logger(),
          "scan_to_map_register_sensor_frame=true: NDT source=sensor cloud, guess=T_odom_sensor_pred");
    }
    if (scan_to_map_ && ndt_coarse_resolution_ > 0.0f) {
      RCLCPP_INFO(
          get_logger(),
          "ndt_coarse_resolution=%.3f: two-pass NDT (coarse then fine)", ndt_coarse_resolution_);
    }
    if (scan_to_map_ && ndt_fuse_prior_planar_yaw_) {
      RCLCPP_INFO(
          get_logger(),
          "ndt_fuse_prior_planar_yaw=true: yaw blend pred weight=%.2f (1=pred only, 0=NDT only)",
          ndt_prior_yaw_blend_);
    }
    if (scan_to_map_ && ndt_corridor_degeneracy_check_) {
      RCLCPP_INFO(
          get_logger(),
          "ndt_corridor_degeneracy_check: |Δyaw|>%.3f rad & |Δxy|<%.3f m → reject",
          ndt_corridor_spin_yaw_min_rad_, ndt_corridor_spin_max_corr_xy_m_);
    }
    if (scan_to_map_ && ndt_fallback_if_planar_correction_below_m_ > 0.0f) {
      RCLCPP_INFO(
          get_logger(),
          "ndt_fallback_if_planar_correction_below_m=%.3f (reject tiny planar correction)",
          ndt_fallback_if_planar_correction_below_m_);
    }
    if (scan_to_map_ && ndt_reject_opposite_ekf_step_) {
      RCLCPP_INFO(get_logger(), "ndt_reject_opposite_ekf_step=true (NDT step vs EKF step dot<0)");
    }
    if (scan_to_map_ && ndt_gate_until_prior_translation_m_ > 0.0f) {
      RCLCPP_INFO(
          get_logger(),
          "ndt_gate_until_prior_translation_m=%.3f: skip NDT until planar |odom prior xy| >= this "
          "(bad EKF startup / map poisoning). force_after_sec=%.2f (0=strict)",
          ndt_gate_until_prior_translation_m_, ndt_gate_force_after_sec_);
    }
    if (scan_to_map_ &&
        (map_merge_keyframe_min_translation_m_ > 1e-6f || map_merge_keyframe_min_yaw_rad_ > 1e-6f)) {
      RCLCPP_INFO(
          get_logger(),
          "map keyframe merge: min_xy=%.3f m min_yaw=%.3f rad",
          map_merge_keyframe_min_translation_m_, map_merge_keyframe_min_yaw_rad_);
    }
    if (log_ndt_relative_) {
      RCLCPP_INFO(
          get_logger(),
          "log_ndt_relative=true: printing NDT_RELATIVE tx,ty (and dyaw) each accepted NDT step");
    }
    if (log_accumulated_pose_) {
      RCLCPP_INFO(
          get_logger(),
          "log_accumulated_pose=true: printing POSE: x,y to stdout (global_pose_ after each step)");
    }
  }

 private:
  /** stdout + logger: integrated odom->base planar pose (not raw getFinalTransformation). */
  void debugPrintAccumulatedPose(const char *tag) {
    if (!log_accumulated_pose_) {
      return;
    }
    const float x = global_pose_.translation().x();
    const float y = global_pose_.translation().y();
    const float yaw = yawFrom2DBlock(global_pose_.matrix());
    std::cout << "POSE: " << x << ", " << y << "  (accumulated global_pose_ [" << tag << "] yaw="
              << yaw << " rad)" << std::endl;
    RCLCPP_INFO(
        get_logger(), "NDT_ACCUM_POSE [%s]: x=%.5f y=%.5f yaw=%.5f rad", tag, x, y, yaw);
  }

  /** Per-scan NDT output only — must be composed into global_pose_; do not assign pose = this. */
  void debugPrintNdtRelativePlanar(const char *tag, const Eigen::Affine3f &rel_planar) {
    if (!log_ndt_relative_) {
      return;
    }
    const Eigen::Matrix4f &M = rel_planar.matrix();
    const float tx = M.coeff(0, 3);
    const float ty = M.coeff(1, 3);
    const float dyaw = yawFrom2DBlock(M);
    std::cout << "NDT_RELATIVE: " << tx << ", " << ty << "  (this-scan only [" << tag << "] dyaw="
              << dyaw << " rad) — NOT accumulated pose" << std::endl;
    RCLCPP_INFO(
        get_logger(),
        "NDT_RELATIVE [%s]: tx=%.5f ty=%.5f dyaw=%.5f rad (compose into global; do not use as pose)",
        tag, tx, ty, dyaw);
  }

  /** Publishable planar odom→base from accumulated full 3D pose. */
  void projectGlobalPoseToPlanar() {
    global_pose_ = planarAffineFromFull(global_pose_full_);
  }

  /** VoxelGridCovariance settings for PCL 1.12 NDT (no public setCovarianceEpsilon on this version). */
  void configureNdt(ConfigurableNdt &ndt, float resolution, int max_iterations) const {
    ndt.setVoxelMinPointsPerVoxel(ndt_voxel_min_points_);
    ndt.setVoxelCovEigInflationRatio(ndt_voxel_cov_eig_inflation_ratio_);
    ndt.setResolution(resolution);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setMaximumIterations(max_iterations);
  }

  /** Planar odom->base prior: EKF TF at cloud time; else latest TF; else last integrated NDT pose. */
  Eigen::Affine3f initialGuessOdomBase(const builtin_interfaces::msg::Time &stamp) const {
    if (!use_tf_initial_guess_ || !tf_buffer_) {
      return global_pose_full_;
    }
    const rclcpp::Time t(stamp);
    const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);
    try {
      const geometry_msgs::msg::TransformStamped ts =
          tf_buffer_->lookupTransform(odom_frame_, base_frame_, t, timeout);
      const Eigen::Isometry3d Te = transformToEigen(ts.transform);
      return planarAffineFromFull(Te.cast<float>());
    } catch (const tf2::TransformException &) {
      try {
        const geometry_msgs::msg::TransformStamped ts = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0.05));
        if (!logged_tf_latest_fallback_) {
          RCLCPP_INFO(
              get_logger(),
              "TF %s->%s at cloud stamp not ready in time; using latest transform for scan_to_map "
              "initial guess (typical EKF vs LiDAR callback ordering on replay)",
              odom_frame_.c_str(), base_frame_.c_str());
          logged_tf_latest_fallback_ = true;
        }
        const Eigen::Isometry3d Te = transformToEigen(ts.transform);
        return planarAffineFromFull(Te.cast<float>());
      } catch (const tf2::TransformException &) {
        if (!logged_tf_fallback_) {
          RCLCPP_WARN(
              get_logger(),
              "TF %s->%s unavailable (EKF not started); "
              "using last NDT pose for scan_to_map initial guess",
              odom_frame_.c_str(), base_frame_.c_str());
          logged_tf_fallback_ = true;
        }
        return global_pose_full_;
      }
    }
  }

  bool filterCloud(const sensor_msgs::msg::PointCloud2 &msg, PointCloudPtr out) {
    PointCloudPtr raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg, *raw);

    PointCloudPtr finite(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> idx;
    pcl::removeNaNFromPointCloud(*raw, *finite, idx);

    if (static_cast<int>(finite->size()) < min_points_) {
      return false;
    }

    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(finite);
    crop.setMin(Eigen::Vector4f(-range_limit_, -range_limit_, -range_limit_, 1.0f));
    crop.setMax(Eigen::Vector4f(range_limit_, range_limit_, range_limit_, 1.0f));
    PointCloudPtr cropped(new pcl::PointCloud<pcl::PointXYZ>);
    crop.filter(*cropped);

    if (static_cast<int>(cropped->size()) < min_points_) {
      return false;
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cropped);
    voxel.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    voxel.filter(*out);

    return static_cast<int>(out->size()) >= min_points_;
  }

  static void publishZeroDelta(
      const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr &pub,
      const builtin_interfaces::msg::Time &stamp, const std::string &odom_frame) {
    geometry_msgs::msg::TwistStamped tw;
    tw.header.stamp = stamp;
    tw.header.frame_id = odom_frame;
    pub->publish(tw);
  }

  void publishDelta(
      const builtin_interfaces::msg::Time &stamp,
      const Eigen::Affine3f &increment_odom_left) {
    geometry_msgs::msg::TwistStamped tw;
    tw.header.stamp = stamp;
    // Planar step T_new * T_pred^{-1} in odom (world xy); NOT body-frame despite TwistStamped norms.
    tw.header.frame_id = odom_frame_;
    tw.twist.linear.x = static_cast<double>(increment_odom_left.translation().x());
    tw.twist.linear.y = static_cast<double>(increment_odom_left.translation().y());
    tw.twist.angular.z = static_cast<double>(yawFrom2DBlock(increment_odom_left.matrix()));
    pub_delta_->publish(tw);
  }

  void publishPoseCorrection(
      const builtin_interfaces::msg::Time &stamp, const Eigen::Matrix4f &T_ndt_planar_xy_yaw) {
    if (!pub_pose_correction_) {
      return;
    }
    geometry_msgs::msg::PoseStamped pc;
    pc.header.stamp = stamp;
    pc.header.frame_id = odom_frame_;
    const float dx = T_ndt_planar_xy_yaw.coeff(0, 3);
    const float dy = T_ndt_planar_xy_yaw.coeff(1, 3);
    const float yaw = yawFrom2DBlock(T_ndt_planar_xy_yaw);
    const Eigen::Quaternionf q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    pc.pose.position.x = static_cast<double>(dx);
    pc.pose.position.y = static_cast<double>(dy);
    pc.pose.position.z = 0.0;
    pc.pose.orientation.x = static_cast<double>(q.x());
    pc.pose.orientation.y = static_cast<double>(q.y());
    pc.pose.orientation.z = static_cast<double>(q.z());
    pc.pose.orientation.w = static_cast<double>(q.w());
    pub_pose_correction_->publish(pc);
  }

  /** If prior_odom_base is set, fill twist from planar (T_pred^{-1} * T_new) / dt (child_frame convention). */
  void publishOdom(
      const builtin_interfaces::msg::Time &stamp,
      const Eigen::Affine3f *prior_odom_base = nullptr) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = static_cast<double>(global_pose_.translation().x());
    odom.pose.pose.position.y = static_cast<double>(global_pose_.translation().y());
    odom.pose.pose.position.z = 0.0;

    const float yaw = yawFrom2DBlock(global_pose_.matrix());
    const Eigen::Quaternionf q(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    odom.pose.pose.orientation.x = static_cast<double>(q.x());
    odom.pose.pose.orientation.y = static_cast<double>(q.y());
    odom.pose.pose.orientation.z = static_cast<double>(q.z());
    odom.pose.pose.orientation.w = static_cast<double>(q.w());

    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[14] = 0.1;
    odom.pose.covariance[35] = 0.05;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    if (prior_odom_base != nullptr && have_twist_ref_stamp_) {
      const rclcpp::Time t_now(stamp);
      const rclcpp::Time t_prev(last_twist_ref_stamp_);
      double dt = (t_now - t_prev).seconds();
      if (dt > 1e-3 && dt < 0.6) {
        const Eigen::Affine3f d_body =
            planarAffineFromFull(prior_odom_base->inverse() * global_pose_);
        const float dth = yawFrom2DBlock(d_body.matrix());
        odom.twist.twist.linear.x = static_cast<double>(d_body.translation().x() / static_cast<float>(dt));
        odom.twist.twist.linear.y = static_cast<double>(d_body.translation().y() / static_cast<float>(dt));
        odom.twist.twist.angular.z = static_cast<double>(dth / static_cast<float>(dt));
      }
    }

    last_twist_ref_stamp_ = stamp;
    have_twist_ref_stamp_ = true;

    pub_odom_->publish(odom);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = odom.pose.pose.position.x;
      tf.transform.translation.y = odom.pose.pose.position.y;
      tf.transform.translation.z = odom.pose.pose.position.z;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  /** Sensor frame -> odom using planar base pose in odom. */
  void cloudSensorToOdom(const PointCloudPtr &cloud_sensor, const Eigen::Affine3f &T_odom_base,
                         PointCloudPtr cloud_odom) const {
    const Eigen::Affine3f T_odom_sensor = T_odom_base * T_base_sensor_;
    pcl::transformPointCloud(*cloud_sensor, *cloud_odom, T_odom_sensor.matrix());
  }

  void mergeIntoMap(const pcl::PointCloud<pcl::PointXYZ> &aligned_in_odom) {
    *map_cloud_ += aligned_in_odom;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(map_cloud_);
    voxel.setLeafSize(map_merge_leaf_, map_merge_leaf_, map_merge_leaf_);
    PointCloudPtr merged(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*merged);
    map_cloud_ = merged;

    int guard = 0;
    while (static_cast<int>(map_cloud_->size()) > map_max_points_ && guard < 8) {
      map_merge_leaf_ *= 1.35f;
      voxel.setInputCloud(map_cloud_);
      voxel.setLeafSize(map_merge_leaf_, map_merge_leaf_, map_merge_leaf_);
      merged.reset(new pcl::PointCloud<pcl::PointXYZ>);
      voxel.filter(*merged);
      map_cloud_ = merged;
      ++guard;
    }
  }

  void onCloudScanToMap(const builtin_interfaces::msg::Time &stamp, const PointCloudPtr &cloud_f) {
    const Eigen::Affine3f T_odom_base_pred = initialGuessOdomBase(stamp);

    PointCloudPtr cloud_odom;
    if (!have_map_ || !scan_to_map_register_sensor_frame_) {
      cloud_odom.reset(new pcl::PointCloud<pcl::PointXYZ>);
      cloudSensorToOdom(cloud_f, T_odom_base_pred, cloud_odom);
    }

    if (!have_map_) {
      // Map is built in odom using T_odom_base_pred; keep integrated pose in sync so the first
      // /lidar/odom is not stuck at identity while EKF already moved (avoids NIS rejects / spin).
      global_pose_full_ = planarAffineFromFull(T_odom_base_pred);
      projectGlobalPoseToPlanar();
      last_map_merge_pose_ = global_pose_;
      map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud_odom));
      have_map_ = true;
      ndt_gate_t0_ = rclcpp::Time(stamp);
      ndt_gate_t0_valid_ = true;
      ndt_prior_motion_ok_ = false;
      publishZeroDelta(pub_delta_, stamp, odom_frame_);
      publishOdom(stamp, nullptr);
      debugPrintAccumulatedPose("scan_to_map_first_cloud_seed");
      return;
    }

    if (scan_to_map_ && ndt_gate_until_prior_translation_m_ > 0.0f && !ndt_prior_motion_ok_) {
      const float prior_xy =
          planarAffineFromFull(T_odom_base_pred).translation().head<2>().norm();
      bool ok = prior_xy >= ndt_gate_until_prior_translation_m_;
      if (!ok && ndt_gate_force_after_sec_ > 0.0 && ndt_gate_t0_valid_) {
        const double elapsed =
            std::max(0.0, (rclcpp::Time(stamp) - ndt_gate_t0_).seconds());
        if (elapsed >= ndt_gate_force_after_sec_) {
          ok = true;
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "ndt_gate: allowing NDT after %.1f s (prior |xy|=%.3f < gate %.3f)",
              elapsed, prior_xy, ndt_gate_until_prior_translation_m_);
        }
      }
      if (!ok) {
        if (log_registration_debug_) {
          RCLCPP_INFO(
              get_logger(),
              "scan_to_map: skip NDT (prior |xy|=%.4f < ndt_gate_until_prior_translation_m=%.3f)",
              prior_xy, ndt_gate_until_prior_translation_m_);
        }
        return;
      }
      ndt_prior_motion_ok_ = true;
    }

    const Eigen::Affine3f T_odom_sensor_pred = T_odom_base_pred * T_base_sensor_;
    Eigen::Matrix4f align_guess = Eigen::Matrix4f::Identity();
    if (scan_to_map_register_sensor_frame_) {
      align_guess = T_odom_sensor_pred.matrix();
    }

    auto set_source_target = [&](ConfigurableNdt &n) {
      if (scan_to_map_register_sensor_frame_) {
        n.setInputSource(cloud_f);
      } else {
        n.setInputSource(cloud_odom);
      }
      n.setInputTarget(map_cloud_);
    };

    if (ndt_coarse_resolution_ > 0.0f) {
      ConfigurableNdt ndt_c;
      const int it_c = std::max(10, ndt_max_iter_ / 3);
      configureNdt(ndt_c, ndt_coarse_resolution_, it_c);
      set_source_target(ndt_c);
      pcl::PointCloud<pcl::PointXYZ> aligned_c;
      ndt_c.align(aligned_c, align_guess);
      if (!ndt_c.hasConverged()) {
        if (log_registration_debug_) {
          RCLCPP_INFO(get_logger(), "scan_to_map NDT coarse: converged=false (skipped)");
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000, "scan_to_map NDT coarse pass did not converge");
        return;
      }
      align_guess = ndt_c.getFinalTransformation();
    }

    ConfigurableNdt ndt;
    configureNdt(ndt, ndt_resolution_, ndt_max_iter_);
    set_source_target(ndt);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    ndt.align(aligned, align_guess);

    if (!ndt.hasConverged()) {
      if (log_registration_debug_) {
        RCLCPP_INFO(get_logger(), "scan_to_map NDT: converged=false (skipped)");
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "scan_to_map NDT did not converge");
      return;
    }

    const double fitness = ndt.getFitnessScore();
    if (fitness > max_fitness_score_) {
      if (log_registration_debug_) {
        RCLCPP_INFO(
            get_logger(),
            "scan_to_map NDT: converged=true fitness=%.6f > max %.3f (skipped)",
            fitness, max_fitness_score_);
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "scan_to_map NDT fitness %.3f > max %.3f — skip", fitness, max_fitness_score_);
      return;
    }

    if (log_registration_debug_) {
      RCLCPP_INFO(
          get_logger(),
          "scan_to_map NDT: converged=true fitness=%.6f (<= max %.3f, accepted)",
          fitness, max_fitness_score_);
    }

    const Eigen::Matrix4f T_ndt = ndt.getFinalTransformation();
    Eigen::Affine3f T_odom_sensor_new = scan_to_map_register_sensor_frame_
                                              ? Eigen::Affine3f(T_ndt)
                                              : (Eigen::Affine3f(T_ndt) * T_odom_sensor_pred);
    T_odom_sensor_new = ndtSensorPosePlanarSeal(T_odom_sensor_new);

    if (ndt_corridor_degeneracy_check_) {
      const float yaw_n = yawFrom2DBlock(T_odom_sensor_new.matrix());
      const float yaw_p = yawFrom2DBlock(T_odom_sensor_pred.matrix());
      const float dyaw = std::abs(wrapAnglePi(yaw_n - yaw_p));
      const Eigen::Affine3f T_corr_raw =
          planarAffineFromFull(T_odom_sensor_new * T_odom_sensor_pred.inverse());
      const float dxy = T_corr_raw.translation().head<2>().norm();
      if (dyaw > ndt_corridor_spin_yaw_min_rad_ && dxy < ndt_corridor_spin_max_corr_xy_m_) {
        if (log_registration_debug_) {
          RCLCPP_INFO(
              get_logger(),
              "scan_to_map: corridor degeneracy (|dyaw|=%.3f, |dxy|=%.4f) — fallback to prediction",
              dyaw, dxy);
        }
        T_odom_sensor_new = T_odom_sensor_pred;
      }
    }

    if (ndt_fuse_prior_planar_yaw_) {
      const float yaw_ndt = yawFrom2DBlock(T_odom_sensor_new.matrix());
      const float yaw_pred = yawFrom2DBlock(T_odom_sensor_pred.matrix());
      const float a = ndt_prior_yaw_blend_;
      const float yaw_f = wrapAnglePi(a * yaw_pred + (1.0f - a) * yaw_ndt);
      Eigen::Affine3f T_locked = Eigen::Affine3f::Identity();
      T_locked.translation() = T_odom_sensor_new.translation();
      T_locked.linear() =
          Eigen::AngleAxisf(yaw_f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
      T_odom_sensor_new = T_locked;
    }

    Eigen::Affine3f T_corr_planar =
        planarAffineFromFull(T_odom_sensor_new * T_odom_sensor_pred.inverse());

    if (ndt_fallback_if_planar_correction_below_m_ > 0.0f) {
      const float dxy = T_corr_planar.translation().head<2>().norm();
      if (dxy < ndt_fallback_if_planar_correction_below_m_) {
        if (log_registration_debug_) {
          RCLCPP_INFO(
              get_logger(),
              "scan_to_map: planar correction |dxy|=%.4f < %.4f — fallback to prediction",
              dxy, ndt_fallback_if_planar_correction_below_m_);
        }
        T_odom_sensor_new = T_odom_sensor_pred;
        T_corr_planar = planarAffineFromFull(Eigen::Affine3f::Identity());
      }
    }

    T_odom_sensor_new = ndtSensorPosePlanarSeal(T_odom_sensor_new);
    Eigen::Affine3f global_new_full = T_odom_sensor_new * T_base_sensor_.inverse();

    if (ndt_reject_opposite_ekf_step_) {
      const Eigen::Affine3f gn_planar = planarAffineFromFull(global_new_full);
      const Eigen::Vector2f ndt_step =
          (gn_planar.translation() - T_odom_base_pred.translation()).head<2>();
      const Eigen::Vector2f ekf_step =
          (T_odom_base_pred.translation() - global_pose_.translation()).head<2>();
      const float n_ekf = ekf_step.norm();
      const float n_ndt = ndt_step.norm();
      if (n_ekf >= ndt_opposite_motion_min_ekf_step_m_ &&
          n_ndt >= ndt_opposite_motion_min_ndt_step_m_ && ekf_step.dot(ndt_step) < 0.0f) {
        if (log_registration_debug_) {
          RCLCPP_INFO(
              get_logger(),
              "scan_to_map: NDT step opposite to EKF step — fallback to prediction");
        }
        T_odom_sensor_new = ndtSensorPosePlanarSeal(T_odom_sensor_pred);
        global_new_full = T_odom_sensor_new * T_base_sensor_.inverse();
        T_corr_planar = planarAffineFromFull(Eigen::Affine3f::Identity());
      }
    }

    if (log_accumulated_pose_ || log_registration_debug_) {
      std::cout << "PRED: " << T_odom_base_pred.translation().x() << " NEW: "
                << global_new_full.translation().x() << std::endl;
    }
    global_pose_full_ = planarAffineFromFull(global_new_full);
    projectGlobalPoseToPlanar();
    const Eigen::Affine3f delta =
        planarAffineFromFull(global_pose_ * T_odom_base_pred.inverse());
    if (log_ndt_relative_) {
      std::cout << "STEP_ODOM: " << delta.translation().x() << " " << delta.translation().y()
                << " dyaw=" << yawFrom2DBlock(delta.matrix()) << std::endl;
    }

    debugPrintNdtRelativePlanar("scan_to_map_T_corr_vs_pred", T_corr_planar);

    publishPoseCorrection(stamp, T_corr_planar.matrix());
    debugPrintAccumulatedPose("scan_to_map_after_Tcorr_chain");

    publishDelta(stamp, delta);
    publishOdom(stamp, &T_odom_base_pred);

    if (scan_to_map_register_sensor_frame_) {
      pcl::transformPointCloud(*cloud_f, aligned, T_odom_sensor_new.matrix());
    }

    bool do_map_merge = true;
    if (map_merge_keyframe_min_translation_m_ > 1e-6f ||
        map_merge_keyframe_min_yaw_rad_ > 1e-6f) {
      const Eigen::Vector2f d =
          (global_pose_.translation() - last_map_merge_pose_.translation()).head<2>();
      const float yaw_curr = yawFrom2DBlock(global_pose_.matrix());
      const float yaw_ref = yawFrom2DBlock(last_map_merge_pose_.matrix());
      const float dyaw = std::abs(wrapAnglePi(yaw_curr - yaw_ref));
      do_map_merge = (d.norm() >= map_merge_keyframe_min_translation_m_) ||
                     (dyaw >= map_merge_keyframe_min_yaw_rad_);
    }

    if (do_map_merge) {
      mergeIntoMap(aligned);
      last_map_merge_pose_ = global_pose_;
      pushScanToMapAlignRing(aligned);
    } else if (log_registration_debug_) {
      RCLCPP_INFO(get_logger(), "scan_to_map: keyframe gate — pose update, skip map merge");
    }
    maybePeriodicScanToMapRefresh();
  }

  void pushScanToMapAlignRing(const pcl::PointCloud<pcl::PointXYZ> &aligned) {
    if (!scan_to_map_ || map_refresh_period_scans_ <= 0) {
      return;
    }
    PointCloudPtr c(new pcl::PointCloud<pcl::PointXYZ>(aligned));
    scan_to_map_align_ring_.push_back(c);
    while (scan_to_map_align_ring_.size() > scan_to_map_ring_max_) {
      scan_to_map_align_ring_.pop_front();
    }
  }

  /** If enabled, periodically discard accumulated map and rebuild target from recent aligned scans. */
  void maybePeriodicScanToMapRefresh() {
    if (!scan_to_map_ || map_refresh_period_scans_ <= 0) {
      return;
    }
    ++scan_to_map_accept_count_;
    if (scan_to_map_accept_count_ % static_cast<size_t>(map_refresh_period_scans_) != 0U) {
      return;
    }
    map_merge_leaf_ = map_merge_leaf_initial_;
    pcl::PointCloud<pcl::PointXYZ> merged;
    if (!scan_to_map_align_ring_.empty()) {
      for (const auto &p : scan_to_map_align_ring_) {
        merged += *p;
      }
    } else {
      return;
    }
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(merged));
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(map_cloud_);
    const float leaf = std::max(leaf_size_, map_merge_leaf_);
    vox.setLeafSize(leaf, leaf, leaf);
    PointCloudPtr pruned(new pcl::PointCloud<pcl::PointXYZ>);
    vox.filter(*pruned);
    map_cloud_ = pruned;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "scan_to_map: periodic map refresh (every %d scans, %zu-scan ring), map points=%zu",
        map_refresh_period_scans_,
        scan_to_map_align_ring_.size(),
        map_cloud_->size());
  }

  void onCloudScanToScan(const builtin_interfaces::msg::Time &stamp, const PointCloudPtr &cloud_f) {
    if (!have_target_) {
      target_cloud_ = cloud_f;
      have_target_ = true;
      global_pose_ = Eigen::Affine3f::Identity();
      global_pose_full_ = Eigen::Affine3f::Identity();
      projectGlobalPoseToPlanar();
      publishZeroDelta(pub_delta_, stamp, odom_frame_);
      publishOdom(stamp, nullptr);
      debugPrintAccumulatedPose("scan_to_scan_first_cloud_identity");
      return;
    }

    ConfigurableNdt ndt;
    configureNdt(ndt, ndt_resolution_, ndt_max_iter_);
    ndt.setInputSource(cloud_f);
    ndt.setInputTarget(target_cloud_);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    ndt.align(aligned);

    if (!ndt.hasConverged()) {
      if (log_registration_debug_) {
        RCLCPP_INFO(get_logger(), "scan_to_scan NDT: converged=false (skipped)");
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "NDT did not converge");
      target_cloud_ = cloud_f;
      return;
    }

    const double fitness = ndt.getFitnessScore();
    if (fitness > max_fitness_score_) {
      if (log_registration_debug_) {
        RCLCPP_INFO(
            get_logger(),
            "scan_to_scan NDT: converged=true fitness=%.6f > max %.3f (skipped)",
            fitness, max_fitness_score_);
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "NDT fitness %.3f > max %.3f — skip", fitness, max_fitness_score_);
      target_cloud_ = cloud_f;
      return;
    }

    if (log_registration_debug_) {
      RCLCPP_INFO(
          get_logger(),
          "scan_to_scan NDT: converged=true fitness=%.6f (<= max %.3f, accepted)",
          fitness, max_fitness_score_);
    }

    const Eigen::Matrix4f T_ndt = ndt.getFinalTransformation();
    const Eigen::Affine3f increment_planar = planarIncrementFromNdt(T_ndt);
    debugPrintNdtRelativePlanar("scan_to_scan_body_increment", increment_planar);
    const Eigen::Affine3f prior_odom_base = global_pose_;
    global_pose_ = global_pose_ * increment_planar;
    global_pose_full_ = global_pose_;
    projectGlobalPoseToPlanar();
    debugPrintAccumulatedPose("scan_to_scan_after_compose");

    const Eigen::Affine3f delta_odom =
        planarAffineFromFull(global_pose_ * prior_odom_base.inverse());
    publishDelta(stamp, delta_odom);
    publishOdom(stamp, &prior_odom_base);
    target_cloud_ = cloud_f;
  }

  void ensureTfListenerLazy() {
    if (!use_tf_initial_guess_ || tf_listener_) {
      return;
    }
    if (!tf_buffer_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    }
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
        *tf_buffer_, std::static_pointer_cast<rclcpp::Node>(shared_from_this()), true);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    ensureTfListenerLazy();
    std::lock_guard<std::mutex> lock(process_mutex_);

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "PointCloud2 header stamp is zero; skip (need valid time for TF/EKF sync)");
      return;
    }

    PointCloudPtr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    if (!filterCloud(*msg, cloud_f)) {
      return;
    }

    const builtin_interfaces::msg::Time &stamp = msg->header.stamp;

    if (scan_to_map_) {
      onCloudScanToMap(stamp, cloud_f);
    } else {
      onCloudScanToScan(stamp, cloud_f);
    }
  }

  std::mutex process_mutex_;
  bool have_target_{false};
  bool have_map_{false};
  PointCloudPtr target_cloud_;
  PointCloudPtr map_cloud_;
  Eigen::Affine3f global_pose_;
  /** Accumulated odom→base (6-DOF); global_pose_ is planar projection for publish/2D logic. */
  Eigen::Affine3f global_pose_full_;

  bool have_twist_ref_stamp_{false};
  builtin_interfaces::msg::Time last_twist_ref_stamp_;

  bool scan_to_map_{false};
  bool scan_to_map_register_sensor_frame_{false};
  std::string topic_cloud_;
  std::string topic_odom_;
  std::string topic_delta_;
  std::string topic_pose_correction_;
  std::string odom_frame_;
  std::string base_frame_;

  float leaf_size_{0.25f};
  float range_limit_{40.0f};
  float ndt_resolution_{1.0f};
  float ndt_coarse_resolution_{0.0f};
  float ndt_step_size_{0.1f};
  float ndt_epsilon_{0.01f};
  int ndt_voxel_min_points_{10};
  double ndt_voxel_cov_eig_inflation_ratio_{0.05};
  int ndt_max_iter_{35};
  double max_fitness_score_{3.0};
  bool log_registration_debug_{false};
  bool log_ndt_relative_{false};
  bool log_accumulated_pose_{false};
  int min_points_{200};
  bool publish_tf_{false};

  bool ndt_fuse_prior_planar_yaw_{false};
  float ndt_prior_yaw_blend_{0.85f};
  bool ndt_corridor_degeneracy_check_{true};
  float ndt_corridor_spin_yaw_min_rad_{0.5f};
  float ndt_corridor_spin_max_corr_xy_m_{0.1f};
  float ndt_fallback_if_planar_correction_below_m_{0.0f};
  bool ndt_reject_opposite_ekf_step_{false};
  float ndt_gate_until_prior_translation_m_{0.0f};
  double ndt_gate_force_after_sec_{0.0};
  bool ndt_prior_motion_ok_{false};
  bool ndt_gate_t0_valid_{false};
  rclcpp::Time ndt_gate_t0_;
  float ndt_opposite_motion_min_ekf_step_m_{0.05f};
  float ndt_opposite_motion_min_ndt_step_m_{0.05f};
  float map_merge_keyframe_min_translation_m_{0.0f};
  float map_merge_keyframe_min_yaw_rad_{0.0f};

  bool use_tf_initial_guess_{true};
  double tf_lookup_timeout_sec_{0.1};
  mutable bool logged_tf_latest_fallback_{false};
  mutable bool logged_tf_fallback_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  float map_merge_leaf_{0.25f};
  float map_merge_leaf_initial_{0.25f};
  int map_max_points_{400000};
  int map_refresh_period_scans_{0};
  size_t scan_to_map_ring_max_{3};
  size_t scan_to_map_accept_count_{0};
  std::deque<PointCloudPtr> scan_to_map_align_ring_;
  Eigen::Affine3f T_base_sensor_{Eigen::Affine3f::Identity()};
  Eigen::Affine3f last_map_merge_pose_{Eigen::Affine3f::Identity()};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_delta_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_correction_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace lidar_odometry

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_odometry::LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
