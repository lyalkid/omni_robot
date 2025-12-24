/**
 * Omni Kinematics Node for 4-wheel omnidirectional robot
 * 
 * Converts cmd_vel commands to individual wheel velocities
 * Publishes odometry from wheel encoders and IMU
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Robot parameters - CHANGE THESE FOR YOUR ROBOT
#define WHEEL_RADIUS 0.03      // meters
#define ROBOT_RADIUS 0.088     // distance from center to wheel (meters)
#define NUM_WHEELS   4
#define HEADING_OFFSET -45.0   // degrees (for 4-wheel config)

class OmniKinematics : public rclcpp::Node
{
public:
  OmniKinematics()
  : Node("omni_kinematics")
  {
    // Initialize transform matrix for inverse kinematics
    init_transform_matrix();

    // Create publishers for each wheel controller
    for (int i = 1; i <= NUM_WHEELS; i++) {
      std::string topic = "wheel" + std::to_string(i) + "_controller/commands";
      auto pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
      wheel_pubs_.push_back(pub);
      
      // Map joint names to indices
      wheel_joint_map_["omni_wheel_joint_" + std::to_string(i)] = i - 1;
    }

    // Odometry publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&OmniKinematics::cmd_vel_callback, this, _1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&OmniKinematics::joint_state_callback, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&OmniKinematics::imu_callback, this, _1));

    // TF broadcaster for odom -> base_footprint
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    last_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Omni Kinematics node started");
    RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m, Robot radius: %.3f m", 
                WHEEL_RADIUS, ROBOT_RADIUS);
  }

private:
  // Transform matrices
  Eigen::MatrixXd T_;      // cmd_vel -> wheel velocities
  Eigen::MatrixXd T_inv_;  // wheel velocities -> robot velocity

  // Publishers & Subscribers
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> wheel_pubs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Joint name to index mapping
  std::unordered_map<std::string, int> wheel_joint_map_;

  // Odometry state
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double yaw_ = 0.0;
  double vx_ = 0.0;
  double vy_ = 0.0;
  double omega_ = 0.0;

  rclcpp::Time last_time_;

  /**
   * Initialize the kinematic transform matrix
   * For N wheels at angle θᵢ:
   *   ωᵢ = (-sin(θᵢ)·vₓ + cos(θᵢ)·vᵧ + R·ω) / r
   */
  void init_transform_matrix()
  {
    T_ = Eigen::MatrixXd::Zero(NUM_WHEELS, 3);
    double del_angle = 360.0 / NUM_WHEELS;

    for (int i = 0; i < NUM_WHEELS; i++) {
      double angle_rad = (del_angle * i + HEADING_OFFSET) * M_PI / 180.0;
      T_(i, 0) = -sin(angle_rad) / WHEEL_RADIUS;  // vx coefficient
      T_(i, 1) = cos(angle_rad) / WHEEL_RADIUS;   // vy coefficient
      T_(i, 2) = ROBOT_RADIUS / WHEEL_RADIUS;     // omega coefficient
    }

    // Compute pseudo-inverse for forward kinematics
    T_inv_ = pseudo_inverse(T_);
    // Only keep vx and vy rows (ignore omega, get it from IMU)
    T_inv_ = Eigen::MatrixXd(T_inv_.block(0, 0, 2, T_inv_.cols()));
  }

  /**
   * Compute Moore-Penrose pseudo-inverse
   */
  Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd& A, double tolerance = 1e-8)
  {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& singularValues = svd.singularValues();
    Eigen::MatrixXd S_inv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (int i = 0; i < singularValues.size(); ++i) {
      if (singularValues(i) > tolerance) {
        S_inv(i, i) = 1.0 / singularValues(i);
      }
    }

    return svd.matrixV() * S_inv * svd.matrixU().transpose();
  }

  /**
   * Handle incoming velocity commands
   */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    Eigen::Vector3d cmd(msg->linear.x, msg->linear.y, msg->angular.z);
    Eigen::VectorXd wheel_velocities = T_ * cmd;

    // Publish to each wheel controller
    for (int i = 0; i < NUM_WHEELS; i++) {
      auto wheel_msg = std_msgs::msg::Float64MultiArray();
      wheel_msg.data = {wheel_velocities(i)};
      wheel_pubs_[i]->publish(wheel_msg);
    }

    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
  }

  /**
   * Handle joint states for odometry calculation
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    Eigen::VectorXd wheel_vels(NUM_WHEELS);
    wheel_vels.setZero();

    // Extract wheel velocities from joint states
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = wheel_joint_map_.find(msg->name[i]);
      if (it != wheel_joint_map_.end() && i < msg->velocity.size()) {
        wheel_vels(it->second) = msg->velocity[i];
      }
    }

    // Calculate time delta
    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0 || dt > 1.0) return;

    // Calculate robot velocity in local frame
    Eigen::Vector2d local_vel = T_inv_ * wheel_vels;

    // Transform to global frame using rotation matrix
    Eigen::Matrix2d R;
    R << cos(yaw_), -sin(yaw_),
         sin(yaw_), cos(yaw_);
    Eigen::Vector2d global_delta = R * local_vel * dt;

    // Update position
    pos_x_ += global_delta(0);
    pos_y_ += global_delta(1);

    // Publish odometry
    publish_odometry(current_time);
  }

  /**
   * Handle IMU data for yaw angle
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    omega_ = msg->angular_velocity.z;

    // Extract yaw from quaternion
    tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w);

    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
  }

  /**
   * Publish odometry message and TF transform
   */
  void publish_odometry(rclcpp::Time current_time)
  {
    // Create quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);

    // Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = pos_x_;
    odom_msg.pose.pose.position.y = pos_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = omega_;

    odom_pub_->publish(odom_msg);

    // TF transform: odom -> base_footprint
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = pos_x_;
    tf_msg.transform.translation.y = pos_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OmniKinematics>());
  rclcpp::shutdown();
  return 0;
}
