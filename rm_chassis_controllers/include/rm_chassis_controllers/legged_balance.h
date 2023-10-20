//
// Created by yuchen on 23-10-19.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/tf_rt_broadcaster.h>
#include <rm_common/ori_tool.h>
#include <rm_common/lqr.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <angles/angles.h>

#include <Eigen/Core>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
struct LeggedBalanceParameters
{
  //  void display() const {
  //    std::cerr << "Balance parameters: " << std::endl;
  //    std::cerr << "mass:   " << mass_ << std::endl;
  //    std::cerr << "wheelRadius:   " << wheelRadius_ << std::endl;
  //    std::cerr << "heightBallCenterToBase:   " << heightBallCenterToBase_ << std::endl;
  //  }
  // Distance between the two wheels
  double d_;  // [m]
  // Distance of shoulder axle to center of mass
  double l_c;  // [m]
  // Wheel radius
  double r_;  // [m]
  // Mass of the pendulum body (except wheels and legs) [kg]
  double massBody_;
  // Mass of the leg [kg]
  double massLeg_;
  // Mass of each wheel [kg]
  double massWheel_;
  // Single wheel moment of inertia (MOI) w.r.t. the wheel axis [kg]
  double jWheel_;  // [kg*m^2]
  // Earth gravitation [m/s^2]
  double g_;
  // MOI of each wheel w.r.t. the vertical axis
  double kWheel_;  // [kg*m^2]
  // MOI of the pendulum body w.r.t.{B} [kg*m^2]
  double i1;
  double i2;
  double i3;

  double powerCoeffEffort_ = 1.339;
  double powerCoeffVel_ = 0.016;
  double powerOffset_ = 9.8;
};

using matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
class VMC
{
public:
  VMC(double l_a, double l_u, double l_d);
  ~VMC() = default;

  matrix_t pendulumEff2JointEff(double F_bl, double T_bl, double front_joint_angle, double back_joint_angle) const;
  matrix_t jointPos2Pendulum(double front_joint_angle, double back_joint_angle, double front_joint_vel,
                             double back_joint_vel) const;

private:
  double l_a_, l_u_, l_d_;
};

class LeggedBalanceLQRController
  : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                       hardware_interface::EffortJointInterface>
{
public:
  LeggedBalanceLQRController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  geometry_msgs::Twist odometry() override;
  void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
  bool loadDynamicsParams(ros::NodeHandle& controller_nh);
  bool loadVMCParams(ros::NodeHandle& controller_nh);
  void legCmdCallback(const std_msgs::UInt32ConstPtr& cmd);

  // Robot mode
  void normal(const ros::Time& time, const ros::Duration& period);

  void block(const ros::Time& time, const ros::Duration& period);
  bool block_state_{ false }, block_state_changed_{ false }, maybe_block_{ false };  // Block Protect
  ros::Time maybe_block_time_, last_block_time_;

  void sitDown(const ros::Time& time, const ros::Duration& period);
  double block_angle_;
  effort_controllers::JointVelocityController left_wheel_controller_, right_wheel_controller_;  // Sit down

  // Interface
  rm_control::RobotStateHandle robot_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;

  // Dynamics
  matrix_t generateA(double l_l, double l_r) const;
  matrix_t generateB(double l_l, double l_r) const;
  static const int STATE_DIM = 10, CONTROL_DIM = 6, INPUT_DIM = 4;
  LeggedBalanceParameters param_;
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> q_{};
  Eigen::Matrix<double, INPUT_DIM, INPUT_DIM> r_{};
  std::vector<std::pair<double, Eigen::Matrix<double, INPUT_DIM, STATE_DIM>>> k_pair_;

  // Leg command
  uint32_t leg_length_index_cmd_;
  ros::Subscriber leg_cmd_sub_;

  // Leg control
  double roll_;
  double pendulum_length_[2], leg_length_cmd_ = 0.13;
  control_toolbox::Pid pid_left_leg_, pid_right_leg_, pid_theta_diff_, pid_roll_;
  ros::Publisher leg_length_publisher_, leg_pendulum_support_force_;

  // VMC
  std::shared_ptr<VMC> vmc_;

  // State Publish
  double observation_state_[STATE_DIM], input_state_[INPUT_DIM];
  typedef std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>> RtPublisherPtr;
  RtPublisherPtr observation_publisher_, input_publisher_;

  // State reset
  double position_offset_ = 0.1;
  double position_clear_threshold_ = 1.0;
};
}  // namespace rm_chassis_controllers
