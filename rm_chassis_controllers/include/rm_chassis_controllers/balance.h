//
// Created by yezi on 2022/11/15.
//

#pragma once

#include <rm_common/lqr.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/rm_imu_sensor_interface.h>
#include <rm_chassis_controllers/QRConfig.h>
#include <rm_msgs/BalanceState.h>

#include "rm_chassis_controllers/chassis_base.h"

namespace rm_chassis_controllers
{
using Eigen::Matrix;
class BalanceController : public ChassisBase<rm_control::RobotStateInterface, rm_control::RmImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
  enum BalanceMode
  {
    NORMAL,
    FALLEN,
    BLOCK
  };

public:
  BalanceController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

private:
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void normal(const ros::Time& time, const ros::Duration& period);
  void block(const ros::Time& time, const ros::Duration& period);

  void publishState(const ros::Time& time);
  void reconfigCB(rm_chassis_controllers::QRConfig& config, uint32_t /*level*/);
  geometry_msgs::Twist odometry() override;
  static const int STATE_DIM = 10;
  static const int CONTROL_DIM = 4;
  void getQ(const XmlRpc::XmlRpcValue& q, Eigen::Matrix<double, STATE_DIM, STATE_DIM>& eigen_q);
  void getR(const XmlRpc::XmlRpcValue& r, Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& eigen_r);
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_mid_{}, k_fallen_{}, k_low_{}, k_high_{};
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a_{}, q_mid_{}, q_fallen_{}, q_low_{}, q_high_{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b_{};
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> r_mid_{}, r_low_{}, r_high_{};
  Eigen::Matrix<double, STATE_DIM, 1> x_;
  double wheel_radius_, wheel_base_;
  double position_des_ = 0;
  double position_offset_ = 0.;
  double position_clear_threshold_ = 0.;
  double yaw_des_ = 0;

  // dynamic reconfigure
  double q_dynamic_[STATE_DIM], r_dynamic_[CONTROL_DIM], q_config_[STATE_DIM], r_config_[CONTROL_DIM];
  bool dynamic_reconfig_initialized_ = false;
  dynamic_reconfigure::Server<rm_chassis_controllers::QRConfig>* reconf_server_;

  int balance_state_;
  int balance_mode_, last_balance_mode_;
  ros::Time block_time_, last_block_time_;
  double fallen_angle_, block_angle_, block_duration_, block_velocity_, block_effort_, anti_block_effort_,
      block_overtime_;
  bool balance_state_changed_ = false, maybe_block_ = false, imu_online_;

  rm_control::RmImuSensorHandle imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_,
      left_momentum_block_joint_handle_, right_momentum_block_joint_handle_;

  typedef std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::BalanceState>> RtpublisherPtr;
  RtpublisherPtr state_pub_;
  geometry_msgs::Vector3 angular_vel_base_;
  double roll_, pitch_, yaw_;
};

}  // namespace rm_chassis_controllers
