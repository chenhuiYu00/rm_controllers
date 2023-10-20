//
// Created by yuchen on 23-10-19.
//

#include <pluginlib/class_list_macros.hpp>

#include "rm_chassis_controllers/legged_balance.h"

namespace rm_chassis_controllers
{
bool LeggedBalanceLQRController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  // Hardware interface
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
  joint_handles_.push_back(effort_joint_interface->getHandle("left_wheel_joint"));
  joint_handles_.push_back(effort_joint_interface->getHandle("right_wheel_joint"));
  joint_handles_.push_back(effort_joint_interface->getHandle("left_front_first_leg_joint"));
  joint_handles_.push_back(effort_joint_interface->getHandle("right_front_first_leg_joint"));
  joint_handles_.push_back(effort_joint_interface->getHandle("left_back_first_leg_joint"));
  joint_handles_.push_back(effort_joint_interface->getHandle("right_back_first_leg_joint"));
  imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // For sit-down
  ros::NodeHandle left = ros::NodeHandle(controller_nh, "left_wheel");
  ros::NodeHandle right = ros::NodeHandle(controller_nh, "right_wheel");
  if (!left_wheel_controller_.init(effort_joint_interface, left) ||
      !right_wheel_controller_.init(effort_joint_interface, right))
    return false;

  if (controller_nh.hasParam("pid_left_leg"))
    if (!pid_left_leg_.init(ros::NodeHandle(controller_nh, "pid_left_leg")))
      return false;
  if (controller_nh.hasParam("pid_right_leg"))
    if (!pid_right_leg_.init(ros::NodeHandle(controller_nh, "pid_right_leg")))
      return false;
  if (controller_nh.hasParam("pid_theta_diff"))
    if (!pid_theta_diff_.init(ros::NodeHandle(controller_nh, "pid_theta_diff")))
      return false;
  if (controller_nh.hasParam("pid_roll"))
    if (!pid_roll_.init(ros::NodeHandle(controller_nh, "pid_roll")))
      return false;

  if (!controller_nh.getParam("block_angle", block_angle_))
  {
    ROS_INFO("Can't get param block_angle in ns: %s", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!loadDynamicsParams(controller_nh))
  {
    ROS_INFO("Fail to init dynamics params");
  }
  if (!loadVMCParams(controller_nh))
  {
    ROS_INFO("Fail to init vmc params");
  }

  q_.setZero();
  r_.setZero();
  XmlRpc::XmlRpcValue q, r;
  controller_nh.getParam("q", q);
  controller_nh.getParam("r", r);
  // Check and get Q
  ROS_ASSERT(q.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(q.size() == STATE_DIM);
  for (int i = 0; i < STATE_DIM; ++i)
  {
    ROS_ASSERT(q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || q[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (q[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      q_(i, i) = static_cast<double>(q[i]);
    }
    else if (q[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      q_(i, i) = static_cast<int>(q[i]);
    }
  }
  // Check and get R
  ROS_ASSERT(r.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_ASSERT(r.size() == INPUT_DIM);
  for (int i = 0; i < INPUT_DIM; ++i)
  {
    ROS_ASSERT(r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble || r[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    if (r[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      r_(i, i) = static_cast<double>(r[i]);
    }
    else if (r[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      r_(i, i) = static_cast<int>(r[i]);
    }
  }

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
  Eigen::Matrix<double, STATE_DIM, INPUT_DIM> b{};
  Eigen::Matrix<double, INPUT_DIM, STATE_DIM> k{};
  Eigen::Matrix<double, STATE_DIM, 1> x;
  XmlRpc::XmlRpcValue leg_length;
  controller_nh.getParam("leg_length", leg_length);
  for (int i = 0; i < leg_length.size(); i++)
  {
    a = generateA(leg_length[i], leg_length[i]);
    b = generateB(leg_length[i], leg_length[i]);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR in leg_length: %f", static_cast<double>(leg_length[i]));
      return false;
    }
    k = lqr.getK();
    k_pair_.push_back(std::make_pair(static_cast<double>(leg_length[i]), k));
    ROS_INFO_STREAM("K of LQR:" << k << "in leg_length: " << static_cast<double>(leg_length[i]));
  }

  observation_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(controller_nh, "state", 100));
  observation_publisher_->msg_.data.resize(STATE_DIM);
  input_publisher_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(controller_nh, "input", 100));
  input_publisher_->msg_.data.resize(INPUT_DIM);

  // Pub leg info
  leg_length_publisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("pendulum_length", 1);
  leg_pendulum_support_force_publisher_ = controller_nh.advertise<std_msgs::Float64MultiArray>("support_force", 1);

  // Sub leg cmd
  leg_cmd_sub_ = controller_nh.subscribe("leg_cmd", 1, &LeggedBalanceLQRController::legCmdCallback, this);

  return true;
}

void LeggedBalanceLQRController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  // State Estimate
  updateStateEstimation(time, period);
  // Update the current state of the system

  /*todo: check block
  // Check block
  double blockAngle = 0.25, blockEffort = 1.5, blockVelocity = 3.0, blockDuration = 0.5;//todo: use param

  if (!blockState_) {
    if (std::abs(observation_state_(1)) > blockAngle &&
        (std::abs(joint_handles_[0].getEffort()) > blockEffort || std::abs(joint_handles_[1].getEffort()) > blockEffort)
  && (std::abs(joint_handles_[0].getVelocity()) < blockVelocity || std::abs(joint_handles_[1].getVelocity()) <
  blockVelocity)) { if (!maybeBlock_) { maybeBlockTime_ = time; maybeBlock_ = true;
      }
      if ((time - maybeBlockTime_).toSec() >= blockDuration) {
        blockState_ = true;
        blockStateChanged_ = true;
      }
    } else {
      maybeBlock_ = false;
    }
  }
   */

  // Move joints
  if (!block_state_)
  {
    normal(time, period);
  }
  else
  {
    block(time, period);
  }

  /*todo: check sit down
  // Sit down
  if (balanceInterface_->getLeggedBalanceControlCmd()->getSitDown()) {
    sitDown(time, period);
  }
   */

  // Power limit
  /*
  double limit = balanceInterface_->getLeggedBalanceControlCmd()->getPowerLimit();
  double a = 0., b = 0., c = 0.;  // Three coefficients of a quadratic equation in one variable
  for (const auto& joint : joint_handles_) {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    a += square(cmd_effort);
    b += std::abs(cmd_effort * real_vel);
    c += square(real_vel);
  }
  a *= params_.powerCoeffEffort_;
  c = c * params_.powerCoeffVel_ + params_.powerOffset_ - limit;  // offset different from rm_chassis_controller
  double zoom = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  for (auto joint : joint_handles_) {
    joint.setCommand(zoom > 1 ? joint.getCommand() : joint.getCommand() * zoom);
  }
   */
}

void LeggedBalanceLQRController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  std::vector<double> jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
  for (size_t i = 0; i < CONTROL_DIM; ++i)
  {
    jointPos.at(i) = joint_handles_[i].getPosition();
    jointVel.at(i) = joint_handles_[i].getVelocity();
  }
  geometry_msgs::Vector3 gyro;
  gyro.x = imu_sensor_handle_.getAngularVelocity()[0];
  gyro.y = imu_sensor_handle_.getAngularVelocity()[1];
  gyro.z = imu_sensor_handle_.getAngularVelocity()[2];
  try
  {
    tf2::doTransform(gyro, gyro,
                     robot_state_handle_.lookupTransform("base_link", imu_sensor_handle_.getFrameId(), time));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2imu, imu2base, odom2base;
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_sensor_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    joint_handles_[0].setCommand(0.);
    joint_handles_[1].setCommand(0.);
    return;
  }
  tf2::Quaternion odom2imuQuaternion;
  tf2::Vector3 odom2imu_origin;
  odom2imuQuaternion.setValue(imu_sensor_handle_.getOrientation()[0], imu_sensor_handle_.getOrientation()[1],
                              imu_sensor_handle_.getOrientation()[2], imu_sensor_handle_.getOrientation()[3]);
  odom2imu_origin.setValue(0, 0, 0);
  odom2imu.setOrigin(odom2imu_origin);
  odom2imu.setRotation(odom2imuQuaternion);
  odom2base = odom2imu * imu2base;
  double pitch = 0, yaw = 0;
  quatToRPY(toMsg(odom2base).rotation, roll_, pitch, yaw);

  //  try {
  //    double pitchUnsed = 0;
  //    odom2base_ = robotStateHandle_.lookupTransform("odom", "base_link", ros::Time(0));
  //    //    if (abs((odom2base_.header.stamp - time).toSec()) < 0.001) {
  //    quatToRPY(odom2base_.transform.rotation, roll, pitchUnsed, yaw);
  //    //    }
  //  } catch (tf2::TransformException& ex) {
  //  }

  // currentObservation_.time += period.toSec();
  double yaw_last = observation_state_[4];
  matrix_t left(4, 1), right(4, 1);
  left = vmc_->jointPos2Pendulum(3.78 - jointPos.at(2), 3.78 + jointPos.at(4), -jointVel.at(2), -jointVel.at(4));
  right = vmc_->jointPos2Pendulum(3.78 - jointPos.at(3), 3.78 + jointPos.at(5), -jointVel.at(3), -jointVel.at(5));

  pendulum_length_[0] = left(0);
  pendulum_length_[1] = right(0);
  // ROS_INFO_THROTTLE(2, "pendulum_len: %f %f", pendulumLength[0], pendulumLength[1]);

  std_msgs::Float64MultiArray pendulum_length_pub_data;
  pendulum_length_pub_data.data.push_back(pendulum_length_[0]);
  pendulum_length_pub_data.data.push_back(pendulum_length_[1]);
  leg_length_publisher_.publish(pendulum_length_pub_data);

  observation_state_[9] = gyro.z;  // may need change
  observation_state_[8] = gyro.y;
  observation_state_[7] = right(3) + gyro.y;
  observation_state_[6] = left(3) + gyro.y;
  observation_state_[5] = (jointVel[0] + jointVel[1]) / 2. * param_.r_;
  observation_state_[4] = yaw_last + angles::shortest_angular_distance(yaw_last, yaw);
  observation_state_[3] = pitch;
  observation_state_[2] = right(1) + pitch;
  observation_state_[1] = left(1) + pitch;
  observation_state_[0] += observation_state_[5] * period.toSec();

  // Publish the observation. Only needed for the command interface
  for (int i = 0; i < STATE_DIM; i++)
    observation_publisher_->msg_.data.at(i) = observation_state_[i];
  observation_publisher_->unlockAndPublish();
}

geometry_msgs::Twist LeggedBalanceLQRController::odometry()
{
  geometry_msgs::Twist twist;
  twist.linear.x = observation_state_[5];
  twist.angular.z = observation_state_[9];
  return twist;
}

void LeggedBalanceLQRController::normal(const ros::Time& time, const ros::Duration& period)
{
  if (block_state_changed_)
  {
    ROS_INFO("[balance] Enter BLOCK");
    block_state_changed_ = false;
  }

  std::vector<double> jointPos(CONTROL_DIM), jointVel(CONTROL_DIM);
  for (size_t i = 0; i < CONTROL_DIM; ++i)
  {
    jointPos.at(i) = joint_handles_[i].getPosition();
    jointVel.at(i) = joint_handles_[i].getVelocity();
  }

  Eigen::Matrix<double, STATE_DIM, 1> x;
  for (int i = 0; i < STATE_DIM; i++)
    x(i) = observation_state_[i];

  double yaw_des = x(4), position_des = x(0);
  yaw_des += vel_cmd_.z * period.toSec();
  position_des += vel_cmd_.x * period.toSec();
  Eigen::Matrix<double, INPUT_DIM, 1> u;
  x(0) -= position_des;
  x(1) = angles::shortest_angular_distance(yaw_des, x(1));
  if (state_ != RAW)
    x(5) -= vel_cmd_.x;
  x(6) -= vel_cmd_.z;
  if (std::abs(x(0) + position_offset_) > position_clear_threshold_)
  {
    x[0] = 0.;
    position_des = position_offset_;
  }
  u = k_pair_.at(leg_length_index_cmd_).second * (-x);

  // Leg control
  matrix_t F_bl(2, 1), J(2, 3), p(3, 1), F_leg(2, 1), legLength(2, 1);
  double F_roll, F_gravity, F_inertial;
  // clang-format off
  J << 1, 1, -1,
      -1, 1, 1;
  // clang-format on
  legLength << pendulum_length_[0], pendulum_length_[1];
  F_roll = pid_roll_.computeCommand(0 - roll_, period);  // todo: dynamic roll angle
  F_leg(0) = pid_left_leg_.computeCommand(leg_length_cmd_ - pendulum_length_[0], period);
  F_leg(1) = pid_right_leg_.computeCommand(leg_length_cmd_ - pendulum_length_[1], period);
  F_gravity = (1. / 2 * param_.massBody_) * param_.g_;
  F_inertial = (1. / 2 * param_.massBody_) * (legLength(0) + legLength(1)) / 2 / (2 * param_.d_) *
               observation_state_[5] * observation_state_[9];
  // F_gravity = 0.5;
  F_inertial = 0;
  p << F_roll, F_gravity, F_inertial;
  F_bl = J * p + F_leg;

  std_msgs::Float64MultiArray leg_force;
  leg_force.data.push_back(F_bl(0));
  leg_force.data.push_back(F_bl(1));
  leg_pendulum_support_force_publisher_.publish(leg_force);

  double T_theta_diff = pid_theta_diff_.computeCommand(observation_state_[1] - observation_state_[2], period);
  matrix_t left_eff, right_eff;
  left_eff = vmc_->pendulumEff2JointEff(F_bl(0), u(0) - T_theta_diff, 3.78 - jointPos.at(2), 3.78 + jointPos.at(4));
  right_eff = vmc_->pendulumEff2JointEff(F_bl(1), u(1) + T_theta_diff, 3.78 - jointPos.at(3), 3.78 + jointPos.at(5));

  //  joint_handles_[0].setCommand(kp * (leftWheelPosRef - joint_handles_[0].getPosition()) +
  //                              kd * (leftWheelVelRef - joint_handles_[0].getVelocity()) + optimizedInput(2));
  //  joint_handles_[1].setCommand(kp * (rightWheelPosRef - joint_handles_[1].getPosition()) +
  //                              kd * (rightWheelVelRef - joint_handles_[1].getVelocity()) + optimizedInput(3));
  joint_handles_[0].setCommand(u(2));
  joint_handles_[1].setCommand(u(3));
  joint_handles_[2].setCommand(left_eff(0));
  joint_handles_[3].setCommand(right_eff(0));
  joint_handles_[4].setCommand(left_eff(1));
  joint_handles_[5].setCommand(right_eff(1));

  for (int i = 0; i < INPUT_DIM; i++)
  {
    input_state_[i] = u(i);
    input_publisher_->msg_.data.at(i) = u(i);
  }
  input_publisher_->unlockAndPublish();
}

void LeggedBalanceLQRController::block(const ros::Time& time, const ros::Duration& period)
{
  //  // todo: use param
  //  double antiBlockEffort = 3.0, antiBlockTime = 0.3;
  //  //
  //  if (blockStateChanged_)
  //  {
  //    ROS_INFO("[balance] Enter BLOCK");
  //    blockStateChanged_ = false;
  //
  //    lastBlockTime_ = time;
  //  }
  //  if ((time - lastBlockTime_).toSec() > antiBlockTime)
  //  {
  //    blockState_ = false;
  //    blockStateChanged_ = true;
  //    ROS_INFO("[balance] Exit BLOCK");
  //  }
  //  else
  //  {
  //    joint_handles_[0].setCommand(observation_state_(1) > 0 ? -antiBlockEffort : antiBlockEffort);
  //    joint_handles_[1].setCommand(observation_state_(1) > 0 ? -antiBlockEffort : antiBlockEffort);
  //  }
}

void LeggedBalanceLQRController::sitDown(const ros::Time& time, const ros::Duration& period)
{
  //  double wheelSpeed = balanceInterface_->getLeggedBalanceControlCmd()->getForwardVel() / params_.r_;
  //  double followSpeed =
  //      pidFollow_.computeCommand(balanceInterface_->getLeggedBalanceControlCmd()->getYawError(), period) * params_.d_
  //      / 2 / params_.r_;
  //  leftWheelController_.setCommand(wheelSpeed - followSpeed);
  //  rightWheelController_.setCommand(wheelSpeed + followSpeed);
  //  leftWheelController_.update(time, period);
  //  rightWheelController_.update(time, period);
}

void LeggedBalanceLQRController::legCmdCallback(const std_msgs::UInt32ConstPtr& cmd)
{
  if (cmd->data > k_pair_.size())
  {
    ROS_WARN_THROTTLE(1.0, "Leg command index out of vector range! which is %d in %u", cmd->data,
                      static_cast<unsigned int>(k_pair_.size()));
  }
  else
  {
    leg_length_index_cmd_ = cmd->data;
    leg_length_cmd_ = k_pair_.at(leg_length_index_cmd_).first;
  }
}

bool LeggedBalanceLQRController::loadDynamicsParams(ros::NodeHandle& controller_nh)
{
  if (!controller_nh.hasParam("dynamics"))
  {
    ROS_INFO("Not set dynamics param in ns: %s", controller_nh.getNamespace().c_str());
    return false;
  }

  ros::NodeHandle dynamics_nh = ros::NodeHandle(controller_nh, "dynamics");

  return dynamics_nh.getParam("d", param_.d_) || dynamics_nh.getParam("l_c", param_.l_c) ||
         dynamics_nh.getParam("r", param_.r_) || dynamics_nh.getParam("massBody", param_.massBody_) ||
         dynamics_nh.getParam("massLeg", param_.massLeg_) || dynamics_nh.getParam("massWheel", param_.massWheel_) ||
         dynamics_nh.getParam("jWheel", param_.jWheel_) || dynamics_nh.getParam("kWheel", param_.kWheel_) ||
         dynamics_nh.getParam("i1", param_.i1) || dynamics_nh.getParam("i2", param_.i2) ||
         dynamics_nh.getParam("i3", param_.i3);
}

bool LeggedBalanceLQRController::loadVMCParams(ros::NodeHandle& controller_nh)
{
  if (!controller_nh.hasParam("vmc"))
  {
    ROS_INFO("Not set vmc param in ns: %s", controller_nh.getNamespace().c_str());
    return false;
  }
  ros::NodeHandle vmc_nh = ros::NodeHandle(controller_nh, "vmc");

  double l_a, l_u, l_d;
  if (!vmc_nh.getParam("l_a", l_a) || !vmc_nh.getParam("l_u", l_u) || !vmc_nh.getParam("l_d", l_d))
    return false;

  vmc_ = std::make_shared<VMC>(l_a, l_u, l_d);
  return true;
}

matrix_t LeggedBalanceLQRController::generateA(double l_l, double l_r) const
{
  // Polynomial fitting
  double O = double(0);
  double a_16 = double(1), a_27 = double(1), a_38 = double(1), a_49 = double(1), a_510 = double(1);
  double a_62 = -0.24724045539306494 + (9.447623529293201) * l_l + (0.06812119732616839) * l_r +
                (3.94254240885233) * l_l * l_l + (-1.9141221364444747) * l_l * l_r + (-0.03956669731274465) * l_r * l_r;
  double a_63 = -0.022166488571220444 + (0.22758277197823715) * l_l + (-0.02461282034650958) * l_r +
                (-0.06151804979328948) * l_l * l_l + (0.7140215626015767) * l_l * l_r +
                (-0.3670949205534176) * l_r * l_r;
  double a_64 = -0.4065914942601602 + (-0.02550524662612847) * l_l + (-0.025505246626128394) * l_r +
                (-0.003631921659981067) * l_l * l_l + (0.00944309981904222) * l_l * l_r +
                (-0.003631921659981153) * l_r * l_r;
  double a_72 = 17.563500684716544 + (1092.7157958985156) * l_l + (-0.3624717848470027) * l_r +
                (-1949.5838327306492) * l_l * l_l + (6.745152655211655) * l_l * l_r + (0.12081521354821234) * l_r * l_r;
  double a_73 = 0.036482980071579754 + (-0.15695330598767016) * l_l + (-0.05980602949571537) * l_r +
                (-1.5450422108990476) * l_l * l_l + (-1.68294536922974) * l_l * l_r + (1.1575322206506211) * l_r * l_r;
  double a_74 = -0.15313827240843528 + (-2.1524246879005666) * l_l + (0.04250975593915829) * l_r +
                (3.952371535915845) * l_l * l_l + (0.13534889333744077) * l_l * l_r +
                (0.009993875096188598) * l_r * l_r;
  double a_82 = 1.285812323705684 + (-14.605889539758245) * l_l + (-12.672013823675613) * l_r +
                (-10.855667228907453) * l_l * l_l + (-61.613259768341955) * l_l * l_r + (42.94350504652487) * l_r * l_r;
  double a_83 = -4.0637597930871525 + (61.29598435795789) * l_l + (-10.77394207243573) * l_r +
                (0.17524201695610842) * l_l * l_l + (-134.59764057058482) * l_l * l_r +
                (50.259883058316596) * l_r * l_r;
  double a_84 = -0.1531382724084352 + (0.04250975593915851) * l_l + (-2.152424687900563) * l_r +
                (0.009993875096189625) * l_l * l_l + (0.13534889333744077) * l_l * l_r +
                (3.9523715359158396) * l_r * l_r;
  double a_92 = 0.3894966450966182 + (-14.883558044517315) * l_l + (-0.10731648983708553) * l_r +
                (-6.2109861388088685) * l_l * l_l + (3.0154618072718677) * l_l * l_r +
                (0.062332419815219975) * l_r * l_r;
  double a_93 = 0.03492055100099553 + (-0.3585284051770047) * l_l + (0.038774443025881206) * l_r +
                (0.09691405061230501) * l_l * l_l + (-1.1248523334007707) * l_l * l_r + (0.578312476250098) * l_r * l_r;
  double a_94 = 16.079217855208334 + (0.04018034984382026) * l_l + (0.04018034984382027) * l_r +
                (0.00572164170936371) * l_l * l_l + (-0.014876431500604432) * l_l * l_r +
                (0.005721641709363705) * l_r * l_r;
  double a_102 = 2.2013492171699287 + (-86.02233419621933) * l_l + (0.7970196059245538) * l_r +
                 (-38.640850484272455) * l_l * l_l + (-19.780990501849566) * l_l * l_r +
                 (-0.394745519379061) * l_r * l_r;
  double a_103 = -0.20137501719002096 + (1.9036930861551788) * l_l + (-0.1785326860396046) * l_r +
                 (0.6059016266073536) * l_l * l_l + (6.898595406819867) * l_l * l_r + (-3.682428873513967) * l_r * l_r;
  double a_104 = 4.1066923104757096e-18 + (0.23407594217239253) * l_l + (-0.23407594217239208) * l_r +
                 (0.035592597346284574) * l_l * l_l + (9.367506770274758e-17) * l_l * l_r +
                 (-0.03559259734628466) * l_r * l_r;

  matrix_t A =
      (matrix_t(10, 10) <<  // clang-format off
                         O,   O,   O,   O,   O,a_16,   O,   O,   O,   O,
                         O,   O,   O,   O,   O,   O,a_27,   O,   O,   O,
                         O,   O,   O,   O,   O,   O,   O,a_38,   O,   O,
                         O,   O,   O,   O,   O,   O,   O,   O,a_49,   O,
                         O,   O,   O,   O,   O,   O,   O,   O,   O,a_510,
                         O,a_62,a_63,a_64,   O,   O,   O,   O,   O,   O,
                         O,a_72,a_73,a_74,   O,   O,   O,   O,   O,   O,
                         O,a_82,a_83,a_84,   O,   O,   O,   O,   O,   O,
                         O,a_92,a_93,a_94,   O,   O,   O,   O,   O,   O,
                         O,a_102,a_103,a_104,O,   O,   O,   O,   O,   O
                      ).finished();  // clang-format on

  return A;
}

matrix_t LeggedBalanceLQRController::generateB(double l_l, double l_r) const
{
  // Polynomial fitting
  double O = double(0);
  double b_61 = 0.1304111001576438 + (0.3602745011975263) * l_l + (-0.0003827527551188703) * l_r +
                (-0.6484876653842095) * l_l * l_l + (-0.024684041604198147) * l_l * l_r +
                (-0.0007019854127603281) * l_r * l_r;
  double b_62 = 0.1304111001576438 + (-0.0003827527551187593) * l_l + (0.36027450119752646) * l_r +
                (-0.0007019854127606456) * l_l * l_l + (-0.024684041604198667) * l_l * l_r +
                (-0.64848766538421) * l_r * l_r;
  double b_63 = 0.5487094330064273 + (-0.41430753532309267) * l_l + (-0.09931990703780416) * l_r +
                (-0.19658342071762983) * l_l * l_l + (0.1032174384033582) * l_l * l_r +
                (-0.012410005346130404) * l_r * l_r;
  double b_64 = 0.5487094330064273 + (-0.09931990703780413) * l_l + (-0.41430753532309283) * l_r +
                (-0.012410005346130445) * l_l * l_l + (0.10321743840335858) * l_l * l_r +
                (-0.19658342071762963) * l_r * l_r;
  double b_71 = 12.633366364121917 + (-45.50031168582462) * l_l + (-0.001674041984454533) * l_r +
                (62.239950184802694) * l_l * l_l + (0.09051869562527326) * l_l * l_r +
                (0.002106340813647023) * l_r * l_r;
  double b_72 = 0.05216187366144019 + (-0.27509024794901676) * l_l + (-0.8432609796401014) * l_r +
                (0.7624074299020027) * l_l * l_l + (-0.7988037884331599) * l_l * l_r + (1.7829133527267544) * l_r * l_r;
  double b_73 = -0.4181294383074379 + (-50.381076368422356) * l_l + (0.17435420679628777) * l_r +
                (89.12078733992048) * l_l * l_l + (0.27676439054641744) * l_l * l_r + (0.03372942074201024) * l_r * l_r;
  double b_74 = -0.6244738447194612 + (-7.733810806919665) * l_l + (0.6341733176058044) * l_r +
                (13.508975691499966) * l_l * l_l + (2.6819539982950062) * l_l * l_r + (0.5410929744818938) * l_r * l_r;
  double b_81 = 0.05216187366144022 + (-0.8432609796401005) * l_l + (-0.27509024794901926) * l_r +
                (1.7829133527267529) * l_l * l_l + (-0.7988037884331596) * l_l * l_r + (0.7624074299020069) * l_r * l_r;
  double b_82 = 12.633366364121919 + (-0.001674041984429664) * l_l + (-45.500311685824684) * l_r +
                (0.0021063408136337003) * l_l * l_l + (0.09051869562519554) * l_l * l_r +
                (62.239950184802815) * l_r * l_r;
  double b_83 = -0.6244738447194612 + (0.6341733176058064) * l_l + (-7.733810806919674) * l_r +
                (0.5410929744818944) * l_l * l_l + (2.6819539982950067) * l_l * l_r + (13.508975691499986) * l_r * l_r;
  double b_84 = -0.41812943830743965 + (0.17435420679625935) * l_l + (-50.3810763684224) * l_r +
                (0.03372942074210217) * l_l * l_l + (0.2767643905464785) * l_l * l_r + (89.12078733992055) * l_r * l_r;
  double b_91 = -4.201833765005014 + (-0.5675677522402356) * l_l + (0.000602979450847807) * l_r +
                (1.0216118136982388) * l_l * l_l + (0.03888664327598085) * l_l * l_r +
                (0.0011058908734895845) * l_r * l_r;
  double b_92 = -4.201833765005013 + (0.0006029794508467246) * l_l + (-0.5675677522402378) * l_r +
                (0.0011058908734910104) * l_l * l_l + (0.038886643275982256) * l_l * l_r +
                (1.0216118136982468) * l_r * l_r;
  double b_93 = -0.8644235950346255 + (0.6526900898562241) * l_l + (0.156466183986885) * l_r +
                (0.3096927755184884) * l_l * l_l + (-0.16260626081463805) * l_l * l_r +
                (0.0195504228475243) * l_r * l_r;
  double b_94 = -0.8644235950346254 + (0.1564661839868854) * l_l + (0.6526900898562216) * l_r +
                (0.01955042284752498) * l_l * l_l + (-0.16260626081463958) * l_l * l_r +
                (0.30969277551849267) * l_r * l_r;
  double b_101 = -0.22319818037202083 + (-3.445331243900265) * l_l + (-0.0016611782587174773) * l_r +
                 (6.354275711437241) * l_l * l_l + (-0.257553107575101) * l_l * l_r +
                 (-0.006969894111447161) * l_r * l_r;
  double b_102 = 0.22319818037202083 + (0.0016611782587176993) * l_l + (3.445331243900267) * l_r +
                 (0.00696989411144619) * l_l * l_l + (0.25755310757509814) * l_l * l_r +
                 (-6.3542757114372455) * l_r * l_r;
  double b_103 = -5.0465793155039185 + (3.7689647437321603) * l_l + (-0.9175845790159323) * l_r +
                 (1.9266014036934025) * l_l * l_l + (0.6959807006530497) * l_l * l_r +
                 (-0.12137111793094524) * l_r * l_r;
  double b_104 = 5.046579315503919 + (0.917584579015932) * l_l + (-3.768964743732158) * l_r +
                 (0.12137111793094277) * l_l * l_l + (-0.695980700653051) * l_l * l_r +
                 (-1.9266014036934143) * l_r * l_r;

  matrix_t B = (matrix_t(10, 4) <<  // clang-format off
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                         O,   O,   O,   O,
                      b_61,b_62,b_63,b_64,
                      b_71,b_72,b_73,b_74,
                      b_81,b_82,b_83,b_84,
                      b_91,b_92,b_93,b_94,
                      b_101,b_102,b_103,b_104
                      ).finished();  // clang-format on

  return B;
}

VMC::VMC(double l_a, double l_u, double l_d)
{
  l_a_ = l_a;
  l_u_ = l_u;
  l_d_ = l_d;
}

matrix_t VMC::pendulumEff2JointEff(double F_bl, double T_bl, double front_joint_angle, double back_joint_angle) const
{
  double xe, ye, x1, y1, x2, y2, e;
  matrix_t joint(2, 1), virtual_eff(2, 1);
  double l(0), theta(0);
  matrix_t J(2, 2), J_inv(2, 2), J_inv_T(2, 2);
  double j11(0), j12(0), j21(0), j22(0);

  x1 = l_a_ - l_u_ * cos(front_joint_angle);
  x2 = l_a_ - l_u_ * cos(back_joint_angle);
  y1 = l_u_ * sin(front_joint_angle);
  y2 = l_u_ * sin(back_joint_angle);
  e = sqrt(-(-l_d_ * l_d_ - 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 +
             y2 * y2) *
           (-l_d_ * l_d_ + 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 +
            y2 * y2));

  xe = (y1 * e - y2 * e + x1 * x2 * x2 - x1 * x1 * x2 - x1 * y1 * y1 - x1 * y2 * y2 + x2 * y1 * y1 + x2 * y2 * y2 -
        x1 * x1 * x1 + x2 * x2 * x2 + 2 * x1 * y1 * y2 - 2 * x2 * y1 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));
  ye = (x1 * e + x2 * e + x1 * x1 * y1 + x1 * x1 * y2 + x2 * x2 * y1 + x2 * x2 * y2 - y1 * y2 * y2 - y1 * y1 * y2 +
        y1 * y1 * y1 + y2 * y2 * y2 + 2 * x1 * x2 * y1 + 2 * x1 * x2 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  l = sqrt(xe * xe + ye * ye);
  theta = atan(xe / ye);

  j11 = 1 / l_u_ * ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j12 = l / l_u_ * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j21 = 1 / l_u_ * ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));
  j22 = l / l_u_ * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));

  J << j11, j12, j21, j22;
  double det = j11 * j22 - j12 * j21;
  J_inv = 1 / det * (matrix_t(2, 2) << j22, -j12, -j21, j11).finished();
  J_inv_T = (matrix_t(2, 2) << J_inv(0, 0), J_inv(1, 0), J_inv(0, 1), J_inv(1, 1)).finished();

  virtual_eff << F_bl, T_bl;
  joint = J_inv_T * virtual_eff;

  joint(0) = -joint(0);

  return joint;
}

matrix_t VMC::jointPos2Pendulum(double front_joint_angle, double back_joint_angle, double front_joint_vel,
                                double back_joint_vel) const
{
  // back_joint: psi_2 front_joint: psi_1
  double xe(0), ye(0), x1(0), y1(0), x2(0), y2(0), e(0);
  double l(0), theta(0);
  double j11(0), j12(0), j21(0), j22(0);

  x1 = l_a_ - l_u_ * cos(front_joint_angle);
  x2 = l_a_ - l_u_ * cos(back_joint_angle);
  y1 = l_u_ * sin(front_joint_angle);
  y2 = l_u_ * sin(back_joint_angle);
  e = sqrt(-(-l_d_ * l_d_ - 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 +
             y2 * y2) *
           (-l_d_ * l_d_ + 2 * l_d_ * l_d_ - l_d_ * l_d_ + x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 +
            y2 * y2));

  xe = (y1 * e - y2 * e + x1 * x2 * x2 - x1 * x1 * x2 - x1 * y1 * y1 - x1 * y2 * y2 + x2 * y1 * y1 + x2 * y2 * y2 -
        x1 * x1 * x1 + x2 * x2 * x2 + 2 * x1 * y1 * y2 - 2 * x2 * y1 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));
  ye = (x1 * e + x2 * e + x1 * x1 * y1 + x1 * x1 * y2 + x2 * x2 * y1 + x2 * x2 * y2 - y1 * y2 * y2 - y1 * y1 * y2 +
        y1 * y1 * y1 + y2 * y2 * y2 + 2 * x1 * x2 * y1 + 2 * x1 * x2 * y2) /
       (2 * (x1 * x1 + 2 * x1 * x2 + x2 * x2 + y1 * y1 - 2 * y1 * y2 + y2 * y2));

  l = sqrt(xe * xe + ye * ye);
  theta = atan(xe / ye);

  j11 = 1 / l_u_ * ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j12 = l / l_u_ * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta)) /
        (-(xe + x1) * sin(front_joint_angle) + (ye - y1) * cos(front_joint_angle));
  j21 = 1 / l_u_ * ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));
  j22 = l / l_u_ * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta)) /
        ((xe - x2) * sin(back_joint_angle) + (ye - y2) * cos(back_joint_angle));

  matrix_t J(2, 2), J_inv(2, 2);
  J << j11, j12, j21, j22;
  double det = j11 * j22 - j12 * j21;
  J_inv = 1 / det * (matrix_t(2, 2) << j22, -j12, -j21, j11).finished();

  matrix_t joint_dot(2, 1), pendulum_dot(2, 1);
  joint_dot << front_joint_vel, back_joint_vel;
  pendulum_dot = J_inv * joint_dot;

  matrix_t m(4, 1);
  m << l, theta, pendulum_dot(0), pendulum_dot(1);

  return m;
}
}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::LeggedBalanceLQRController, controller_interface::ControllerBase)
