// Copyright 2025 Ioannis Tsikelis

#include <hurocore/root_node.h>

#include <memory>
#include <string>

namespace hurocore {

using ImuStateMsg = RootNode::ImuStateMsg;
using MotorStateMsg = RootNode::MotorStateMsg;
using LowStateMsg = RootNode::LowStateMsg;
using OdometryMsg = RootNode::OdometryMsg;

using JointStateMsg = RootNode::JointStateMsg;
using TransformStamped = RootNode::TransformStamped;

using TransformBroadcaster = RootNode::TransformBroadcaster;

RootNode::RootNode() : Node("root_node") {
  this->declare_parameter("hfreq", true);
  this->declare_parameter("info_imu", false);
  this->declare_parameter("info_motor", false);
  this->declare_parameter("num_motors", 0);
  this->declare_parameter<std::vector<std::string>>("joint_names");

  this->get_parameter("high_freq", hfreq_);
  this->get_parameter("info_imu", info_imu_);
  this->get_parameter("info_motor", info_motor_);
  this->get_parameter("num_motors", num_motors_);
  this->get_parameter("joint_names", joint_names_);

  // Update topic names conditionally
  std::string ls_topic = hfreq_ ? "/lowstate" : "/lf/lowstate";
  std::string odom_topic = hfreq_ ? "/odommodestate" : "/lf/odommodestate";

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<TransformBroadcaster>(*this);

  // Set up publishers
  jointstate_pub_ = this->create_publisher<JointStateMsg>("/joint_states", 10);

  // Set up subscribers
  lowstate_sub_ = this->create_subscription<LowStateMsg>(
      ls_topic, 10,
      std::bind(&RootNode::LowStateHandler, this, std::placeholders::_1));
  odometry_sub_ = this->create_subscription<OdometryMsg>(
      odom_topic, 10,
      std::bind(&RootNode::OdometryHandler, this, std::placeholders::_1));
}

void RootNode::LowStateHandler(LowStateMsg::SharedPtr message) {
  if (info_imu_) {
    ImuStateMsg imu = message->imu_state;
    RCLCPP_INFO(this->get_logger(),
                "Euler angle -- roll: %f; pitch: %f; yaw: %f", imu.rpy[0],
                imu.rpy[1], imu.rpy[2]);
    RCLCPP_INFO(this->get_logger(),
                "Quaternion -- qw: %f; qx: %f; qy: %f; qz: %f",
                imu.quaternion[0], imu.quaternion[1], imu.quaternion[2],
                imu.quaternion[3]);
    RCLCPP_INFO(this->get_logger(), "Gyroscope -- wx: %f; wy: %f; wz: %f",
                imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2]);
    RCLCPP_INFO(this->get_logger(), "Accelerometer -- ax: %f; ay: %f; az: %f",
                imu.accelerometer[0], imu.accelerometer[1],
                imu.accelerometer[2]);
  }
  if (info_motor_) {
    for (int i = 0; i < num_motors_; i++) {
      MotorStateMsg motor = message->motor_state[i];
      RCLCPP_INFO(this->get_logger(),
                  "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f", i,
                  motor.q, motor.dq, motor.ddq, motor.tau_est);
    }
  }

  JointStateMsg jointstate_msg;
  jointstate_msg.header.stamp = this->now();
  for (int i = 0; i < num_motors_; ++i) {
    std::string joint_name = joint_names_[i];
    jointstate_msg.name.push_back(joint_name);
    jointstate_msg.position.push_back(message->motor_state[i].q);
    jointstate_msg.velocity.push_back(message->motor_state[i].dq);
    jointstate_msg.effort.push_back(message->motor_state[i].tau_est);
  }

  jointstate_pub_->publish(jointstate_msg);
}

void RootNode::OdometryHandler(OdometryMsg::SharedPtr message) {
  TransformStamped tf = GenerateTransformMsg(message);
  tf_broadcaster_->sendTransform(tf);
}

TransformStamped
RootNode::GenerateTransformMsg(OdometryMsg::SharedPtr message) {
  TransformStamped tf;

  tf.header.stamp = this->get_clock()->now();
  tf.header.frame_id = "world";
  tf.child_frame_id = "pelvis";

  tf.transform.translation.x = message->position[0];
  tf.transform.translation.y = message->position[1];
  tf.transform.translation.z = message->position[2];

  tf.transform.rotation.w = message->imu_state.quaternion[0];
  tf.transform.rotation.x = message->imu_state.quaternion[1];
  tf.transform.rotation.y = message->imu_state.quaternion[2];
  tf.transform.rotation.z = message->imu_state.quaternion[3];

  return tf;
}

} // namespace hurocore
