// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>

#include "Gimbal2dPlugin.hpp"

#include <gazebo/common/PID.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/float32.hpp>

#include <memory>

using namespace std;

namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GimbalPluginPrivate
{
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publisher to the gimbal status topic
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub;

  /// Subscriber to the gimbal command topic
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub;

  /// Parent model of this plugin
  gazebo::physics::ModelPtr model;

  /// Joint for tilting the gimbal
  gazebo::physics::JointPtr tiltJoint;

  /// Command that updates the gimbal tilt angle
  double command = IGN_PI_2;

  /// PID controller for the gimbal
  gazebo::common::PID pid;

  /// Last update sim time
  gazebo::common::Time lastUpdateTime;

};

GimbalPlugin::GimbalPlugin()
: impl_(std::make_unique<GimbalPluginPrivate>())
{
  this->impl_->pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

GimbalPlugin::~GimbalPlugin()
{
}

void GimbalPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // The model pointer gives you direct access to the physics object,
  // for example:
  // RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GimbalPlugin::OnUpdate, this));

  // Set model
  impl_->model = model;
  
  // Get Joint Details
  std::string jointName = "tilt_joint";
  if (sdf->HasElement("joint"))
  {
    jointName = sdf->Get<std::string>("joint");
  }
  impl_->tiltJoint = impl_->model->GetJoint(jointName);
  if (!impl_->tiltJoint)
  {
    std::string scopedJointName = model->GetScopedName() + "::" + jointName;
    RCLCPP_WARN(impl_->ros_node_->get_logger(), "joint [%s] not found, trying again with scoped joint name [%s]", jointName.c_str(), scopedJointName.c_str());
    impl_->tiltJoint = impl_->model->GetJoint(scopedJointName);
  }
  if (!impl_->tiltJoint)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Gimbal2dPlugin::Load ERROR! Can't get joint %s", jointName.c_str());
  }

  // Get initial angle details
  if (sdf->HasElement("initial_angle"))
  {
    impl_->command = sdf->Get<double>("initial_angle");
  }

  // Initialise time
  impl_->lastUpdateTime = model->GetWorld()->SimTime();

  // Gimbal state publisher
  impl_->pub = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
    "gimbal_tilt_status", qos.get_publisher_qos("grasping", rclcpp::QoS(1)));
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Advertise gimbal status on [%s]", impl_->pub->get_topic_name());

  // Gimbal subscription, callback simply sets the command
  impl_->sub = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    "gimbal_tilt_cmd", 10,
    [this](const std_msgs::msg::Float32::SharedPtr msg){
      if(msg) {
        this->impl_->command = msg->data;
      } else {
        RCLCPP_WARN(this->impl_->ros_node_->get_logger(), "Received Gimbal Plugin Angle not valid");
      }
    }
  );
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(),
    "Receiving gimbal pose2d theta cmd on [%s]", impl_->sub->get_topic_name()); 
}

void GimbalPlugin::OnUpdate()
{
  // Do something every simulation iteration
  
  // If not initialised yet 
  if (!impl_->tiltJoint){return;}

  // 
  double angle = impl_->tiltJoint->Position(0);

  // Get current time
  gazebo::common::Time time = impl_->model->GetWorld()->SimTime();
  if (time < impl_->lastUpdateTime)
  {
    impl_->lastUpdateTime = time;
  }
  else if (time > impl_->lastUpdateTime)
  {
    double dt = (impl_->lastUpdateTime - time).Double();
    double error = angle - impl_->command;
    double force = impl_->pid.Update(error, dt);
    impl_->tiltJoint->SetForce(0, force);
    impl_->lastUpdateTime = time;
  }

  auto msg = std_msgs::msg::Float32();
  msg.data = angle;
  impl_->pub->publish(msg);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GimbalPlugin)
}  // namespace gazebo_plugins
