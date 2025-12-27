/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ros_gz_omni_gazebo/OmniDrive.hh"
#include "ros_gz_omni_gazebo/OmniMath.hh"

#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>

#include <chrono>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
// #include <gz/math/MecanumDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

// #include <gz/math/Pose3.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

GZ_ADD_PLUGIN(
    ros_gz_omni_gazebo::OmniDrive,
    gz::sim::System,
    ros_gz_omni_gazebo::OmniDrive::ISystemConfigure,
    ros_gz_omni_gazebo::OmniDrive::ISystemPreUpdate,
    ros_gz_omni_gazebo::OmniDrive::ISystemPostUpdate
  )
  
using namespace ros_gz_omni_gazebo;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Lateral velocity.
  double lat;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), lat(0.0), ang(0.0) {}
};

class ros_gz_omni_gazebo::OmniDrivePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const gz::msgs::Twist &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
  public: gz::transport::Node node;

  /// \brief Entity of the front left joint
  public: std::vector<gz::sim::Entity> frontLeftJoints;

  /// \brief Entity of the front right joint
  public: std::vector<gz::sim::Entity> frontRightJoints;

  /// \brief Entity of the back left joint
  public: std::vector<gz::sim::Entity> backLeftJoints;

  /// \brief Entity of the back right joint
  public: std::vector<gz::sim::Entity> backRightJoints;

  /// \brief Name of front left joint
  public: std::vector<std::string> frontLeftJointNames;

  /// \brief Name of front right joint
  public: std::vector<std::string> frontRightJointNames;

  /// \brief Name of back left joint
  public: std::vector<std::string> backLeftJointNames;

  /// \brief Name of back right joint
  public: std::vector<std::string> backRightJointNames;

  /// \brief Calculated speed of front left joint
  public: double frontLeftJointSpeed{0};

  /// \brief Calculated speed of front right joint
  public: double frontRightJointSpeed{0};

  /// \brief Calculated speed of back left joint
  public: double backLeftJointSpeed{0};

  /// \brief Calculated speed of back right joint
  public: double backRightJointSpeed{0};

  /// \brief Lateral distance between left and right wheels
  public: double wheelSeparation{1.0};

  /// \brief Longitudinal distance between front and back wheels
  public: double wheelbase{1.0};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  /// \brief Model interface
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief The model's canonical link.
  public: gz::sim::Link canonicalLink{gz::sim::kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Mecanum drive odometry.
  public: ros_gz_omni_gazebo::OmniMath odom{10};
  // public: gz::math::MecanumDriveOdometry odom;

  /// \brief Mecanum drive odometry message publisher.
  public: gz::transport::Node::Publisher odomPub;

  /// \brief Mecanum drive tf message publisher.
  public: gz::transport::Node::Publisher tfPub;

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<gz::math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<gz::math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: gz::msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;
};

//////////////////////////////////////////////////
ros_gz_omni_gazebo::OmniDrive::OmniDrive() : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

//////////////////////////////////////////////////
void OmniDrive::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);

  // Get the canonical link
  std::vector<gz::sim::Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), gz::sim::components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = gz::sim::Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "OmniDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("front_left_joint");
  while (sdfElem)
  {
    this->dataPtr->frontLeftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("front_left_joint");
  }
  sdfElem = _sdf->FindElement("front_right_joint");
  while (sdfElem)
  {
    this->dataPtr->frontRightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("front_right_joint");
  }

  sdfElem = _sdf->FindElement("back_left_joint");
  while (sdfElem)
  {
    this->dataPtr->backLeftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("back_left_joint");
  }
  sdfElem = _sdf->FindElement("back_right_joint");
  while (sdfElem)
  {
    this->dataPtr->backRightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("back_right_joint");
  }

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
  this->dataPtr->wheelbase = _sdf->Get<double>("wheelbase",
      this->dataPtr->wheelbase).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<gz::math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<gz::math::SpeedLimiter>();

  // Parse speed limiter parameters.
  if (_sdf->HasElement("min_velocity"))
  {
    double minVel = _sdf->Get<double>("min_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minVel);
    this->dataPtr->limiterAng->SetMinVelocity(minVel);
  }
  if (_sdf->HasElement("max_velocity"))
  {
    double maxVel = _sdf->Get<double>("max_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
    this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
  }
  if (_sdf->HasElement("min_acceleration"))
  {
    double minAccel = _sdf->Get<double>("min_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
    this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
  }
  if (_sdf->HasElement("max_acceleration"))
  {
    double maxAccel = _sdf->Get<double>("max_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
  }
  if (_sdf->HasElement("min_jerk"))
  {
    double minJerk = _sdf->Get<double>("min_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minJerk);
    this->dataPtr->limiterAng->SetMinJerk(minJerk);
  }
  if (_sdf->HasElement("max_jerk"))
  {
    double maxJerk = _sdf->Get<double>("max_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
    this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
      this->dataPtr->wheelbase,
      this->dataPtr->wheelRadius,
      this->dataPtr->wheelRadius);

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = gz::sim::validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &OmniDrivePrivate::OnCmdVel,
      this->dataPtr.get());

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = gz::sim::validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<gz::msgs::Odometry>(
      odomTopic);

  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/tf"};
  if (_sdf->HasElement("tf_topic"))
    tfTopic = _sdf->Get<std::string>("tf_topic");
  this->dataPtr->tfPub = this->dataPtr->node.Advertise<gz::msgs::Pose_V>(
      tfTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  gzmsg << "OmniDrive publishing odom messages on [" << odomTopic << "]"
         << std::endl;

  gzmsg << "OmniDrive subscribing to twist messages on [" << topic << "]"
         << std::endl;
}

//////////////////////////////////////////////////
void OmniDrive::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("OmniDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->frontLeftJoints.empty() ||
      this->dataPtr->frontRightJoints.empty() ||
      this->dataPtr->backLeftJoints.empty() ||
      this->dataPtr->backRightJoints.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->frontLeftJointNames)
    {
      gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != gz::sim::kNullEntity)
        this->dataPtr->frontLeftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->frontRightJointNames)
    {
      gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != gz::sim::kNullEntity)
        this->dataPtr->frontRightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->backLeftJointNames)
    {
      gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != gz::sim::kNullEntity)
        this->dataPtr->backLeftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->backRightJointNames)
    {
      gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != gz::sim::kNullEntity)
        this->dataPtr->backRightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->frontLeftJoints.empty() ||
      this->dataPtr->frontRightJoints.empty() ||
      this->dataPtr->backLeftJoints.empty() ||
      this->dataPtr->backRightJoints.empty())
  {
    return;
  }

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    gzmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  for (gz::sim::Entity joint : this->dataPtr->frontLeftJoints)
  {
    // Update wheel velocity
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
      {this->dataPtr->frontLeftJointSpeed});
  }

  for (gz::sim::Entity joint : this->dataPtr->frontRightJoints)
  {
    // Update wheel velocity
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
      {this->dataPtr->frontRightJointSpeed});
  }

  for (gz::sim::Entity joint : this->dataPtr->backLeftJoints)
  {
    // Update wheel velocity
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
      {this->dataPtr->backLeftJointSpeed});
  }

  for (gz::sim::Entity joint : this->dataPtr->backRightJoints)
  {
    // Update wheel velocity
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
      {this->dataPtr->backRightJointSpeed});
  }

  // Create the joint position components if they don't exist.
  auto frontLeftPos = _ecm.Component<gz::sim::components::JointPosition>(
      this->dataPtr->frontLeftJoints[0]);
  if (!frontLeftPos && _ecm.HasEntity(this->dataPtr->frontLeftJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->frontLeftJoints[0],
        gz::sim::components::JointPosition());
  }

  auto frontRightPos = _ecm.Component<gz::sim::components::JointPosition>(
      this->dataPtr->frontRightJoints[0]);
  if (!frontRightPos && _ecm.HasEntity(this->dataPtr->frontRightJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->frontRightJoints[0],
        gz::sim::components::JointPosition());
  }

  auto backLeftPos = _ecm.Component<gz::sim::components::JointPosition>(
      this->dataPtr->backLeftJoints[0]);
  if (!backLeftPos && _ecm.HasEntity(this->dataPtr->backLeftJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->backLeftJoints[0],
        gz::sim::components::JointPosition());
  }

  auto backRightPos = _ecm.Component<gz::sim::components::JointPosition>(
      this->dataPtr->backRightJoints[0]);
  if (!backRightPos && _ecm.HasEntity(this->dataPtr->backRightJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->backRightJoints[0],
        gz::sim::components::JointPosition());
  }

}

//////////////////////////////////////////////////
void OmniDrive::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("OmniDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateOdometry(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("OmniDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  if (this->frontLeftJoints.empty()  ||
      this->frontRightJoints.empty() ||
      this->backLeftJoints.empty()   ||
      this->backRightJoints.empty())
    return;

  // Get the first joint positions for each wheel joint.
  auto frontLeftPos = _ecm.Component<gz::sim::components::JointPosition>(
    this->frontLeftJoints[0]);
  auto frontRightPos = _ecm.Component<gz::sim::components::JointPosition>(
    this->frontRightJoints[0]);
  auto backLeftPos = _ecm.Component<gz::sim::components::JointPosition>(
    this->backLeftJoints[0]);
  auto backRightPos = _ecm.Component<gz::sim::components::JointPosition>(
    this->backRightJoints[0]);

  // Abort if the joints were not found or just created.
  if (!frontLeftPos || !frontRightPos || !backLeftPos || !backRightPos ||
   frontLeftPos->Data().empty() || frontRightPos->Data().empty() ||
   backLeftPos->Data().empty() || backRightPos->Data().empty())
  {
    return;
  }

  this->odom.Update(frontLeftPos->Data()[0],
                    frontRightPos->Data()[0],
                    backLeftPos->Data()[0],
                    backRightPos->Data()[0],
                    std::chrono::steady_clock::time_point(_info.simTime));

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  gz::msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
  msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

  gz::math::Quaterniond orientation(0, 0, *this->odom.Heading());
  gz::msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
  msg.mutable_twist()->mutable_linear()->set_y(this->odom.LateralVelocity());
  msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      gz::sim::convert<gz::msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  gz::msgs::Pose_V tfMsg;
  gz::msgs::Pose *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void OmniDrivePrivate::UpdateVelocity(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("OmniDrive::UpdateVelocity");

  double linVel;
  double latVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    latVel = this->targetVel.linear().y();
    angVel = this->targetVel.angular().z();
  }


  // if ((std::abs(linVel) > 0.0 && std::abs(angVel) > 0.0)) {
  //   linVel = 0.0;
  //   latVel = 0.0;
  //   angVel = 0.0;
  // }

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
      linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterLin->Limit(
      latVel, this->last0Cmd.lat, this->last1Cmd.lat, _info.dt);
  this->limiterAng->Limit(
      angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.lat = latVel;
  this->last0Cmd.ang = angVel;
  
  // constant used in computing target velocities
  // const double angularLength = std::sqrt(
  //       std::pow(this->wheelSeparation / 2.0, 2) + 
  //       std::pow(this->wheelbase / 2.0, 2)
  //   );
  const double angularLength = 0.5 * (this->wheelSeparation + this->wheelbase);
  const double invWheelRadius = 1 / this->wheelRadius;
  

  // Convert the target velocities to joint velocities.
  // These calculations are based on the following references:
  // https://robohub.org/drive-kinematics-skid-steer-and-mecanum-ros-twist-included
  // https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
	// https://easychair.org/publications/preprint/jHj9/open

	// https://en.cppreference.com/index.html
	
	double sqtwoinv = 1/std::sqrt(2);

	this->frontLeftJointSpeed = sqtwoinv * (linVel - latVel + (-angVel * angularLength)) * invWheelRadius;

	this->frontRightJointSpeed = sqtwoinv * (-linVel - latVel + (-angVel * angularLength)) * invWheelRadius;

	this->backLeftJointSpeed = sqtwoinv * (linVel + latVel + (-angVel * angularLength)) * invWheelRadius;

	this->backRightJointSpeed = sqtwoinv * (-linVel + latVel + (-angVel * angularLength)) * invWheelRadius;

  // gzdbg << "Speed:" << this->frontLeftJointSpeed << ", " << this->frontRightJointSpeed << ", " << this->backRightJointSpeed << ", " << this->backLeftJointSpeed << std::endl;

  // finally motor speed of each wheel is in rad/s takes linevel, latvel, angvel ie. x, y, z 
}

//////////////////////////////////////////////////
void OmniDrivePrivate::OnCmdVel(const gz::msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}
