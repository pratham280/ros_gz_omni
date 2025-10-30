/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <cmath>
#include <chrono>
#include "ros_gz_omni_gazebo/OmniMath.hh"
#include "gz/math/RollingMean.hh"

#include <string>
#include <gz/common/Console.hh>

// The implementation was borrowed from: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/odometry.cpp
// And these calculations are based on the following references:
// https://robohub.org/drive-kinematics-skid-steer-and-mecanum-ros-twist-included
// https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf

namespace ros_gz_omni_gazebo
{
  
class OmniMath::Implementation
{
  /// \brief Constructor.
  /// \param[in] _windowSize Rolling window size used to compute the
  /// velocity mean.
  public: explicit Implementation(size_t _windowSize)
    : linearMean(_windowSize), lateralMean(_windowSize),
      angularMean(_windowSize)
  {
  }

  /// \brief Integrates the pose.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _lateral, double _angular);

  /// \brief Integrates the pose using second order Runge-Kuffa approximation.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _lateral Lateral velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear, double _lateral,
    double _angular);

  /// \brief Current timestamp.
  public: std::chrono::steady_clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: gz::math::Angle heading;

  /// \brief Current linear velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current lateral velocity in meter/second.
  public: double lateralVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: gz::math::Angle angularVel;

  /// \brief Left wheel radius in meters.
  public: double leftWheelRadius{0.0};

  /// \brief Right wheel radius in meters.
  public: double rightWheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{1.0};

  /// \brief Wheel base in meters.
  public: double wheelBase{1.0};

  /// \brief Previous frontleft wheel position/state in radians.
  public: double frontLeftWheelOldPos{0.0};

  /// \brief Previous frontright wheel position/state in radians.
  public: double frontRightWheelOldPos{0.0};

  /// \brief Previous backleft wheel position/state in radians.
  public: double backLeftWheelOldPos{0.0};

  /// \brief Previous backright wheel position/state in radians.
  public: double backRightWheelOldPos{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: gz::math::RollingMean linearMean;

  /// \brief Rolling mean accumulators for the lateral velocity
  public: gz::math::RollingMean lateralMean;

  /// \brief Rolling mean accumulators for the angular velocity
  public: gz::math::RollingMean angularMean;

  /// \brief Initialized flag.
  public: bool initialized{false};

  /// \brief Current yaw orientation in Radians.
  public: double yaw{0.0};
};
}

using namespace ros_gz_omni_gazebo;

//////////////////////////////////////////////////
OmniMath::OmniMath(size_t _windowSize)
  : dataPtr(gz::utils::MakeImpl<Implementation>(_windowSize))
{
}

//////////////////////////////////////////////////
void OmniMath::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->dataPtr->linearMean.Clear();
  this->dataPtr->lateralMean.Clear();
  this->dataPtr->angularMean.Clear();
  this->dataPtr->x = 0.0;
  this->dataPtr->y = 0.0;
  this->dataPtr->yaw = 0.0;
  this->dataPtr->heading = 0.0;
  this->dataPtr->linearVel = 0.0;
  this->dataPtr->lateralVel = 0.0;
  this->dataPtr->angularVel = 0.0;
  this->dataPtr->frontLeftWheelOldPos = 0.0;
  this->dataPtr->frontRightWheelOldPos = 0.0;
  this->dataPtr->backLeftWheelOldPos = 0.0;
  this->dataPtr->backRightWheelOldPos = 0.0;

  this->dataPtr->lastUpdateTime = _time;
  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
bool OmniMath::Initialized() const
{
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
bool OmniMath::Update(const gz::math::Angle &_frontLeftPos,
  const gz::math::Angle &_frontRightPos, const gz::math::Angle &_backLeftPos,
  const gz::math::Angle &_backRightPos, const std::chrono::steady_clock::time_point &_time)
{
  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->dataPtr->lastUpdateTime;

  // Get current wheel joint positions:
  const double frontLeftWheelCurPos =
    *_frontLeftPos * this->dataPtr->leftWheelRadius;
  const double frontRightWheelCurPos =
    *_frontRightPos * this->dataPtr->rightWheelRadius;
  const double backLeftWheelCurPos =
    *_backLeftPos * this->dataPtr->leftWheelRadius;
  const double backRightWheelCurPos =
    *_backRightPos * this->dataPtr->rightWheelRadius;

  // Estimate velocity of wheels using old and current position:
  const double frontLeftWheelEstVel = frontLeftWheelCurPos -
                                 this->dataPtr->frontLeftWheelOldPos;

  const double frontRightWheelEstVel = frontRightWheelCurPos -
                                  this->dataPtr->frontRightWheelOldPos;

  const double backLeftWheelEstVel = backLeftWheelCurPos -
                                 this->dataPtr->backLeftWheelOldPos;

  const double backRightWheelEstVel = backRightWheelCurPos -
                                  this->dataPtr->backRightWheelOldPos;

  // Update old position with current
  this->dataPtr->frontLeftWheelOldPos = frontLeftWheelCurPos;
  this->dataPtr->frontRightWheelOldPos = frontRightWheelCurPos;
  this->dataPtr->backLeftWheelOldPos = backLeftWheelCurPos;
  this->dataPtr->backRightWheelOldPos = backRightWheelCurPos;

  // constant used in computing target velocities
  // TODO(danilogsch): support different wheel radius
  // const double angularConst =
    // (1/(4*(0.5*(this->dataPtr->wheelSeparation + this->dataPtr->wheelBase))));

  const double wheelDistance = std::sqrt(
    std::pow(this->dataPtr->wheelSeparation / 2.0, 2) + 
    std::pow(this->dataPtr->wheelBase / 2.0, 2)
  );
  const double angularConst =
    (1.0 / (4.0 * wheelDistance));

  // gzdbg << "Angular Const: " << angularConst << std::endl;
  
  // Compute linear and angular diff
  const double linear = (frontLeftWheelEstVel + frontRightWheelEstVel
    + backLeftWheelEstVel + backRightWheelEstVel) * 0.25;
  const double lateral = (-frontLeftWheelEstVel + frontRightWheelEstVel
    + backLeftWheelEstVel - backRightWheelEstVel) * 0.25;
  const double angular = (-frontLeftWheelEstVel + frontRightWheelEstVel
    + backLeftWheelEstVel - backRightWheelEstVel) * angularConst;

  this->dataPtr->IntegrateExact(linear, lateral, angular);

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (gz::math::equal(0.0, dt.count()))
    return false;

  this->dataPtr->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->dataPtr->linearMean.Push(linear / dt.count());
  this->dataPtr->lateralMean.Push(lateral / dt.count());
  this->dataPtr->angularMean.Push(angular / dt.count());

  this->dataPtr->linearVel = this->dataPtr->linearMean.Mean();
  this->dataPtr->lateralVel = this->dataPtr->lateralMean.Mean();
  this->dataPtr->angularVel = this->dataPtr->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void OmniMath::SetWheelParams(double _wheelSeparation,
  double _wheelBase, double _leftWheelRadius, double _rightWheelRadius)
{
  this->dataPtr->wheelSeparation = _wheelSeparation;
  this->dataPtr->wheelBase = _wheelBase;
  this->dataPtr->leftWheelRadius = _leftWheelRadius;
  this->dataPtr->rightWheelRadius = _rightWheelRadius;
}

//////////////////////////////////////////////////
void OmniMath::SetVelocityRollingWindowSize(size_t _size)
{
  this->dataPtr->linearMean.SetWindowSize(_size);
  this->dataPtr->lateralMean.SetWindowSize(_size);
  this->dataPtr->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const gz::math::Angle &OmniMath::Heading() const
{
  return this->dataPtr->heading;
}

//////////////////////////////////////////////////
double OmniMath::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
double OmniMath::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
double OmniMath::LinearVelocity() const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
double OmniMath::LateralVelocity() const
{
  return this->dataPtr->lateralVel;
}

//////////////////////////////////////////////////
const gz::math::Angle &OmniMath::AngularVelocity() const
{
  return this->dataPtr->angularVel;
}

//////////////////////////////////////////////////
double OmniMath::WheelSeparation() const
{
  return this->dataPtr->wheelSeparation;
}

//////////////////////////////////////////////////
double OmniMath::WheelBase() const
{
  return this->dataPtr->wheelBase;
}

//////////////////////////////////////////////////
double OmniMath::LeftWheelRadius() const
{
  return this->dataPtr->leftWheelRadius;
}

//////////////////////////////////////////////////
double OmniMath::RightWheelRadius() const
{
  return this->dataPtr->rightWheelRadius;
}

//////////////////////////////////////////////////
void OmniMath::Implementation::IntegrateRungeKutta2(
    double _linear, double _lateral, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += (_linear * std::cos(direction)) - (_lateral * std::sin(direction));
  this->y += (_linear * std::sin(direction)) + (_lateral * std::cos(direction));
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OmniMath::Implementation::IntegrateExact(double _linear,
  double _lateral, double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _lateral, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    
    const double ratio = _linear / _angular;
    const double ratio2 = _lateral / _angular;

    this->heading += _angular;
    
    this->x += (ratio * (std::sin(*this->heading) - std::sin(headingOld))) + (ratio2 * (std::cos(*this->heading) - std::cos(headingOld)));
    this->y += (-ratio * (std::cos(*this->heading) - std::cos(headingOld))) + (ratio2 * (std::sin(*this->heading) - std::sin(headingOld)));
  }
}