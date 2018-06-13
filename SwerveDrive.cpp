/*----------------------------------------------------------------------------
 * SwerveDrive
 * FRC Team 1740
 *----------------------------------------------------------------------------*/

#include "SwerveDrive.h"

//#include <algorithm>
//#include <cmath>

#include <HAL/HAL.h>

#include "Drive/Vector2d.h"
#include "SmartDashboard/SendableBuilder.h"
#include "SpeedController.h"

using namespace frc;

constexpr double kPi = 3.14159265358979323846;

SwerveDrive::SwerveDrive(SpeedController& left_front_motor,
                         SpeedController& left_rear_motor,
                         SpeedController& right_front_motor,
                         SpeedController& right_rear_motor)
    : m_left_front_motor(left_front_motor),
      m_left_rear_motor(left_rear_motor),
      m_right_front_motor(right_front_motor),
      m_right_rear_motor(right_rear_motor) {
  AddChild(&m_left_front_motor);
  AddChild(&m_left_rear_motor);
  AddChild(&m_right_front_motor);
  AddChild(&m_right_rear_motor);
  static int instances = 0;
  ++instances;
  SetName("SwerveDrive", instances);
}

void SwerveDrive::DriveCartesian(double ySpeed,
                                 double xSpeed,
                                 double zRotation,
                                 double gyroAngle) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
               HALUsageReporting::kRobotDrive_SwerveCartesian);
    reported = true;
  }

//Insert logic here ---------------------

//done Insert logic here ---------------------

  // Compensate for gyro angle.
  Vector2d input{ySpeed, xSpeed};
  input.Rotate(-gyroAngle);

  double wheelSpeeds[4];
  wheelSpeeds[kFrontLeft]  =
  wheelSpeeds[kFrontRight] =
  wheelSpeeds[kRearLeft]   =
  wheelSpeeds[kRearRight]  =

  Normalize(wheelSpeeds);

  m_left_front_motor.Set(wheelSpeeds[kFrontLeft] * m_maxOutput);
  m_right_front_motor.Set(wheelSpeeds[kFrontRight] * m_maxOutput);
  m_left_rear_motor.Set(wheelSpeeds[kRearLeft] * m_maxOutput);
  m_right_rear_motor.Set(wheelSpeeds[kRearRight] * m_maxOutput);

  m_safetyHelper.Feed();
}

void SwerveDrive::StopMotor() {
  m_left_front_motor.StopMotor();
  m_right_front_motor.StopMotor();
  m_left_rear_motor.StopMotor();
  m_right_rear_motor.StopMotor();
  m_safetyHelper.Feed();
}

void SwerveDrive::GetDescription(wpi::raw_ostream& desc) const {
  desc << "SwerveDrive";
}

void SwerveDrive::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("SwerveDrive");
  builder.AddDoubleProperty("Left Front Motor Speed",
                            [=]() { return m_left_front_motor.Get(); },
                            [=](double value) { m_left_front_motor.Set(value); });
  builder.AddDoubleProperty("Right Front Motor Speed",
                            [=]() { return m_right_front_motor.Get(); },
                            [=](double value) { m_right_front_motor.Set(value); });
  builder.AddDoubleProperty("Left Rear Motor Speed",
                            [=]() { return m_left_rear_motor.Get(); },
                            [=](double value) { m_left_rear_motor.Set(value); });
  builder.AddDoubleProperty("Right Rear Motor Speed",
                            [=]() { return m_right_rear_motor.Get(); },
                            [=](double value) { m_right_rear_motor.Set(value); });
}

