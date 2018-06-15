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
#define LF 0
#define LR 1
#define RF 2
#define RR 3
#define radians(d) (d / 180.0 * kPi)
#define degrees(r) (r * 180.0 / kPi)

/**
 * Conceptual representative of a Swerve wheel fixture
 *
 */
class Wheel {
}

SwerveDrive::SwerveDrive(SpeedController& lf_drive_motor,
                         SpeedController& lr_drive_motor,
                         SpeedController& rf_drive_motor,
                         SpeedController& rr_drive_motor)
SwerveDrive::SwerveDrive(SpeedController& lf_steer_motor,
                         SpeedController& lr_steer_motor,
                         SpeedController& rf_steer_motor,
                         SpeedController& rr_steer_motor)
    : m_lf_drive_motor(lf_drive_motor),
      m_lr_drive_motor(lr_drive_motor),
      m_rf_drive_motor(rf_drive_motor),
      m_rr_drive_motor(rr_drive_motor) {
    : m_lf_drive_motor(lf_steer_motor),
      m_lr_drive_motor(lr_steer_motor),
      m_rf_drive_motor(rf_steer_motor),
      m_rr_drive_motor(rr_steer_motor) {
  AddChild(&m_lf_drive_motor);
  AddChild(&m_lr_drive_motor);
  AddChild(&m_rf_drive_motor);
  AddChild(&m_rr_drive_motor);
  AddChild(&m_lf_steer_motor);
  AddChild(&m_lr_steer_motor);
  AddChild(&m_rf_steer_motor);
  AddChild(&m_rr_steer_motor);
  static int instances = 0;
  ++instances;
  SetName("SwerveDrive", instances);
}

void SwerveDrive::DriveCartesian(double north,
                                 double east,
                                 double yaw,
                                 double gyro) {
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
               HALUsageReporting::kRobotDrive_SwerveCartesian);
    reported = true;
  }

  // Compensate for gyro angle.
  Vector2d input{north, east};
  input.Rotate(gyro);

//Insert logic here ---------------------

  double wheelSpeeds[4];
  wheelSpeeds[LF] =
  wheelSpeeds[LR] =
  wheelSpeeds[RF] =
  wheelSpeeds[RR] =

//done Insert logic here ---------------------

  Normalize(wheelSpeeds);

  m_lf_drive_motor.Set(wheelSpeeds[LF] * m_maxOutput);
  m_rf_drive_motor.Set(wheelSpeeds[LR] * m_maxOutput);
  m_lr_drive_motor.Set(wheelSpeeds[RF] * m_maxOutput);
  m_rr_drive_motor.Set(wheelSpeeds[RR] * m_maxOutput);
  m_lf_steer_motor.Set(wheelSpeeds[LF] * m_maxOutput);
  m_rf_steer_motor.Set(wheelSpeeds[LR] * m_maxOutput);
  m_lr_steer_motor.Set(wheelSpeeds[RF] * m_maxOutput);
  m_rr_steer_motor.Set(wheelSpeeds[RR] * m_maxOutput);

  m_safetyHelper.Feed();
}

void SwerveDrive::StopMotor() {
  m_lf_drive_motor.StopMotor();
  m_rf_drive_motor.StopMotor();
  m_lr_drive_motor.StopMotor();
  m_rr_drive_motor.StopMotor();
  m_lf_steer_motor.StopMotor();
  m_rf_steer_motor.StopMotor();
  m_lr_steer_motor.StopMotor();
  m_rr_steer_motor.StopMotor();
  m_safetyHelper.Feed();
}

void SwerveDrive::GetDescription(wpi::raw_ostream& desc) const {
  desc << "SwerveDrive";
}

void SwerveDrive::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("SwerveDrive");
  builder.AddDoubleProperty("Left Front Motor Speed",
                            [=]() { return m_lf_drive_motor.Get(); },
                            [=](double value) { m_lf_drive_motor.Set(value); });
  builder.AddDoubleProperty("Right Front Motor Speed",
                            [=]() { return m_rf_drive_motor.Get(); },
                            [=](double value) { m_rf_drive_motor.Set(value); });
  builder.AddDoubleProperty("Left Rear Motor Speed",
                            [=]() { return m_lr_drive_motor.Get(); },
                            [=](double value) { m_lr_drive_motor.Set(value); });
  builder.AddDoubleProperty("Right Rear Motor Speed",
                            [=]() { return m_rr_drive_motor.Get(); },
                            [=](double value) { m_rr_drive_motor.Set(value); });
}

