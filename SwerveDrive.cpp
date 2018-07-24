/*----------------------------------------------------------------------------
 * SwerveDrive
 * FRC Team 1740
 * Not exactly clear what this means, but it appears that we are forced to manage
 * PID ourselves:
 * https://www.chiefdelphi.com/forums/showthread.php?p=1726937
 *  Note that if you use the WPI-compatible wrappers in a WPI drivetrain object,
 *  you are restricted to open-loop control."
 *
 *  SpeedController interface does not have a method for changing control mode
 *  Write your own Talon wrapper implementing SpeedController that allows you
 *   to use closed-loop control
 *
 * Questions:
 *  Can we subclass RobotDriveBase/RobotDrive(deprecated)?
 *  What motors do/do not need PID and how to manage?
 *    inside vs outside class
 *    velocity vs distance
 *
 *----------------------------------------------------------------------------*/

#include "SwerveDrive.h"

//#include <algorithm>
#include <cmath>

//#include <HAL/HAL.h>

#include "Drive/Vector2d.h"
#include "SmartDashboard/SendableBuilder.h"
#include "SpeedController.h"

using namespace frc;

constexpr double kPi = 3.14159265358979323846;
#define FL 0
#define RL 1
#define FR 2
#define RR 3
#define radians(d) (d / 180.0 * kPi)
#define degrees(r) (r * 180.0 / kPi)

/**
 * Conceptual representative of a Swerve wheel fixture
 *
class Wheel {
}
 */

SwerveDrive::SwerveDrive(SpeedController& fl_drive_motor,
                         SpeedController& rl_drive_motor,
                         SpeedController& fr_drive_motor,
                         SpeedController& rr_drive_motor,
                         SpeedController& fl_steer_motor,
                         SpeedController& rl_steer_motor,
                         SpeedController& fr_steer_motor,
                         SpeedController& rr_steer_motor,
                         double base_width,
                         double base_length)
    : m_fl_drive_motor(fl_drive_motor),
      m_rl_drive_motor(rl_drive_motor),
      m_fr_drive_motor(fr_drive_motor),
      m_rr_drive_motor(rr_drive_motor),
      m_fl_steer_motor(fl_steer_motor),
      m_rl_steer_motor(rl_steer_motor),
      m_fr_steer_motor(fr_steer_motor),
      m_rr_steer_motor(rr_steer_motor),
      m_base_width(base_width),
      m_base_length(base_length) {
  AddChild(&m_fl_drive_motor);
  AddChild(&m_rl_drive_motor);
  AddChild(&m_fr_drive_motor);
  AddChild(&m_rr_drive_motor);
  AddChild(&m_fl_steer_motor);
  AddChild(&m_rl_steer_motor);
  AddChild(&m_fr_steer_motor);
  AddChild(&m_rr_steer_motor);
  static int instances = 0;
  ++instances;
  SetName("SwerveDrive", instances);
}

void SwerveDrive::DriveCartesian(double north,
                                 double east,
                                 double yaw,
                                 double gyro) {
#if 0
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 4,
               HALUsageReporting::kRobotDrive_MecanumCartesian);
    reported = true;
  }
#endif

  // Compensate for gyro angle. Positive rotation is counter-clockwise
  Vector2d input{north, east};
  input.Rotate(gyro);

//Insert logic here ---------------------

  double wheelSpeeds[4];
  wheelSpeeds[FL] = 0.0;
  wheelSpeeds[RL] = 0.0;
  wheelSpeeds[FR] = 0.0;
  wheelSpeeds[RR] = 0.0;

  double wheelAngles[4];
  wheelAngles[FL] = 0.0;
  wheelAngles[RL] = 0.0;
  wheelAngles[FR] = 0.0;
  wheelAngles[RR] = 0.0;

  /* Normalize(wheelSpeeds);
void RobotDriveBase::Normalize(wpi::MutableArrayRef<double> wheelSpeeds) {
  double maxMagnitude = std::abs(wheelSpeeds[0]);
  for (size_t i = 1; i < wheelSpeeds.size(); i++) {
    double temp = std::abs(wheelSpeeds[i]);
    if (maxMagnitude < temp) {
      maxMagnitude = temp;
    }
  }
  if (maxMagnitude > 1.0) {
    for (size_t i = 0; i < wheelSpeeds.size(); i++) {
      wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
    }
  }
}
 */
/* Scale wheel speeds */

//done Insert logic here ---------------------

  /* Set angles first */
  m_fl_steer_motor.Set(wheelAngles[FL]);
  m_fr_steer_motor.Set(wheelAngles[RL]);
  m_rl_steer_motor.Set(wheelAngles[FR]);
  m_rr_steer_motor.Set(wheelAngles[RR]);

  m_fl_drive_motor.Set(wheelSpeeds[FL]);
  m_fr_drive_motor.Set(wheelSpeeds[RL]);
  m_rl_drive_motor.Set(wheelSpeeds[FR]);
  m_rr_drive_motor.Set(wheelSpeeds[RR]);

  m_safetyHelper.Feed();
}

void SwerveDrive::StopMotor() {
  m_fl_drive_motor.StopMotor();
  m_fr_drive_motor.StopMotor();
  m_rl_drive_motor.StopMotor();
  m_rr_drive_motor.StopMotor();

  m_fl_steer_motor.StopMotor();
  m_fr_steer_motor.StopMotor();
  m_rl_steer_motor.StopMotor();
  m_rr_steer_motor.StopMotor();

  m_safetyHelper.Feed();
}

void SwerveDrive::GetDescription(wpi::raw_ostream& desc) const {
  desc << "SwerveDrive";
}

void SwerveDrive::InitSendable(SendableBuilder& builder) {
#ifndef LOCAL_TEST
  builder.SetSmartDashboardType("SwerveDrive");
  builder.AddDoubleProperty("Front Left Motor Speed",
                            [=]() { return m_fl_drive_motor.Get(); },
                            [=](double value) { m_fl_drive_motor.Set(value); });
  builder.AddDoubleProperty("Front Right Motor Speed",
                            [=]() { return m_fr_drive_motor.Get(); },
                            [=](double value) { m_fr_drive_motor.Set(value); });
  builder.AddDoubleProperty("Rear Left Motor Speed",
                            [=]() { return m_rl_drive_motor.Get(); },
                            [=](double value) { m_rl_drive_motor.Set(value); });
  builder.AddDoubleProperty("Rear Right Motor Speed",
                            [=]() { return m_rr_drive_motor.Get(); },
                            [=](double value) { m_rr_drive_motor.Set(value); });
  builder.AddDoubleProperty("Front Left Motor Angle",
                            [=]() { return m_fl_steer_motor.Get(); },
                            [=](double value) { m_fl_steer_motor.Set(value); });
  builder.AddDoubleProperty("Front Right Motor Angle",
                            [=]() { return m_fr_steer_motor.Get(); },
                            [=](double value) { m_fr_steer_motor.Set(value); });
  builder.AddDoubleProperty("Rear Left Motor Angle",
                            [=]() { return m_rl_steer_motor.Get(); },
                            [=](double value) { m_rl_steer_motor.Set(value); });
  builder.AddDoubleProperty("Rear Right Motor Angle",
                            [=]() { return m_rr_steer_motor.Get(); },
                            [=](double value) { m_rr_steer_motor.Set(value); });
#endif // LOCAL_TEST
}

