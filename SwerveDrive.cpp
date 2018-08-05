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

#include <cmath>

#include "SwerveDrive.h"
#include "SmartDashboard/SendableBuilder.h"

#ifndef LOCAL_TEST
#define DBG
#define DBGST(a,...)
#endif

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
  DBG;
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

  DBGST("north %f east %f yaw %f gyro %f", north, east, yaw, gyro);
  // Compensate for gyro angle. Positive rotation is counter-clockwise
  RotateVector(north, east, gyro);

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

  /* Scale wheel speeds */
  Normalize(wheelSpeeds);

//done Insert logic here ---------------------

  /* Set angles first */
  m_fl_steer_motor.Set(wheelAngles[FL]);
  m_fr_steer_motor.Set(wheelAngles[RL]);
  m_rl_steer_motor.Set(wheelAngles[FR]);
  m_rr_steer_motor.Set(wheelAngles[RR]);

// Verify that wheels are accurately positioned

// if every wheel is within tolerance {
  m_fl_drive_motor.Set(wheelSpeeds[FL]);
  m_fr_drive_motor.Set(wheelSpeeds[RL]);
  m_rl_drive_motor.Set(wheelSpeeds[FR]);
  m_rr_drive_motor.Set(wheelSpeeds[RR]);

  m_safetyHelper.Feed();
}

void SwerveDrive::StopMotor() {
  DBG;
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
  DBG;
  desc << "SwerveDrive";
}

void SwerveDrive::InitSendable(SendableBuilder& builder) {

  DBG;
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
}

// Borrowed from RobotDrive
void SwerveDrive::RotateVector(double& x, double& y, double angle) {
  DBGST("IN  x %f y %f angle %f", x, y, angle);
  double r = radians(angle);
  double cosA = std::cos(r);
  double sinA = std::sin(r);
  double xOut = x * cosA - y * sinA;
  double yOut = x * sinA + y * cosA;
  x = xOut;
  y = yOut;
  DBGST("OUT x %f y %f", x, y);
}

