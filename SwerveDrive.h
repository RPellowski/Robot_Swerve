/*----------------------------------------------------------------------------
 * Swerve Drive
 * FRC Team 1740
 *----------------------------------------------------------------------------*/

#pragma once

//#include <memory>

#include <wpi/raw_ostream.h>

#include "Drive/RobotDriveBase.h"

class SpeedController;

/**
 * Just as with MecanumDrive use the NED axis convention (North-East-Down)
 * http://www.nuclearprojects.com/ins/images/axis_big.png.
 *
 * The positive X axis points ahead, the positive Y axis points to the right,
 * and the positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 */

/**
 * TBD
 * Add wheel locations relative to frame
 * Add rotation motors
 * Add encoders for rotation
 * Add encoders for speed
 * Add default and settable PID values for rotation
 * Add default and settable PID values for speed (maybe)
 * Add Ackermann mode
 * Add reverse kinematics for calculation of location
 */

class SwerveDrive : public RobotDriveBase {
 public:
  SwerveDrive(SpeedController& left_front_motor,
              SpeedController& left_rear_motor,
              SpeedController& right_front_motor,
              SpeedController& right_rear_motor);

  ~SwerveDrive() override = default;

  SwerveDrive(const SwerveDrive&) = delete;
  SwerveDrive& operator=(const SwerveDrive&) = delete;

  void DriveCartesian(double north,
                      double east,
                      double yaw,
                      double gyro_angle = 0.0);

  void StopMotor() override;
  void GetDescription(wpi::raw_ostream& desc) const override;

  void InitSendable(SendableBuilder& builder) override;

 private:
  SpeedController& m_left_front_motor;
  SpeedController& m_left_rear_motor;
  SpeedController& m_right_front_motor;
  SpeedController& m_right_rear_motor;

  bool reported = false;
};

