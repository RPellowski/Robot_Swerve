/*----------------------------------------------------------------------------
 * Swerve Drive
 * FRC Team 1740
 *----------------------------------------------------------------------------*/

#pragma once

//#include <memory>

//#include <wpi/raw_ostream.h>

#include "Drive/RobotDriveBase.h"
#include "SpeedController.h"

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
 * Add encoders for rotation
 * Add encoders for speed (for kinematics)
 * Add default and settable PID values for rotation
 * Add default and settable PID values for speed (maybe)
 * Add Ackermann mode
 * Add reverse kinematics for calculation of location
 * Add (non-linear) scaling on inputs
 * Add scaling on outputs
 */

class SwerveDrive : public RobotDriveBase {
 public:
  SwerveDrive(SpeedController& fl_drive_motor,
              SpeedController& rl_drive_motor,
              SpeedController& fr_drive_motor,
              SpeedController& rr_drive_motor,
              SpeedController& fl_steer_motor,
              SpeedController& rl_steer_motor,
              SpeedController& fr_steer_motor,
              SpeedController& rr_steer_motor,
              double base_width,
              double base_length);

  ~SwerveDrive() override = default;

  // disable copy and assignment operators
  SwerveDrive(const SwerveDrive&) = delete;
  SwerveDrive& operator=(const SwerveDrive&) = delete;

  /**
   * DriveCartesian method for Swerve platform.
   *
   * @param north  The robot's velocity along the forward X axis [-1.0..1.0].
   * @param east   The robot's velocity along the right Y axis [-1.0..1.0].
   * @param yaw    The robot's rotation rate around the Z axis in degrees/sec.
   *               From above, positive is clockwise.
   * @param gyro   The current angle reading from the gyro in degrees around
   *               the Z axis. Use this to implement field-oriented controls.
   *               From above, positive is clockwise and 0 is north.
   */
  void DriveCartesian(double north,
                      double east,
                      double yaw,
                      double gyro = 0.0);

  void StopMotor() override;
  void GetDescription(wpi::raw_ostream& desc) const override;

  void InitSendable(SendableBuilder& builder) override;

  // Borrowed from RobotDrive
  void RotateVector(double& x, double& y, double angle);

 private:
  SpeedController& m_fl_drive_motor;
  SpeedController& m_rl_drive_motor;
  SpeedController& m_fr_drive_motor;
  SpeedController& m_rr_drive_motor;
  SpeedController& m_fl_steer_motor;
  SpeedController& m_rl_steer_motor;
  SpeedController& m_fr_steer_motor;
  SpeedController& m_rr_steer_motor;
  double m_base_width;
  double m_base_length;
};

