/*----------------------------------------------------------------------------
 * Swerve Drive
 * FRC Team 1740
 *----------------------------------------------------------------------------*/

#pragma once

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
 * Add compensation for rotation near discontinuity (0/360)
 * Add compensation for speed inverted instead of rotation of 180
 * Add default and settable PID values for speed (maybe)
 * Add Ackermann mode
 * Add reverse kinematics for calculation of location
 * Add (non-linear) scaling on speed inputs
 * Add (non-linear) scaling on rotation inputs
 * Add scaling on speed outputs
 * Add scaling on rotation outputs
 * Enable motor safety? where?
 * Test mode
 *  Single wheel direction
 *  Single wheel speed
 *  All wheels same direction
 * Autonomous
 *  Drive around point with rotation
 *  Drive around point with no rotation
 *
 */

class Wheel;
constexpr int kWheels = 4;
enum { FL = 0, RL, RR, FR };

class SwerveDrive : public RobotDriveBase {
 public:
  SwerveDrive(int deviceNumbers[8],
              double base_width,
              double base_length);

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

  ~SwerveDrive() override;

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

  void NormalizeSpeeds();
  void StopMotor() override;
  void GetDescription(wpi::raw_ostream& desc) const override;

  void InitSendable(SendableBuilder& builder) override;

  // Borrowed from RobotDrive
  void RotateVector(double& x, double& y, double angle);

 private:
  SpeedController* m_drive[kWheels];
  SpeedController* m_steer[kWheels];
  double m_base_width;
  double m_base_length;
  Wheel *m_wheel[kWheels];
};

