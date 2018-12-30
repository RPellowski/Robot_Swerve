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

class Wheel;
constexpr int kWheels = 4;
// Use same ordering convention as MecanumDrive
enum { FL = 0, RL, FR, RR };

class SwerveDrive : public RobotDriveBase {
 public:
  SwerveDrive();
#if 0
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
#endif
  ~SwerveDrive() override;

  // Disable copy and assignment operators
  SwerveDrive(const SwerveDrive&) = delete;
  SwerveDrive& operator=(const SwerveDrive&) = delete;

  /**
   * DriveCartesian method for Swerve platform.
   *
   * @param north  The robot's velocity along the forward X axis [-1.0..1.0].
   * @param east   The robot's velocity along the right Y axis [-1.0..1.0].
   * @param yaw    The robot's rotation rate around the Z axis in units/sec.
   *               From above, positive is clockwise.
   * @param gyro   The current angle reading from the gyro in degrees around
   *               the Z axis. Use this to implement field-oriented controls.
   *               From above, positive is clockwise and 0 is north.
   */
  void DriveCartesian(double north,
                      double east,
                      double yaw,
                      double gyro = 0.0);
  /**
   * SetMotors - common method to apply calculated settings to the motors
   */
  void SetMotors();

  /**
   * DriveAckermann method for Swerve platform.
   *
   * @param north  The robot's velocity along the forward X axis [-1.0..1.0].
   * @param east   The robot's velocity along the right Y axis [-1.0..1.0].
   */
  void DriveAckermann(double north,
                      double east);

  /**
   * DriveTank method for Swerve platform.
   *
   * This mode is unconventional- convert a polar input into right and left
   * speeds.
   * Essentially a drop-in replacement but will have a different feel and
   * resulting behavior.
   *
   * @param north  The robot's velocity along the forward X axis [-1.0..1.0].
   * @param east   The robot's velocity along the right Y axis [-1.0..1.0].
   */
  void DriveTank(double north,
                 double east);

  void StopMotor() override;

  void GetDescription(wpi::raw_ostream& desc) const override;
  void InitSendable(SendableBuilder& builder) override;

 private:
  SpeedController* m_drive[kWheels];
  SpeedController* m_steer[kWheels];
  PIDSource* m_anglePidIn[kWheels];
  PIDOutput* m_anglePidOut[kWheels];
  PIDController* m_anglePid[kWheels];
  PIDSource* m_drivePidIn[kWheels];
  PIDOutput* m_drivePidOut[kWheels];
  PIDController* m_drivePid[kWheels];
  Encoder* m_angleEnc[kWheels];
  Encoder* m_driveEnc[kWheels];
  double m_angleP;
  double m_angleI;
  double m_angleD;
  double m_driveP;
  double m_driveI;
  double m_driveD;
  double m_base_width;
  double m_base_length;
  Wheel *m_wheel[kWheels];

  double GetAngle(int index);
  void SetAngle(int index, double angle);
  void NormalizeSpeeds();
  static void RotateVector(double& x, double& y, double angle);
};

