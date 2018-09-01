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
 * References:
 * Java version
 *
 *----------------------------------------------------------------------------*/

#include <cmath>

#include "SwerveDrive.h"
#include "SmartDashboard/SendableBuilder.h"

#ifndef LOCAL_TEST
#define DBG
#define DBGf
#define DBGf2
#define DBGf3
#define DBGf4
#define DBGST(a,...)
#endif

using namespace frc;

constexpr double kPi = 3.14159265358979323846;
#define radians(d) (d / 180.0 * kPi)
#define degrees(r) (r * 180.0 / kPi)

/* ======================================================================== */

/**
 * Conceptual representative of a Swerve wheel fixture
 */
class Wheel {
 public:
  Wheel(double north, double east, double period = 0.05);
  ~Wheel();
  static double AngleModulus(double a);
  static double AngularDistance(double speed_prev, double prev, double speed_next, double next);
  void NormalizeRotation();
  void ApplyTranslationAndRotation(double north, double east, double omega = 0.);
  static void CalculateAckermanCG(double north, double east,
                                  double cgNorth, double cgEast,
                                  double& corDistance, double& omega);
  void ApplyAckermann(double north, double corDistance, double omega);
  double NormalizeSpeed(double norm);
  double Speed();
  double Angle();
 private:
  double m_north;
  double m_east;
  double m_period;
  double m_speed;
  double m_angle;
  double m_speed_prev;
  double m_angle_prev;
};

Wheel::Wheel(double north, double east, double period)
  : m_north(north),
    m_east(east),
    m_period(period),
    m_speed(0.),
    m_angle(0.),
    m_speed_prev(0.),
    m_angle_prev(0.) {
  DBGST("north %f east %f", m_north, m_east);
};

Wheel::~Wheel() { DBG; };

double Wheel::AngleModulus(double a) {
  // Put angle into range of (-180,180]
  double ret = a;
#if 1
  while (ret <= 180.) { ret += 360.; }
  while (ret > 180.) { ret -= 360.; }
#else
  double n = 360;
  while (ret < 0.) { ret += n; }
  while (ret >= n) { ret -= n; }
  if (ret > 180.) { ret -= 360.; }
#endif
  //DBGf2(a,ret);
  return ret;
}

double Wheel::AngularDistance(double speed_prev, double prev, double speed_next, double next) {
  // Calculate angle from previous to next
  //DBGf4(speed_prev, prev, speed_next, next);
  if (speed_prev * speed_next < 0.) {
    next = AngleModulus(next + 180.);
  }
  return AngleModulus(next - prev);
}

void Wheel::NormalizeRotation() {
  // First, make signs equal so that angles can be compared
  double distance;
  if (m_speed_prev * m_speed < 0.) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }
  // Always keep new angle within 90 degrees of previous angle
  // For larger deltas, the motor is reversed and a closer angle is selected
  distance = AngularDistance(m_speed_prev, m_angle_prev, m_speed, m_angle);
  //DBGf(distance);
  if (std::abs(distance) > 90) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }
  //DBGf4(m_speed_prev, m_angle_prev, m_speed, m_angle);
};

void Wheel::ApplyTranslationAndRotation(double north, double east, double omega) {
  double dX;
  double dY;
  double dOmegaX;
  double dOmegaY;
  DBGST("north %f east %f omega %.3f", north, east, omega);

  // Translation deltas
  dX = north;
  dY = east;
  //DBGf2(dX,dY);
  // Rotation deltas -
#if 0
  dOmegaX = omega * (0. - m_north) * m_period;
  dOmegaY = omega * m_east * m_period;
#else
  //   note that east affects X and north affects Y
  dOmegaX = omega * (0. - m_east) * m_period;
  dOmegaY = omega * m_north * m_period;
#endif
  //DBGf2(dOmegaX,dOmegaY);

  // Net deltas
  dX += dOmegaX;
  dY += dOmegaY;
  //DBGf2(dX,dY);

  // Now speed and angle
  m_speed_prev = m_speed;
  m_angle_prev = m_angle;
  m_speed = std::sqrt(dX * dX + dY * dY);
  m_angle = degrees(std::atan2(dY, dX));
  //DBGST("speed %f angle %.1f", m_speed, m_angle);

  // TBD: If speed is zero, then use the previous angle

  // Adjust speed sign and rotation angle for range of rotation
#ifndef xSKIP_ROTATION_NORM
  NormalizeRotation();
#endif
  DBGST("speed %f angle %.1f", m_speed, m_angle);
};

void Wheel::CalculateAckermanCG(double north, double east,
                                double cgNorth, double cgEast,
                                double& corDistance, double& omega) {
  constexpr double maxAngle = 45.;
  constexpr double minAngle = 0.25;
  double angle;
  double magnitude;
  double cgDistance;
  angle = degrees(std::atan2(east, std::abs(north)));
  if (angle < -maxAngle) { angle = -maxAngle; }
  if (angle >  maxAngle) { angle =  maxAngle; }
  if (std::abs(angle) < minAngle) { angle = minAngle; }
  magnitude = std::sqrt(east * east + north * north);
  if (north < 0) { magnitude = -magnitude; }
  cgDistance = cgNorth / std::sin(radians(angle));
  corDistance = std::sqrt(cgDistance * cgDistance - cgNorth * cgNorth);
  if (angle < 0) { corDistance = -corDistance; }
  omega = magnitude / cgDistance;
  DBGf4(north, east, cgNorth, cgEast);
  DBGf4(corDistance, cgDistance, magnitude, omega);
}
void Wheel::ApplyAckermann(double north, double corDistance, double omega) {
  DBGf3(north, corDistance, omega);
  double dX;
  double dY;
  double wheelDistance;

  // Angles with respect to the center of rotation
  dX = corDistance - m_east;
  dY = m_north + std::abs(m_north);
  DBGf2(dX,dY);
  wheelDistance = std::sqrt(dX * dX + dY * dY);

  // Now speed and angle
  m_speed_prev = m_speed;
  m_angle_prev = m_angle;

  m_speed = wheelDistance * std::abs(omega);
  if (north < 0) { m_speed = -m_speed; }
  m_angle = degrees(std::asin(dY / wheelDistance));
  if (corDistance < 0) { m_angle = -m_angle; }
  DBGf2(m_speed, m_angle);
};

double Wheel::NormalizeSpeed(double norm) {
  if (norm > 0.) { m_speed /= norm; }
  DBGf2(norm, m_speed);
  return m_speed;
};

double Wheel::Speed() {
  DBGf(m_speed);
  return m_speed;
};

double Wheel::Angle() {
  DBGf(m_angle);
  return m_angle;
};

/* ======================================================================== */

SwerveDrive::SwerveDrive(int deviceNumbers[8],
            double base_width,
            double base_length) {
  DBG;
/*
  WPI_TalonSRX *m1 = new WPI_TalonSRX(1);
  WPI_TalonSRX *m2 = new WPI_TalonSRX(2);
  WPI_TalonSRX *m3 = new WPI_TalonSRX(3);
  WPI_TalonSRX *m4 = new WPI_TalonSRX(4);
  WPI_TalonSRX *m5 = new WPI_TalonSRX(5);
  WPI_TalonSRX *m6 = new WPI_TalonSRX(6);
  WPI_TalonSRX *m7 = new WPI_TalonSRX(7);
  WPI_TalonSRX *m8 = new WPI_TalonSRX(8);

  pid[FL] = new PIDController(kP, kI, kD, new PIDSource() {
      public double pidGet() {
          return findEncAng(encFL.getDistance());
      }
      public void setPIDSourceType(PIDSourceType pidSource) {
      }
      public PIDSourceType getPIDSourceType() {
          return PIDSourceType.kDisplacement;
      }
    }, new PIDOutput() {
        public void pidWrite(double d) {
          rotFL.set(d);
        }
      }
    );
    pidFL.setContinuous();
    pidFL.setInputRange(-180, 180);
    pidFL.setOutputRange(-1, 1);
    pidFL.setSetpoint(0);
    pidFL.enable();
 */
};

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
    : m_drive{&fl_drive_motor,
              &rl_drive_motor,
              &rr_drive_motor,
              &fr_drive_motor},
      m_steer{&fl_steer_motor,
              &rl_steer_motor,
              &rr_steer_motor,
              &fr_steer_motor},
      m_base_width(base_width),
      m_base_length(base_length) {
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    AddChild(&m_drive[i]);
    AddChild(&m_steer[i]);
  }

  // Assume center of robot is geometric center of wheels
  // And geometric center is center of gravity for robot
  double l = base_length / 2.;
  double w = base_width / 2.;
  m_wheel[FL] = new Wheel( l,-w);
  m_wheel[RL] = new Wheel(-l,-w);
  m_wheel[RR] = new Wheel(-l, w);
  m_wheel[FR] = new Wheel( l, w);

  static int instances = 0;
  ++instances;
  SetName("SwerveDrive", instances);
}

SwerveDrive::~SwerveDrive() {
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    delete m_wheel[i];
  }
}

void SwerveDrive::DriveCartesian(double north,
                                 double east,
                                 double yaw,
                                 double gyro) {

  DBGST("north %f east %f yaw %.1f gyro %.1f", north, east, yaw, gyro);
  // Compensate for gyro angle. Positive rotation is counter-clockwise
  RotateVector(north, east, gyro);

  for (size_t i = 0; i < kWheels; i++) {
    DBGz("---");
    m_wheel[i]->ApplyTranslationAndRotation(north, east, yaw);
  }

  // Scale wheel speeds
  NormalizeSpeeds();

  // Set angles first
  for (size_t i = 0; i < kWheels; i++) {
    m_steer[i]->Set(m_wheel[i]->Angle());
  }

  // Verify that wheels are accurately positioned
  // If every wheel is within tolerance, then ...
  for (size_t i = 0; i < kWheels; i++) {
    m_drive[i]->Set(m_wheel[i]->Speed());
  }

  m_safetyHelper.Feed();
}

void SwerveDrive::NormalizeSpeeds() {
  double norm = 1.0;
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    double temp = std::abs(m_wheel[i]->Speed());
    if (norm < temp) {
      norm = temp;
    }
  }
  if (norm > 1.0) {
    for (size_t i = 0; i < kWheels; i++) {
      m_wheel[i]->NormalizeSpeed(norm);
    }
  }
}

void SwerveDrive::StopMotor() {
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    m_drive[i]->StopMotor();
    m_steer[i]->StopMotor();
  }
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
                            [=]() { return m_drive[FL]->Get(); },
                            [=](double value) { m_drive[FL]->Set(value); });
  builder.AddDoubleProperty("Front Right Motor Speed",
                            [=]() { return m_drive[FR]->Get(); },
                            [=](double value) { m_drive[FR]->Set(value); });
  builder.AddDoubleProperty("Rear Left Motor Speed",
                            [=]() { return m_drive[RL]->Get(); },
                            [=](double value) { m_drive[RL]->Set(value); });
  builder.AddDoubleProperty("Rear Right Motor Speed",
                            [=]() { return m_drive[RR]->Get(); },
                            [=](double value) { m_drive[RR]->Set(value); });
  builder.AddDoubleProperty("Front Left Motor Angle",
                            [=]() { return m_steer[FL]->Get(); },
                            [=](double value) { m_steer[FL]->Set(value); });
  builder.AddDoubleProperty("Front Right Motor Angle",
                            [=]() { return m_steer[FR]->Get(); },
                            [=](double value) { m_steer[FR]->Set(value); });
  builder.AddDoubleProperty("Rear Left Motor Angle",
                            [=]() { return m_steer[RL]->Get(); },
                            [=](double value) { m_steer[RL]->Set(value); });
  builder.AddDoubleProperty("Rear Right Motor Angle",
                            [=]() { return m_steer[RR]->Get(); },
                            [=](double value) { m_steer[RR]->Set(value); });
}

// Modified from RobotDrive version
void SwerveDrive::RotateVector(double& x, double& y, double angle) {
  DBGST("IN  x %f y %f (%.1f) angle %.1f", x, y, degrees(std::atan2(y, x)), angle);
  double r = radians(angle);
  double cosA = std::cos(r);
  double sinA = std::sin(r);
  double xOut = x * cosA - y * sinA;
  double yOut = x * sinA + y * cosA;
  x = xOut;
  y = yOut;
  DBGST("OUT x %f y %f (%.1f)", x, y, degrees(std::atan2(y, x)));
}

