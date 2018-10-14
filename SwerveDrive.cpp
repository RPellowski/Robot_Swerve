/*----------------------------------------------------------------------------
 * SwerveDrive
 * FRC Team 1740
 * Notes
 *   It appears that we need to manage PID ourselves:
 *     https://www.chiefdelphi.com/forums/showthread.php?p=1726937
 *     Note that if you use the WPI-compatible wrappers in a WPI drivetrain object,
 *     you are restricted to open-loop control."
 *     SpeedController interface does not have a method for changing control mode
 *     Write your own Talon wrapper implementing SpeedController that allows you
 *     to use closed-loop control
 *
 *   What motors do/do not need PID?
 *     where to use PID for velocity vs distance
 *   Where is the best location to manage motors?
 *     inside vs outside class
 *
 * References:
 *   Java version
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
 * Conceptual representative of a Swerve wheel fixture.
 *   Has no knowledge of underlying hardware
 *   Provides for a logical wheel assembly of two motors (drive, steer)
 *   Preserves history
 *   Enables calculations to adjust settings based on inputs and history
 */
class Wheel {
 public:
  Wheel(double north, double east, double period = 0.05);
  ~Wheel();
  static double AngleModulus(double a);
  static double AngularDistance(double speed_prev, double prev,
                                double speed_next, double next);
  void NormalizeRotation();
  void ApplyTranslationAndRotation(double north, double east, double omega = 0);
  static void CalculateAckermanCG(double north, double east, double cgNorth,
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

/*
 * Constructor for the wheel assembly
 *   north - distance of wheel from the center of gravity; forward is positive
 *   east - distance of wheel from the center of gravity; right is positive
 *   period - the time in seconds between calls for motor activity;
 *     used for calculating rates of angular motion
 */
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

/*
 * Helper function - does not modify Wheel
 *   Given any angle, return equivalent angle in the range of (-180,180]
 */
double Wheel::AngleModulus(double a) {
  double ret = a;
  while (ret <= 180.) { ret += 360.; }
  while (ret > 180.) { ret -= 360.; }
  return ret;
}

/*
 * Helper function - does not modify Wheel
 *
 *  Given two angles and speeds, retun delta angle to be applied to move the
 *    wheel from the previous angle setting to the next
 *  Only the speed signs are used (not magnitude) to enable comparison of
 *    angles having the same sign
 */
double Wheel::AngularDistance(double speed_prev, double prev,
                              double speed_next, double next) {
  // If only one sign is negative, perform adjustment so we can compare
  if (speed_prev * speed_next < 0.) {
    next = AngleModulus(next + 180.);
  }
  return AngleModulus(next - prev);
}

/*
 * Modify Wheel - m_speed and m_angle
 *
 * Adjust new angle so it differs from the previous angle by less than 90 degrees
 *   First, given previous and next, calculate the delta angle to be applied
 *   Second, set a new angle that is within 90 degrees
 *   Last, set new speed sign (which may be negative for reverse direction)
 */
void Wheel::NormalizeRotation() {
  double distance;

  // If only one sign is negative, perform adjustment so we can compare
  if (m_speed_prev * m_speed < 0.) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }

  // Always keep new angle within 90 degrees of previous angle
  // For larger deltas, the motor is reversed and a closer angle is selected
  distance = AngularDistance(m_speed_prev, m_angle_prev, m_speed, m_angle);
  if (std::abs(distance) > 90) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }
};

/*
 * Swerve drive operation
 *
 * Modify Wheel - m_speed_prev, m_angle_prev, m_speed, m_angle
 *
 * Apply new object values from three axes of input

 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *   omega - clockwise rotation rate setting (yaw)
 *
 * Note: Calculation of the wheel velocity and angle is determined by
 *   - the rotation rate omega
 *   - the periodic sampling rate m_period
 *
 * Outputs
 *   m_speed_prev - previous m_speed
 *   m_angle_prev - previous m_angle
 *   m_speed - new calculation
 *   m_angle - new calculation
 */
void Wheel::ApplyTranslationAndRotation(double north, double east, double omega) {
  double dX;
  double dY;
  double dOmegaX;
  double dOmegaY;
  DBGST("north %f east %f omega %.3f", north, east, omega);

  // Translation deltas
  dX = north;
  dY = east;

  // Rotation deltas - note that east affects X and north affects Y
  dOmegaX = omega * -m_east * m_period;
  dOmegaY = omega * m_north * m_period;

  // Net deltas
  dX += dOmegaX;
  dY += dOmegaY;

  // Now assign speed and angle
  m_speed_prev = m_speed;
  m_angle_prev = m_angle;
  m_speed = std::sqrt(dX * dX + dY * dY);
  m_angle = degrees(std::atan2(dY, dX));

  // TBD: If speed is zero, then use the previous angle

  // Adjust speed sign and rotation angle for range of rotation
  NormalizeRotation();
  DBGST("speed %f angle %.1f", m_speed, m_angle);
};

/*
 * Helper function - does not modify Wheel
 *
 * Given an input vector and the distance between rear axle and robot
 * Center of Gravity, return a distance and rotation rate
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *   cgNorth - distance between rear axle and CoG
 *
 * Intermediate
 *   steerAngle - input vector based on north and east
 *     range is minAngle to maxAngle (in any of the 4 quadrants)
 *   cgDistance - distance between robot CoG and CoR
 *     range is a finite value (positive for right, negative for left)
 *
 * Notes
 *   Minimum steerAngle must be non-zero to calculate a finite cgDistance
 *     The value was chosen to minimize error (couple of inches left or right
 *       over the field length)
 *   Maximum steerAngle was chosen to keep the steering wheels less than
 *     90 degrees given nominal robot dimensions
 *
 * Outputs
 *   corDistance - distance between Center of Rear Axle and Center of Rotation
 *   omega - angular velocity around the Center of Rotation
 *
 */
void Wheel::CalculateAckermanCG(double north, double east, double cgNorth,
                                double& corDistance, double& omega) {
  constexpr double maxAngle = 45.;
  constexpr double minAngle = 0.25;
  double steerAngle;
  double cgDistance;
  double magnitude;

  // Get steerAngle into a workable range
  steerAngle = degrees(std::atan2(east, std::abs(north)));
  if (steerAngle < -maxAngle) { steerAngle = -maxAngle; }
  if (steerAngle >  maxAngle) { steerAngle =  maxAngle; }
  if (std::abs(steerAngle) < minAngle) { steerAngle = minAngle; }

  // Calculate distance between center of rear axle and Center of Rotation
  // (corDistance)
  cgDistance = cgNorth / std::sin(radians(steerAngle));
  corDistance = std::sqrt(cgDistance * cgDistance - cgNorth * cgNorth);
  if (steerAngle < 0) { corDistance = -corDistance; }
  //corDistance = std::copysign(corDistance, steerAngle);

  // Calculate angular velocity around center of rotation (omega)
  magnitude = std::sqrt(east * east + north * north);
  if (north < 0) { magnitude = -magnitude; }
  //magnitude = std::copysign(magnitude, north);
  omega = magnitude / cgDistance;

  DBGf3(north, east, cgNorth);
  DBGf4(corDistance, cgDistance, magnitude, omega);
}

/*
 * Ackermann drive operation
 *
 * Modify Wheel - m_speed_prev, m_angle_prev, m_speed, m_angle
 *
 * Inputs
 *   north - velocity in the forward direction
 *   corDistance - distance between Center of Rear Axle and Center of Rotation
 *   omega - angular velocity around the Center of Rotation
 *
 * Intermediate
 *   wheelDistance - this wheel's distance to the Center of Rotation,
 *     calculated knowing corDistance and the wheel's position relative to
 *     the Center of Rear Axle (from m_north, m_east)
 *
 * Outputs
 *   m_speed_prev - previous m_speed
 *   m_angle_prev - previous m_angle
 *   m_speed - new calculation
 *   m_angle - new calculation
 *
 */
void Wheel::ApplyAckermann(double north, double corDistance, double omega) {
  DBGf3(north, corDistance, omega);
  double dX;
  double dY;
  double wheelDistance;

  dX = corDistance - m_east;
  dY = m_north + std::abs(m_north);
  wheelDistance = std::sqrt(dX * dX + dY * dY);

  m_speed_prev = m_speed;
  m_angle_prev = m_angle;

  // Now speed and angle
  m_speed = wheelDistance * std::abs(omega);
  // Could be backing up
  if (north < 0) { m_speed = -m_speed; }
  //m_speed = std::copysign(m_speed, north);

  m_angle = degrees(std::asin(dY / wheelDistance));
  // Negative Center of Rotation is left hand turn
  if (corDistance < 0) { m_angle = -m_angle; }
  //m_angle = std::copysign(m_angle, corDistance);

  DBGf2(m_speed, m_angle);
};

/*
 * Given a normaliation value, apply it to the current m_speed and save
 *
 * modify Wheel - m_speed
 *
 * Input
 *   norm - normalization value
 * Output
 *   m_speed - value after normalization
 */
double Wheel::NormalizeSpeed(double norm) {
  if (norm > 0.) { m_speed /= norm; }
  DBGf2(norm, m_speed);
  return m_speed;
};

/*
 * Getter for speed
 */
double Wheel::Speed() {
  DBGf(m_speed);
  return m_speed;
};

/*
 * Getter for angle
 */
double Wheel::Angle() {
  DBGf(m_angle);
  return m_angle;
};

/* ======================================================================== */
#if 0
double SwerveDrive::getEncoderAngle(std::string foo) {
  DBGz(foo.c_str());
  return 0.;
}
void SwerveDrive::rotFL_set(double d) {
  DBGf(d);
}
#endif
SwerveDrive::SwerveDrive(int deviceNumbers[8],
            double base_width,
            double base_length) {
  DBG;
#if 0
  double kP = 1.;
  double kI = 0.;
  double kD = 0.;

  WPI_TalonSRX *m1 = new WPI_TalonSRX(1);
  WPI_TalonSRX *m2 = new WPI_TalonSRX(2);
  WPI_TalonSRX *m3 = new WPI_TalonSRX(3);
  WPI_TalonSRX *m4 = new WPI_TalonSRX(4);
  WPI_TalonSRX *m5 = new WPI_TalonSRX(5);
  WPI_TalonSRX *m6 = new WPI_TalonSRX(6);
  WPI_TalonSRX *m7 = new WPI_TalonSRX(7);
  WPI_TalonSRX *m8 = new WPI_TalonSRX(8);

  pid[FL] = new PIDController(kP, kI, kD,
    new PIDSource() {
      public double pidGet() {
        DBG;
        return findEncAng("encFL.getDistance()");
      }
      public void setPIDSourceType(PIDSourceType pidSource) {
        DBG;
        // Disallow setting of SourceType
        // Default is PIDSourceType.kDisplacement
      }
    },
    new PIDOutput() {
      public void pidWrite(double d) {
        DBG;
        rotFL_set(d);
      }
    }
  );
  pidFL.setContinuous();
  pidFL.setInputRange(-180, 180);
  pidFL.setOutputRange(-1, 1);
  pidFL.setSetpoint(0);
  // this.controllerRotate.setPercentTolerance(0.07);
  pidFL.enable();
  //enc.setDistancePerPulse(0.875);
  //enc.setSamplesToAverage(127);
#endif
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
  // The following normalizes rotation
  double p = 1.0 / std::sqrt(l * l + w * w);
  m_wheel[FL] = new Wheel( l,-w, p);
  m_wheel[RL] = new Wheel(-l,-w, p);
  m_wheel[RR] = new Wheel(-l, w, p);
  m_wheel[FR] = new Wheel( l, w, p);

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

/*
 * Swerve drive operation
 *
 * Given set of inputs, calculate the appropriate motor settings and apply
 *  them to the individual motors
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *   omega - clockwise rotation rate setting (yaw)
 *   gyro - current rotation measurement relative to the field (zero at initialization)
 *
 * Intermediates
 *   m_wheel[] - modify Wheel object representations with get and set logic
 *
 * Outputs
 *   m_steer[] - apply steering motor settings from calculations
 *   m_drive[] - apply drive motor settings from calculations
 *
 */
void SwerveDrive::DriveCartesian(double north,
                                 double east,
                                 double yaw,
                                 double gyro) {

  DBGST("north %f east %f yaw %.1f gyro %.1f", north, east, yaw, gyro);
  // Compensate for gyro angle. Positive rotation is counter-clockwise
  RotateVector(north, east, gyro);

  // Perform calculations to get steer and drive settings
  for (size_t i = 0; i < kWheels; i++) {
    DBGz("---");
    m_wheel[i]->ApplyTranslationAndRotation(north, east, yaw);
  }

  // Scale wheel speeds so that a magnitude of 1. is max
  NormalizeSpeeds();

  // Set steering motor angles first
  for (size_t i = 0; i < kWheels; i++) {
    m_steer[i]->Set(m_wheel[i]->Angle());
  }

  // Verify that wheels are accurately positioned
  // If every wheel is within tolerance, then ...
  // Set drive motor values
  for (size_t i = 0; i < kWheels; i++) {
    m_drive[i]->Set(m_wheel[i]->Speed());
  }

  // Reset watchdog timer
  m_safetyHelper.Feed();
}

/*
 * Wheel speed scaling
 *
 * Ensure all speeds are within the maximum allowed range (-1..1)
 *
 * Intermediates
 *   norm - maximum magnitude of all m_wheel[] speeds
 *
 * Outputs
 *   m_wheel[] - Wheel objects after each m_speed has been normalized
 *
 */
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

/*
 * Set all drive and steer motors amplitudes to zero
 */
void SwerveDrive::StopMotor() {
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    m_drive[i]->StopMotor();
    // TBD - is this right? Position to zero or just set amplitude to 0?
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

/*
 * Helper function - does not modify SwerveDrive object
 *
 * Apply angle of rotation to (x, y) pair and return that modified pair
 *
 * Inputs
 *   x and y - values to be modified
 *   angle - modifier
 * Outputs
 *   x and y - values after applying angle modifier
 *
 * Based on RobotDrive version
 */
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

