/* --------------------------------------------------------------------------
 * SwerveDrive
 * FRC Team 1740
 * Notes
 *   Manage PID in this module:
 *     https://www.chiefdelphi.com/forums/showthread.php?p=1726937
 *     Note that if you use the WPI-compatible wrappers in a WPI drivetrain
 *     object, you are restricted to open-loop control.
 *     SpeedController interface does not have a method for changing control
 *     mode. Write your own Talon wrapper implementing SpeedController that
 *     allows you to use closed-loop control.
 *
 *   Use of PID for motors:
 *     PID for steer
 *     PID for velocity
 *     possibly PID for distance in autonomous mode
 *
 *   The objects involved:
 *     Wheel - manages only the mathematical representation of an assembly
 *       Steer Motor, Drive Motor, geometry
 *       Calculates angles and speeds given input vectors on three axes
 *     SwerveDrive - like MecanumDrive, derived from RobotDriveBase object
 *       Methods to implement
 *         DriveCartesian, DriveAckermann, DriveTank
 *         StopMotor, GetDescription, InitSendable
 *       Methods called
 *         AddChild, SetName, Set, Feed
 *         SetPosition, SetExpiration, SetSafetyEnabled
 *       Additional responsibilities
 *         Create Talons, Encoders, PID, Wheel objects
 *     SwerveSubsystem - derived from Subsystem object
 *       Methods to implement
 *         InitDefaultCommand, Go, Stop
 *       Methods called
 *         SwerveDrive methods: DriveCartesian, DriveAckermann
 *
 *   Where to create and maintain gyro?
 *     Probably the utility module
 *     Command?
 *
 * References:
 *   Java version: http://team484.org/programming/notes/swerve-drive/
 *
 * ------------------------------------------------------------------------ */

/*
 * TBD
 * (done) Add encoders for rotation (steering)
 * (done) Add encoders for speed/distance (for kinematics)
 * (done) Add default PID values for rotation
 * Add settable PID values for rotation
 * (done) Add compensation for rotation near discontinuity (0/360)
 * (done) Add default PID values for drive
 * Add settable PID values for drive
 * Add reverse kinematics for calculation of location
 * Add (non-linear) scaling on speed inputs
 * Add (non-linear) scaling on rotation inputs
 * (done) Add local scaling on speed outputs
 * (done) Add local scaling on rotation outputs
 * (done) Add global scaling on speed outputs
 * (done) Add global scaling on rotation outputs
 * (done) Add encoder inversion and type
 * (done) Enable motor safety? where?
 * Reset previous angle with drive = 0
 * Use positional PID for steering
 * Use velocity PID for drive
 * Override PIDController::Calculate
 * Use RTC in Calculate
 * Send PID data to workstation
 * Reverse Ackermann mode
 * Tank drive mode
 * Initialize methods
 *   ResetEncoders
 *   SetLinearPower?
 * Gradual enablement of PID features during build
 *
 * Test mode
 *  Single wheel direction
 *  Single wheel speed
 *  All wheels same direction
 *  All wheels same speed
 * Autonomous
 *  Drive distance and/or rotation
 *  Drive around point with rotation
 *  Drive around point with no rotation
 *
 */

#include <cmath>

#include "SwerveDrive.h"
#include "SmartDashboard/SendableBuilder.h"

#ifndef LOCAL_TEST
#undef DBG
#undef DBGf
#undef DBGf2
#undef DBGf3
#undef DBGf4
#undef DBGST
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

/* ======================================================================== *
                                  Wheel
 * ======================================================================== */

/*
 * Wheel
 * Conceptual representative of a Swerve wheel fixture.
 *   - Has no knowledge of underlying hardware
 *   - Provides for a logical wheel assembly of two motors (drive, steer)
 *   - Remembers previous values
 *   - Enables calculations to adjust settings based on inputs and history
 *   - Supports two modes: Swerve, Ackermann
 */
class Wheel {
 public:
  Wheel(double north, double east, double period = 0.05);
  ~Wheel();
  static double AngleModulus(double a);
  static double AngularDistance(double speed_prev, double prev,
                                double speed_next, double next);
  void NormalizeRotation();

  void ApplyTranslationAndRotation(double north, double east,
                                   double omega = 0);
  static void CalculateAckermannCG(double north, double east, double cgNorth,
                                   double& corDistance, double& omega);
  void ApplyAckermann(double north, double corDistance, double omega);

  double Speed();
  void Speed(double speed);
  double Angle();
  void Angle(double angle);
  double SteerOutputScale();
  void SteerOutputScale(double steer_output_scale);
  double DriveOutputScale();
  void DriveOutputScale(double drive_output_scale);
 private:
  double m_north;
  double m_east;
  double m_period;
  double m_speed;
  double m_angle;
  double m_speed_prev;
  double m_angle_prev;
  double m_steer_output_scale;
  double m_drive_output_scale;
};

/*
 * Constructor for the wheel assembly
 * Inputs
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
    m_angle_prev(0.),
    m_steer_output_scale(1.),
    m_drive_output_scale(1.) {
  DBGST("north %f east %f", m_north, m_east);
};

Wheel::~Wheel() { DBG; };

/*
 * AngleModulus
 *   Helper function - does not modify Wheel
 *
 * Given any angle, return equivalent angle in the range of (-180,180]
 */
double Wheel::AngleModulus(double a) {
  double ret = a;
  while (ret <= 180.) { ret += 360.; }
  while (ret  > 180.) { ret -= 360.; }
  return ret;
}

/*
 * AngularDistance
 *   Helper function - does not modify Wheel
 *
 * Given two angles and speeds, return delta angle to be applied to move the
 *   wheel from the previous angle setting to the next
 * Only the speed signs are used (not magnitude) to enable comparison of
 *   angles having the same sign
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
 * NormalizeRotation
 *   Modify Wheel - m_speed and m_angle
 *
 * Adjust new angle so it differs from the previous angle by < 90 degrees
 *   First, given previous and next, calculate the delta angle to be applied
 *   Second, set a new angle that is within 90 degrees
 *   Last, set new speed sign (which may be negative for reverse direction)
 */
void Wheel::NormalizeRotation() {
  double delta;

  // If only one sign is negative, perform adjustment so we can compare
  if (m_speed_prev * m_speed < 0.) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }

  // Always keep new angle within 90 degrees of previous angle
  // For larger deltas, the motor is reversed and a closer angle is selected
  delta = AngularDistance(m_speed_prev, m_angle_prev, m_speed, m_angle);
  if (std::abs(delta) > 90) {
    m_speed = 0. - m_speed;
    m_angle = AngleModulus(m_angle + 180.);
  }
};

/*
 * ApplyTranslationAndRotation
 *   Modify Wheel - m_speed_prev, m_angle_prev, m_speed, m_angle
 *
 * Swerve drive operation
 *   Apply new object values from three axes of input
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *   omega - clockwise rotation rate setting (yaw)
 *
 * Note: Calculation of the wheel velocity and angle is determined by
 *   - the forward and right translation velocities
 *   - the rotation rate omega
 *   - the periodic sampling rate m_period
 *
 * Outputs
 *   m_speed_prev - previous m_speed
 *   m_angle_prev - previous m_angle
 *   m_speed - new calculation
 *   m_angle - new calculation
 */
void Wheel::ApplyTranslationAndRotation(double north, double east,
                                        double omega) {
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
 * CalculateAckermannCG
 *   Helper function - does not modify Wheel
 *
 * Given an input vector and the distance between rear axle and robot
 *   Center of Gravity, return a distance and rotation rate
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
 *   magnitude - rotation vector length at the Center of Gravity
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
void Wheel::CalculateAckermannCG(double north, double east, double cgNorth,
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

  // Calculate corDistance between Center of Rear Axle and Center of Rotation
  cgDistance = cgNorth / std::sin(radians(steerAngle));
  corDistance = std::sqrt(cgDistance * cgDistance - cgNorth * cgNorth);
  if (steerAngle < 0) { corDistance = -corDistance; }

  // Calculate angular velocity around center of rotation (omega)
  magnitude = std::sqrt(east * east + north * north);
  if (north < 0) { magnitude = -magnitude; }
  omega = magnitude / cgDistance;

  DBGf3(north, east, cgNorth);
  DBGf4(corDistance, cgDistance, magnitude, omega);
}

/*
 * ApplyAckermann
 *   Modify Wheel - m_speed_prev, m_angle_prev, m_speed, m_angle
 *
 * Ackermann drive operation
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

  // Now calculate speed and angle
  m_speed = wheelDistance * std::abs(omega);
  // Could be backing up
  if (north < 0) { m_speed = -m_speed; }

  m_angle = degrees(std::asin(dY / wheelDistance));
  // Negative Center of Rotation is left hand turn
  if (corDistance < 0) { m_angle = -m_angle; }

  DBGf2(m_speed, m_angle);
};

/* Getter for speed */
double Wheel::Speed() {
  DBGf(m_speed);
  return m_speed;
};

/* Setter for speed */
void Wheel::Speed(double speed) {
  DBGf2(m_speed, speed);
  m_speed = speed;
};

/* Getter for angle */
double Wheel::Angle() {
  DBGf(m_angle);
  return m_angle;
};

/* Setter for angle */
void Wheel::Angle(double angle) {
  DBGf2(m_angle, angle);
  m_angle = angle;
};

/* Getter for steer_output_scale */
double Wheel::SteerOutputScale() {
  DBGf(m_steer_output_scale);
  return m_steer_output_scale;
};

/* Setter for steer_output_scale */
void Wheel::SteerOutputScale(double steer_output_scale) {
  DBGf2(m_steer_output_scale, steer_output_scale);
  m_steer_output_scale = steer_output_scale;
};

/* Getter for drive_output_scale */
double Wheel::DriveOutputScale() {
  DBGf(m_drive_output_scale);
  return m_drive_output_scale;
};

/* Setter for drive_output_scale */
void Wheel::DriveOutputScale(double drive_output_scale) {
  DBGf2(m_drive_output_scale, drive_output_scale);
  m_drive_output_scale = drive_output_scale;
};

/* ======================================================================== *
                             Swerve Drive
 * ======================================================================== */

// TBD: move constants to an include
constexpr int FL_DRIVE_MOTOR_ID = 1;
constexpr int RL_DRIVE_MOTOR_ID = 2;
constexpr int FR_DRIVE_MOTOR_ID = 3;
constexpr int RR_DRIVE_MOTOR_ID = 4;

constexpr int FL_STEER_MOTOR_ID = 5;
constexpr int RL_STEER_MOTOR_ID = 6;
constexpr int FR_STEER_MOTOR_ID = 7;
constexpr int RR_STEER_MOTOR_ID = 8;

// Temporary scaling until non-linear mapping developed
#define MapDriveIn(_d) (_d)
#define MapAngleIn(_d) (_d)
#define MapDriveOut(_d) (_d)
#define MapAngleOut(_d) (_d)

// Individual motor scaling is for fine-tuning, transmission matching or
// motor inversion
constexpr double FL_DRIVE_MOTOR_SCALE = 1.0;
constexpr double RL_DRIVE_MOTOR_SCALE = 1.0;
constexpr double FR_DRIVE_MOTOR_SCALE = 1.0;
constexpr double RR_DRIVE_MOTOR_SCALE = 1.0;

constexpr double FL_STEER_MOTOR_SCALE = 1.0;
constexpr double RL_STEER_MOTOR_SCALE = 1.0;
constexpr double FR_STEER_MOTOR_SCALE = 1.0;
constexpr double RR_STEER_MOTOR_SCALE = 1.0;

// The following, for example, could be set at 0.5 for half speed for demos
// or negative for inverting all motors
constexpr double DRIVE_MOTORS_SCALE = 1.0;
// The following would always be 1.0 (or -1.0 for inverting all motors)
// Any other tuning could be done with PID
constexpr double STEER_MOTORS_SCALE = 1.0;

constexpr int FL_DRIVE_ENCODER_CHAN_A = 21;
constexpr int RL_DRIVE_ENCODER_CHAN_A = 22;
constexpr int FR_DRIVE_ENCODER_CHAN_A = 23;
constexpr int RR_DRIVE_ENCODER_CHAN_A = 24;

constexpr int FL_DRIVE_ENCODER_CHAN_B = 25;
constexpr int RL_DRIVE_ENCODER_CHAN_B = 26;
constexpr int FR_DRIVE_ENCODER_CHAN_B = 27;
constexpr int RR_DRIVE_ENCODER_CHAN_B = 28;

constexpr int FL_STEER_ENCODER_CHAN_A = 11;
constexpr int RL_STEER_ENCODER_CHAN_A = 12;
constexpr int FR_STEER_ENCODER_CHAN_A = 13;
constexpr int RR_STEER_ENCODER_CHAN_A = 14;

constexpr int FL_STEER_ENCODER_CHAN_B = 15;
constexpr int RL_STEER_ENCODER_CHAN_B = 16;
constexpr int FR_STEER_ENCODER_CHAN_B = 17;
constexpr int RR_STEER_ENCODER_CHAN_B = 18;

constexpr bool FL_STEER_ENCODER_REVERSED = false;
constexpr bool RL_STEER_ENCODER_REVERSED = false;
constexpr bool FR_STEER_ENCODER_REVERSED = false;
constexpr bool RR_STEER_ENCODER_REVERSED = false;

constexpr bool FL_DRIVE_ENCODER_REVERSED = false;
constexpr bool RL_DRIVE_ENCODER_REVERSED = false;
constexpr bool FR_DRIVE_ENCODER_REVERSED = false;
constexpr bool RR_DRIVE_ENCODER_REVERSED = false;

constexpr CounterBase::EncodingType \
  STEER_ENCODER_COMMON_TYPE = CounterBase::EncodingType::k4X;
constexpr CounterBase::EncodingType \
  FL_STEER_ENCODER_TYPE = STEER_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  RL_STEER_ENCODER_TYPE = STEER_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  FR_STEER_ENCODER_TYPE = STEER_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  RR_STEER_ENCODER_TYPE = STEER_ENCODER_COMMON_TYPE;

constexpr CounterBase::EncodingType \
  DRIVE_ENCODER_COMMON_TYPE = CounterBase::EncodingType::k4X;
constexpr CounterBase::EncodingType
  FL_DRIVE_ENCODER_TYPE = DRIVE_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  RL_DRIVE_ENCODER_TYPE = DRIVE_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  FR_DRIVE_ENCODER_TYPE = DRIVE_ENCODER_COMMON_TYPE;
constexpr CounterBase::EncodingType \
  RR_DRIVE_ENCODER_TYPE = DRIVE_ENCODER_COMMON_TYPE;

constexpr double BASE_WIDTH = 24.5;
constexpr double BASE_LENGTH = 36.5;

constexpr double kAngleP = 1.;
constexpr double kAngleI = 0.;
constexpr double kAngleD = 0.;

constexpr double kDriveP = 1.;
constexpr double kDriveI = 0.;
constexpr double kDriveD = 0.;

//constexpr double kAngleTolerance = 0.25 / 360.0;
constexpr double kAngleDistancePerPulse = 360.0 / 1000.0;
//constexpr int kAngleSamplesToAverage = 127;

//constexpr double kDriveTolerance = 0.2;
constexpr double kDriveDistancePerPulse = 1.;
//constexpr int kDriveSamplesToAverage = 127;

SwerveDrive::SwerveDrive() {
  DBG;
  // Create motors
  m_drive[FL] = new WPI_TalonSRX(FL_DRIVE_MOTOR_ID);
  m_drive[RL] = new WPI_TalonSRX(RL_DRIVE_MOTOR_ID);
  m_drive[FR] = new WPI_TalonSRX(FR_DRIVE_MOTOR_ID);
  m_drive[RR] = new WPI_TalonSRX(RR_DRIVE_MOTOR_ID);
  m_steer[FL] = new WPI_TalonSRX(FL_STEER_MOTOR_ID);
  m_steer[RL] = new WPI_TalonSRX(RL_STEER_MOTOR_ID);
  m_steer[FR] = new WPI_TalonSRX(FR_STEER_MOTOR_ID);
  m_steer[RR] = new WPI_TalonSRX(RR_STEER_MOTOR_ID);

  // Create encoders
  m_angleEnc[FL] = new Encoder(FL_STEER_ENCODER_CHAN_A,
                               FL_STEER_ENCODER_CHAN_B,
                               FL_STEER_ENCODER_REVERSED,
                               FL_STEER_ENCODER_TYPE);
  m_angleEnc[RL] = new Encoder(RL_STEER_ENCODER_CHAN_A,
                               RL_STEER_ENCODER_CHAN_B,
                               RL_STEER_ENCODER_REVERSED,
                               RL_STEER_ENCODER_TYPE);
  m_angleEnc[FR] = new Encoder(FR_STEER_ENCODER_CHAN_A,
                               FR_STEER_ENCODER_CHAN_B,
                               FR_STEER_ENCODER_REVERSED,
                               FR_STEER_ENCODER_TYPE);
  m_angleEnc[RR] = new Encoder(RR_STEER_ENCODER_CHAN_A,
                               RR_STEER_ENCODER_CHAN_B,
                               RR_STEER_ENCODER_REVERSED,
                               RR_STEER_ENCODER_TYPE);

  m_driveEnc[FL] = new Encoder(FL_DRIVE_ENCODER_CHAN_A,
                               FL_DRIVE_ENCODER_CHAN_B,
                               FL_DRIVE_ENCODER_REVERSED,
                               FL_DRIVE_ENCODER_TYPE);
  m_driveEnc[RL] = new Encoder(RL_DRIVE_ENCODER_CHAN_A,
                               RL_DRIVE_ENCODER_CHAN_B,
                               RL_DRIVE_ENCODER_REVERSED,
                               RL_DRIVE_ENCODER_TYPE);
  m_driveEnc[FR] = new Encoder(FR_DRIVE_ENCODER_CHAN_A,
                               FR_DRIVE_ENCODER_CHAN_B,
                               FR_DRIVE_ENCODER_REVERSED,
                               FR_DRIVE_ENCODER_TYPE);
  m_driveEnc[RR] = new Encoder(RR_DRIVE_ENCODER_CHAN_A,
                               RR_DRIVE_ENCODER_CHAN_B,
                               RR_DRIVE_ENCODER_REVERSED,
                               RR_DRIVE_ENCODER_TYPE);

  // Set dimensions
  m_base_width = BASE_WIDTH;
  m_base_length = BASE_LENGTH;

  // Set common PID parameters
  m_angleP = kAngleP;
  m_angleI = kAngleI;
  m_angleD = kAngleD;

  m_driveP = kDriveP;
  m_driveI = kDriveI;
  m_driveD = kDriveD;

  // Assume center of robot is geometric center of wheels
  // And geometric center is center of gravity for robot
  double l = m_base_length / 2.;
  double w = m_base_width / 2.;

  // The following normalizes rotation
  double p = 1.0 / std::sqrt(l * l + w * w);
  m_wheel[FL] = new Wheel( l,-w, p);
  m_wheel[RL] = new Wheel(-l,-w, p);
  m_wheel[FR] = new Wheel( l, w, p);
  m_wheel[RR] = new Wheel(-l, w, p);

  m_wheel[FL]->SteerOutputScale(FL_STEER_MOTOR_SCALE);
  m_wheel[RL]->SteerOutputScale(RL_STEER_MOTOR_SCALE);
  m_wheel[FR]->SteerOutputScale(FR_STEER_MOTOR_SCALE);
  m_wheel[RR]->SteerOutputScale(RR_STEER_MOTOR_SCALE);

  m_wheel[FL]->DriveOutputScale(FL_DRIVE_MOTOR_SCALE);
  m_wheel[RL]->DriveOutputScale(RL_DRIVE_MOTOR_SCALE);
  m_wheel[FR]->DriveOutputScale(FR_DRIVE_MOTOR_SCALE);
  m_wheel[RR]->DriveOutputScale(RR_DRIVE_MOTOR_SCALE);


    // Inner objects to support PIDController (needs PIDSource and PIDOutput)
    class AnglePIDSource : public PIDSource {
     public:
      SwerveDrive* m_swerve;
      int m_index;
      AnglePIDSource(SwerveDrive* swerve, int index) : PIDSource() {
        DBGv(index);
        m_swerve = swerve;
        m_index = index;
      }
      double PIDGet() {
        DBG;
        return m_swerve->GetAngle(m_index);
      }
      void SetPIDSourceType(PIDSourceType pidSource) {
        DBG;
        // No-op - Prevent change from intended setting of kDisplacement or kRate
      }
    };

    class AnglePIDOutput : public PIDOutput {
     public:
      SwerveDrive* m_swerve;
      int m_index;
      AnglePIDOutput(SwerveDrive* swerve, int index) : PIDOutput() {
        DBGv(index);
        m_swerve = swerve;
        m_index = index;
      }
      void PIDWrite(double d) {
        DBGf(d);
        m_swerve->SetAngle(m_index, d);
      }
    };

    class DrivePIDSource : public PIDSource {
     public:
      SwerveDrive* m_swerve;
      int m_index;
      double m_prev;
      DrivePIDSource(SwerveDrive* swerve, int index) : PIDSource() {
        DBGv(index);
        m_swerve = swerve;
        m_index = index;
        m_pidSource = PIDSourceType::kRate;
        m_prev = 0;
      }
      double PIDGet() {
        double next = m_swerve->GetDrive(m_index);
        double rate = next - m_prev;
        DBGf3(m_prev, next, rate);
        m_prev = next;
        return rate;
      }
      void SetPIDSourceType(PIDSourceType pidSource) {
        DBG;
        // No-op - Prevent change from intended setting of kDisplacement or kRate
      }
    };

    class DrivePIDOutput : public PIDOutput {
     public:
      SwerveDrive* m_swerve;
      int m_index;
      DrivePIDOutput(SwerveDrive* swerve, int index) : PIDOutput() {
        DBGv(index);
        m_swerve = swerve;
        m_index = index;
      }
      void PIDWrite(double d) {
        DBGf(d);
        m_swerve->SetDrive(m_index, d);
      }
    };

  // Create PID controllers
  for (size_t i = 0; i < kWheels; i++) {
    m_anglePidIn[i] = new AnglePIDSource(this, i);
    m_anglePidOut[i] = new AnglePIDOutput(this, i);
    m_anglePid[i] = new PIDController(m_angleP, m_angleI, m_angleD,
                                      m_anglePidIn[i], m_anglePidOut[i]);
    m_anglePid[i]->SetContinuous();
    m_anglePid[i]->SetInputRange(-180, 180);
    m_anglePid[i]->SetOutputRange(-1, 1);
    m_anglePid[i]->SetSetpoint(0);
    m_anglePid[i]->Enable();
    // m_anglePid[i]->SetPercentTolerance(kAngleTolerance);
    m_angleEnc[i]->SetDistancePerPulse(kAngleDistancePerPulse);
    // m_angleEnc[i]->SetSamplesToAverage(kAngleSamplesToAverage);

    m_drivePidIn[i] = new DrivePIDSource(this, i);
    m_drivePidOut[i] = new DrivePIDOutput(this, i);
    m_drivePid[i] = new PIDController(m_driveP, m_driveI, m_driveD,
                                      m_drivePidIn[i], m_drivePidOut[i]);
    m_drivePid[i]->SetInputRange(-1, 1);
    m_drivePid[i]->SetOutputRange(-1, 1);
    m_drivePid[i]->SetSetpoint(0);
    m_drivePid[i]->Enable();
    m_driveEnc[i]->SetDistancePerPulse(kDriveDistancePerPulse);
    // m_driveEnc[i]->SetSamplesToAverage(kDriveSamplesToAverage);
  };

  for (size_t i = 0; i < kWheels; i++) {
    AddChild(&m_drive[i]);
    AddChild(&m_steer[i]);
  }

  static int instances = 0;
  ++instances;
  SetName("SwerveDrive", instances);
}

SwerveDrive::~SwerveDrive() {
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    // Math calculation objects
    delete m_wheel[i];

    // Angle Controller objects
    m_anglePid[i]->Disable();
    delete m_anglePid[i];
    delete m_anglePidIn[i];
    delete m_anglePidOut[i];

    // Drive Controller objects
    m_drivePid[i]->Disable();
    delete m_drivePid[i];
    delete m_drivePidIn[i];
    delete m_drivePidOut[i];

    // Motor objects
    delete m_steer[i];
    delete m_drive[i];

    // Encoder objects
    delete m_angleEnc[i];
    delete m_driveEnc[i];
  }
}

/*
 * DriveCartesian
 *   Set steer and drive motors
 *
 * Swerve drive operation
 *   Given set of inputs, calculate the appropriate motor settings and apply
 *     them to the individual motors
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *   omega - clockwise rotation rate setting (yaw)
 *   gyro - current rotation measurement relative to the field (zero at init)
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
  north = MapDriveIn(north);
  east = MapDriveIn(east);
  gyro = MapAngleIn(gyro);

  // Compensate for gyro angle. Positive rotation is counter-clockwise
  RotateVector(north, east, gyro);

  // Perform calculations to get steer and drive settings
  for (size_t i = 0; i < kWheels; i++) {
    DBGz("---");
    m_wheel[i]->ApplyTranslationAndRotation(north, east, yaw);
  }

  SetMotors();
}

void SwerveDrive::SetMotors() {
  DBG;

  // Scale wheel speeds so that a magnitude of 1. is max
  NormalizeSpeeds();

#if 0
  /*
   * Take extra care to account for real-world conditions
   *   - prevent unnecessary steering movement if not driving
   *   - minimize friction wear on wheels not within steering tolerance
   *
   * These are the steps taken
   *   1. Get final drive speed settings, after all applied scaling transforms
   *   2. If any drive speeds are zero, reset corresponding angle to the
   *      previous one
   *   3. Set steer wheels to their next setpoints
   *   4. Measure current steering angles
   *   5. If any steer wheel is not within its setpoint tolerance, defer all
   *      drive motor settings to the next time this method is called
   *   6. With all conditions satisfied, set the drive motor speeds
   */
  constexpr double deadband = 0.1;
  double driveOut[kWheels];
  for (size_t i = 0; i < kWheels; i++) {
    driveOut[i] = MapDriveOut(m_wheel[i]->Speed() * DRIVE_MOTORS_SCALE);
    if ((std::abs(driveOut[i]) <= deadband) {
      driveOut[i] = 0;
    }
  }

  // Set steering motor angles first
  for (size_t i = 0; i < kWheels; i++) {
    if (driveOut[i] == 0) {
      m_wheel[i]->ResetAngle();
    }
    m_pid[i]->SetSetpoint(m_wheel[i]->Angle());
  }

  // Verify that wheels are accurately positioned
  constexpr double steer_tolerance = 5;
  bool within_tolerance = true;
  for (size_t i = 0; i < kWheels; i++) {
    // TBD: account for discontinuity
    double currentAngle = GetAngle(i);
    m_wheel[i]->Angle();
    if ((std::abs() > steer_tolerance) {
      within_tolerance = false;
    }
  }

  // If every wheel is within tolerance, then ...
  // Set drive motor values
  if (within_tolerance) {
    for (size_t i = 0; i < kWheels; i++) {
      m_drive[i]->Set(driveOut[i]);
    }
  }
#else
  // Set steering motor angles first
  for (size_t i = 0; i < kWheels; i++) {
#ifdef VELOCITY_PID
    if (i == 0) {DBGon = true;}
    DBGz("-------------------------------");
    m_pid[i]->SetSetpoint(m_wheel[i]->Angle());
    m_pid[i]->Calculate();
    // Estimate degrees in 50 ms
    int steerTravel = int(m_steer[i]->Get() * 2.0 / 360. * 4000);
DBGv(steerTravel);
    m_angle[i]->SetRaw(m_angle[i]->GetRaw()+steerTravel);
    DBGon = false;
#else
    m_anglePid[i]->SetSetpoint(m_wheel[i]->Angle());
#endif
  }

  // Verify that wheels are accurately positioned
  // If every wheel is within tolerance, then ...
  // Set drive motor values
  for (size_t i = 0; i < kWheels; i++) {
    double d = MapDriveOut(m_wheel[i]->Speed() * DRIVE_MOTORS_SCALE);
    m_drive[i]->Set(d);
  }
#endif

  // Reset watchdog timer
  m_safetyHelper.Feed();
}

/*
 * DriveAckermann
 *   Set steer and drive motors to operate with front wheel steering
 *
 * Ackermann drive operation
 *   Rear wheels are locked at 0 degrees
 *   Given set of inputs, calculate
 *     steering for the front wheels
 *     speeds for all wheels
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *
 * Intermediates
 *   m_wheel[] - modify Wheel object representations with get and set logic
 *
 * Outputs
 *   m_steer[] - apply steering motor settings from calculations
 *   m_drive[] - apply drive motor settings from calculations
 *
 */
void SwerveDrive::DriveAckermann(double north,
                                 double east) {

  DBGf2(north, east);
  north = MapDriveIn(north);
  east = MapDriveIn(east);
  double distance;
  double omega;

  Wheel::CalculateAckermannCG(north, east, m_base_length / 2, distance, omega);
  // Perform calculations to get steer and drive settings
  for (size_t i = 0; i < kWheels; i++) {
    DBGz("---");
    m_wheel[i]->ApplyAckermann(north, distance, omega);
  }

  SetMotors();
}

/*
 * DriveTank
 *   Set steer and drive motors to operate in tank mode
 *
 * Tank drive operation
 *   All wheels are locked at 0 degrees
 *   Given set of inputs, calculate
 *     speeds for all wheels
 *
 * Inputs
 *   north - forward velocity setting
 *   east - right velocity setting
 *
 * Intermediates
 *   m_wheel[] - modify Wheel object representations with get and set logic
 *
 * Outputs
 *   m_steer[] - apply steering motor settings from calculations
 *   m_drive[] - apply drive motor settings from calculations
 *
 */
void SwerveDrive::DriveTank(double north,
                            double east) {

  DBGf2(north, east);
  north = MapDriveIn(north);
  east = MapDriveIn(east);

  SetMotors();
}

/*
 * NormalizeSpeeds
 *
 * Wheel speed scaling
 *   Ensure all speeds are within the maximum allowed range (-1..1)
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
  double temp[kWheels];
  DBG;
  for (size_t i = 0; i < kWheels; i++) {
    temp[i] = std::abs(m_wheel[i]->Speed());
    if (norm < temp[i]) {
      norm = temp[i];
    }
  }
  if (norm > 1.0) {
    for (size_t i = 0; i < kWheels; i++) {
      m_wheel[i]->Speed(temp[i] / norm);
    }
  }
}

/*
 * StopMotor
 *   Set all drive and steer motor amplitudes to zero
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
 * GetAngle
 *   Called by a PIDSource object to get the current steer motor angle as
 *     measured by its encoder - feeds into the PIDController
 *
 * Inputs
 *   index - which steer motor / encoder
 *
 * Outputs
 *   angle - current encoder value
 */
double SwerveDrive::GetAngle(int index) {
  double angle = 0.;
  if (m_angleEnc[index] != nullptr) {
    angle = m_angleEnc[index]->GetDistance();
  }
  DBGST("index %d angle" f1f, index, angle);
  return angle;
}

/*
 * SetAngle
 *   Called by a PIDOutput object to set a steer motor speed to achieve the
 *     desired angle objective - based on calculation of the PIDController
 *
 * Inputs
 *   index - which steer motor
 *   angle - motor speed to set
 */
void SwerveDrive::SetAngle(int index, double angle) {
  DBGST("index %d angle" f1f, index, angle);
  if (m_steer[index] != nullptr) {
    double d = MapAngleOut(angle * STEER_MOTORS_SCALE);
    m_steer[index]->Set(d);
  }
}

/*
 * GetDrive
 *   Called by a PIDSource object to get the current drive motor as
 *     measured by its encoder - feeds into the PIDController
 *
 * Inputs
 *   index - which drive motor / encoder
 *
 * Outputs
 *   drive - current encoder value
 */
double SwerveDrive::GetDrive(int index) {
  double drive = 0.;
  if (m_driveEnc[index] != nullptr) {
    drive = m_driveEnc[index]->GetDistance();
  }
  DBGST("index %d drive" f1f, index, drive);
  return drive;
}

/*
 * SetDrive
 *   Called by a PIDOutput object to set a drive motor speed to achieve the
 *     desired velocity objective - based on calculation of the PIDController
 *
 * Inputs
 *   index - which drive motor
 *   drive - motor speed to set
 */
void SwerveDrive::SetDrive(int index, double drive) {
  DBGST("index %d drive" f1f, index, drive);
  if (m_drive[index] != nullptr) {
    double d = MapDriveOut(drive * DRIVE_MOTORS_SCALE);
    m_drive[index]->Set(d);
  }
}

/*
 * RotateVector
 *   Helper function - does not modify SwerveDrive object
 *
 * Apply angle of rotation to (x, y) pair and return the pair, modified
 *
 * Inputs
 *   x and y - values to be modified
 *   angle - modifier in degrees
 *
 * Outputs
 *   x and y - values after applying angle modifier
 *
 * Based on RobotDrive version
 */
void SwerveDrive::RotateVector(double& x, double& y, double angle) {
  DBGST("IN  x %f y %f theta %.1f angle %.1f",
        x, y, degrees(std::atan2(y, x)), angle);
  double r = radians(angle);
  double cosA = std::cos(r);
  double sinA = std::sin(r);
  double xOut = x * cosA - y * sinA;
  double yOut = x * sinA + y * cosA;
  x = xOut;
  y = yOut;
  DBGST("OUT x %f y %f angle %.1f", x, y, degrees(std::atan2(y, x)));
}

