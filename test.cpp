/*
 * Test wrapper for Swerve Drive object
 * Compile with
 *  g++ -std=c++11 -o a test.cpp
 */
#include <string>
#include <cmath>
#include <libgen.h>
#include <pthread.h>
#include "/home/rob/utils/dbg"

// -----------------------------------------------------------------
class MotorSafety {
 public:
  virtual void SetExpiration(double timeout) = 0;
  virtual double GetExpiration() const = 0;
  virtual bool IsAlive() const = 0;
  virtual void StopMotor() = 0;
  virtual void SetSafetyEnabled(bool enabled) = 0;
  virtual bool IsSafetyEnabled() const = 0;
  virtual void GetDescription() const = 0;
};
// -----------------------------------------------------------------
class SendableBuilder;

class Sendable {
 public:
  virtual ~Sendable() = default;
  virtual std::string GetName() const = 0;
  virtual void SetName(const std::string& name) = 0;
  void SetName(const std::string& subsystem, const std::string& name) {
    SetSubsystem(subsystem);
    SetName(name);
  }
  virtual std::string GetSubsystem() const = 0;
  virtual void SetSubsystem(const std::string& subsystem) = 0;
  virtual void InitSendable(SendableBuilder& builder) = 0;
};
// -----------------------------------------------------------------
class SendableBase : public Sendable {
 public:
  SendableBase(bool addLiveWindow = true);
  virtual ~SendableBase() override;
  void SetName(const std::string& name) override;
  std::string GetName() const override;
  void SetSubsystem(const std::string& subsystem) override;
  std::string GetSubsystem() const override;
  virtual void InitSendable(SendableBuilder& builder);
 private:
  std::string m_name;
  std::string m_subsystem = "Ungrouped";
};
SendableBase::SendableBase(bool addLiveWindow) { DBG; }
SendableBase::~SendableBase() { DBG; }
void SendableBase::SetName(const std::string& name) { m_name = name; DBG; };
std::string SendableBase::GetName() const { DBG; return m_name; }
void SendableBase::SetSubsystem(const std::string& subsystem) { m_subsystem = subsystem; DBG; };
std::string SendableBase::GetSubsystem() const { DBG; return m_subsystem; }
void SendableBase::InitSendable(SendableBuilder& builder) { DBG; };
// -----------------------------------------------------------------
class SendableBuilder {
public:
  SendableBuilder();
  virtual ~SendableBuilder() = default;
  void SetSmartDashboardType();
  void AddDoubleProperty();
};
SendableBuilder::SendableBuilder() { DBG; };
//SendableBuilder::~SendableBuilder() { DBG; };
void SendableBuilder::SetSmartDashboardType() { DBG; };
void SendableBuilder::AddDoubleProperty() { DBG; };
// -----------------------------------------------------------------
class PIDOutput {
 public:
  virtual void PIDWrite(double output) = 0;
};
// -----------------------------------------------------------------
class SpeedController : public PIDOutput {
 public:
  virtual ~SpeedController() = default;
  virtual void Set(double speed) = 0;
  virtual double Get() const = 0;
  virtual void SetInverted(bool isInverted) = 0;
  virtual bool GetInverted() const = 0;
  virtual void Disable() = 0;
  virtual void StopMotor() = 0;
};
// -----------------------------------------------------------------
class TalonSRX {
};
// -----------------------------------------------------------------
class WPI_TalonSRX : public virtual TalonSRX,
    public SpeedController,
    public SendableBase,
    public MotorSafety {
 public:
  WPI_TalonSRX(int deviceNumber);
  virtual ~WPI_TalonSRX();
  WPI_TalonSRX() = delete;
  WPI_TalonSRX(WPI_TalonSRX const&) = delete;
  WPI_TalonSRX& operator=(WPI_TalonSRX const&) = delete;
  virtual void Set(double speed);
    virtual void PIDWrite(double output);
  virtual double Get() const;
    virtual void Set(ControlMode mode, double value);
    virtual void Set(ControlMode mode, double demand0, double demand1);
  virtual void SetInverted(bool isInverted);
  virtual bool GetInverted() const;
  virtual void Disable();
  virtual void StopMotor();
    void SetExpiration(double timeout);
    double GetExpiration() const;
    bool IsAlive() const;
    bool IsSafetyEnabled() const;
    void SetSafetyEnabled(bool enabled);
    void GetDescription(llvm::raw_ostream& desc) const;
    virtual void InitSendable(frc::SendableBuilder& builder);
 private:
  double m_speed;
  bool m_inverted;
    double _speed = 0;
    std::string _desc;
    frc::MotorSafetyHelper _safetyHelper;

void WPI_TalonSRX::Set(double speed) { DBG; }
void WPI_TalonSRX::PIDWrite(double output) { DBG; }
double WPI_TalonSRX::Get() const { DBG; }
void WPI_TalonSRX::Set(ControlMode mode, double value) { DBG; }
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) { DBG; }
void WPI_TalonSRX::SetInverted(bool isInverted) { DBG; }
bool WPI_TalonSRX::GetInverted() const { DBG; }
void WPI_TalonSRX::Disable() { DBG; }
void WPI_TalonSRX::StopMotor() { DBG; }
void WPI_TalonSRX::SetExpiration(double timeout) { DBG; }
double WPI_TalonSRX::GetExpiration() const { DBG; }
bool WPI_TalonSRX::IsAlive() const { DBG; }
bool WPI_TalonSRX::IsSafetyEnabled() const { DBG; }
void WPI_TalonSRX::SetSafetyEnabled(bool enabled) { DBG; }
void WPI_TalonSRX::GetDescription(llvm::raw_ostream& desc) const { DBG; }
void WPI_TalonSRX::InitSendable(frc::SendableBuilder& builder) { DBG; }







};
void WPI_TalonSRX::Set(double speed) { m_speed = speed; DBG; };
double WPI_TalonSRX::Get() const { DBG; return m_speed; };
void WPI_TalonSRX::SetInverted(bool isInverted) { m_inverted = isInverted; DBG; };
bool WPI_TalonSRX::GetInverted() const { DBG; return m_inverted; };
void WPI_TalonSRX::Disable() { DBG; };
void WPI_TalonSRX::StopMotor() { DBG; };
// -----------------------------------------------------------------
class MotorSafetyHelper {
 public:
  MotorSafetyHelper();
  ~MotorSafetyHelper();
  void Feed();
};
MotorSafetyHelper::MotorSafetyHelper() { DBG; }
MotorSafetyHelper::~MotorSafetyHelper() { DBG; }
void MotorSafetyHelper::Feed() { DBG; }
// -----------------------------------------------------------------
class RobotDriveBase : public MotorSafety, public SendableBase {
 public:
  int instances;
  RobotDriveBase();
  ~RobotDriveBase() override = default;
  virtual void InitSendable(SendableBuilder& builder) = 0;
  virtual void StopMotor() = 0;
  virtual void SetName(const char *name, int count);
  virtual void SetExpiration(double timeout);
  virtual double GetExpiration() const;
  virtual bool IsAlive() const;
  virtual void SetSafetyEnabled(bool enabled);
  virtual bool IsSafetyEnabled() const;
  virtual void GetDescription() const;
  void AddChild(void *child);
  MotorSafetyHelper m_safetyHelper;
};
RobotDriveBase::RobotDriveBase() { DBG; };
//RobotDriveBase::~RobotDriveBase() { DBG; };
void RobotDriveBase::SetName(const char *name, int count) { DBG; } ;
void RobotDriveBase::AddChild(void *child) { DBG; };

void RobotDriveBase::SetExpiration(double timeout) { DBG; }
double RobotDriveBase::GetExpiration() const { DBG; return 0.0; }
bool RobotDriveBase::IsAlive() const { DBG; return false; }
void RobotDriveBase::SetSafetyEnabled(bool enabled) { DBG; }
bool RobotDriveBase::IsSafetyEnabled() const { DBG; return false; }
void RobotDriveBase::GetDescription() const { DBG; }
// -----------------------------------------------------------------
struct Vector2d {
  Vector2d() = default;
  Vector2d(double x, double y);
  void Rotate(double angle);
  double Dot(const Vector2d& vec) const;
  double Magnitude() const;
  double ScalarProject(const Vector2d& vec) const;
  double x = 0.0;
  double y = 0.0;
};
Vector2d::Vector2d(double x, double y) { this->x = x; this->y = y; }
void Vector2d::Rotate(double angle) {
#define vPi 3.14159265358979323846
#define vrad(d) (d * vPi / 180.0)
  double cosA = std::cos(vrad(angle));
  double sinA = std::sin(vrad(angle));
  double out[2];
  out[0] = x * cosA - y * sinA;
  out[1] = x * sinA + y * cosA;
  x = out[0];
  y = out[1];
}
double Vector2d::Dot(const Vector2d& vec) const { return x * vec.x + y * vec.y; }
double Vector2d::Magnitude() const { return std::sqrt(x * x + y * y); }
double Vector2d::ScalarProject(const Vector2d& vec) const { return Dot(vec) / vec.Magnitude();
}
// -----------------------------------------------------------------
#define LOCAL_TEST
#include "SwerveDrive.h"
#include "SwerveDrive.cpp"

#include <stdio.h>

int main()
{
  WPI_TalonSRX m = WPI_TalonSRX{};
  //WPI_TalonSRX motor[8] = {WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{}};
  //SwerveDrive s = SwerveDrive();
  printf("Hello, World!\n");
  DBG;
}
