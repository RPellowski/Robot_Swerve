/*
 * Test wrapper for Swerve Drive object
 * Compile with
 *  g++ -std=c++11 -o a test.cpp
 */
#include <string>
#include <ostream>
#include <cmath>
#include <libgen.h>
#include <pthread.h>
#include "/home/rob/utils/dbg"
#define DBGf(a) DBGST("%f",(a))

// -----------------------------------------------------------------
namespace llvm {
  typedef std::ostream raw_ostream;
};
// -----------------------------------------------------------------
namespace wpi {
  typedef std::ostream raw_ostream;
};

namespace frc {
// -----------------------------------------------------------------
class MotorSafety {
 public:
  virtual void SetExpiration(double timeout) = 0;
  virtual double GetExpiration() const = 0;
  virtual bool IsAlive() const = 0;
  virtual void StopMotor() = 0;
  virtual void SetSafetyEnabled(bool enabled) = 0;
  virtual bool IsSafetyEnabled() const = 0;
  virtual void GetDescription(wpi::raw_ostream& desc) const = 0;
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
enum class ControlMode { PercentOutput };
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
  WPI_TalonSRX(WPI_TalonSRX const*) = delete;
  WPI_TalonSRX* operator=(WPI_TalonSRX const*) = delete;
#if 1
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
#endif
 private:
  int m_ID;
  double m_speed;
  bool m_inverted;
    double _speed = 0;
    std::string _desc;
    frc::MotorSafetyHelper _safetyHelper;
};

WPI_TalonSRX::WPI_TalonSRX(int deviceNumber) { m_ID = deviceNumber; DBGST("ID %d", m_ID); }
WPI_TalonSRX::~WPI_TalonSRX() { DBG; }
#if 1
void WPI_TalonSRX::Set(double speed) { m_speed = speed; DBGST("ID %d speed %f", m_ID, speed); };
void WPI_TalonSRX::PIDWrite(double output) { DBGf(output); }
double WPI_TalonSRX::Get() const { DBGf(m_speed); return m_speed; };
void WPI_TalonSRX::Set(ControlMode mode, double value) { DBG; }
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) { DBG; }
void WPI_TalonSRX::SetInverted(bool isInverted) { m_inverted = isInverted; DBG; };
bool WPI_TalonSRX::GetInverted() const { DBG; return m_inverted; };
void WPI_TalonSRX::Disable() { DBGST("ID %d", m_ID); }
void WPI_TalonSRX::StopMotor() { DBGST("ID %d ", m_ID); }
void WPI_TalonSRX::SetExpiration(double timeout) { DBGST("ID %d timeout %f", m_ID, timeout); }
double WPI_TalonSRX::GetExpiration() const { DBG; }
bool WPI_TalonSRX::IsAlive() const { DBG; }
bool WPI_TalonSRX::IsSafetyEnabled() const { DBG; }
void WPI_TalonSRX::SetSafetyEnabled(bool enabled) { DBGST("ID %d enabled %d", m_ID, enabled); }
void WPI_TalonSRX::GetDescription(llvm::raw_ostream& desc) const { DBG; desc << "WPI_TalonSRX"; }
void WPI_TalonSRX::InitSendable(frc::SendableBuilder& builder) { DBG; }
#endif

// -----------------------------------------------------------------
class RobotDriveBase : public MotorSafety, public SendableBase {
 public:
  int instances;
  RobotDriveBase();
  ~RobotDriveBase() override = default;
  virtual void InitSendable(SendableBuilder& builder) = 0;
  virtual void StopMotor() = 0;
  virtual void SetName(const std::string& name, int count);
  virtual void SetExpiration(double timeout);
  virtual double GetExpiration() const;
  virtual bool IsAlive() const;
  virtual void SetSafetyEnabled(bool enabled);
  virtual bool IsSafetyEnabled() const;
  virtual void GetDescription(wpi::raw_ostream& desc) const;
  void AddChild(void *child);
  MotorSafetyHelper m_safetyHelper;
};
RobotDriveBase::RobotDriveBase() { DBG; };
//RobotDriveBase::~RobotDriveBase() { DBG; };
void RobotDriveBase::SetName(const std::string& name, int count) { DBGz(name.c_str()); } ;
void RobotDriveBase::AddChild(void *child) { DBG; };

void RobotDriveBase::SetExpiration(double timeout) { DBG; }
double RobotDriveBase::GetExpiration() const { DBG; return 0.0; }
bool RobotDriveBase::IsAlive() const { DBG; return false; }
void RobotDriveBase::SetSafetyEnabled(bool enabled) { DBG; }
bool RobotDriveBase::IsSafetyEnabled() const { DBG; return false; }
void RobotDriveBase::GetDescription(wpi::raw_ostream& desc) const { DBG; }
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

} // namespace frc
// -----------------------------------------------------------------
using namespace frc;

#define LOCAL_TEST
#include "SwerveDrive.h"
#include "SwerveDrive.cpp"

#include <stdio.h>

int main()
{
  WPI_TalonSRX *m1 = new WPI_TalonSRX(1);
  WPI_TalonSRX *m2 = new WPI_TalonSRX(2);
  WPI_TalonSRX *m3 = new WPI_TalonSRX(3);
  WPI_TalonSRX *m4 = new WPI_TalonSRX(4);
  WPI_TalonSRX *m5 = new WPI_TalonSRX(5);
  WPI_TalonSRX *m6 = new WPI_TalonSRX(6);
  WPI_TalonSRX *m7 = new WPI_TalonSRX(7);
  WPI_TalonSRX *m8 = new WPI_TalonSRX(8);
  //WPI_TalonSRX motor[8] = {WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{},WPI_TalonSRX{}};
  SwerveDrive *s = new SwerveDrive(*m1,*m2,*m3,*m4,*m5,*m6,*m7,*m8,12.,24.);
  s->DriveCartesian(1.,1.,90.,0.);
  s->StopMotor();
  //printf("Hello, World!\n");
  DBG;
}
