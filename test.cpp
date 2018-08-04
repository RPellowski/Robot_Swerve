/*
 * Test wrapper for Swerve Drive object
 * Compile with
 *  g++ -std=c++11 -o a test.cpp
 */
#include <string>
#include <ostream>
#include <cmath>
#include <functional>
#include <regex>
#include <libgen.h>
#include <pthread.h>
#include "/home/rob/utils/dbg"
#undef DBGST
#define DBGST(a,...) \
  do { \
    std::string f;                                                     \
    std::regex re1("\\(.*");                                           \
    std::regex re2(".*[ :](.*::)([^\\(]*).*");                         \
    std::string repl1 = "";                                            \
    std::string repl2 = "$1$2";                                        \
    f = regex_replace(__PRETTY_FUNCTION__, re1, repl1);                \
    f = regex_replace(f,                   re2, repl2);                \
    fprintf(stdout, "%5d %-20.20s %-40.40s : " a "\n",                 \
      __LINE__, basename((char *)__FILE__), f.c_str(), ##__VA_ARGS__); \
  } while (0)

//    std::regex re("([^:]*::)+(.*::[^(]*)\\(.*");                          \

#define DBGf(a) DBGST(#a "%f",(a))

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
void SendableBase::SetName(const std::string& name) { m_name = name; DBGz(name.c_str()); };
std::string SendableBase::GetName() const { DBGz(m_name.c_str()); return m_name; }
void SendableBase::SetSubsystem(const std::string& subsystem) { m_subsystem = subsystem; DBGz(subsystem.c_str()); };
std::string SendableBase::GetSubsystem() const { DBGz(m_subsystem.c_str()); return m_subsystem; }
void SendableBase::InitSendable(SendableBuilder& builder) { DBG; };
// -----------------------------------------------------------------
class SendableBuilder {
public:
  SendableBuilder();
  virtual ~SendableBuilder() = default;
  void SetSmartDashboardType(const std::string& name);
  void AddDoubleProperty(const std::string& key,
      std::function<double()> getter, std::function<void(double)> setter);
};
SendableBuilder::SendableBuilder() { DBG; };
//SendableBuilder::~SendableBuilder() { DBG; };
void SendableBuilder::SetSmartDashboardType(const std::string& name) { DBGz(name.c_str()); };
void SendableBuilder::AddDoubleProperty(const std::string& key,
    std::function<double()> getter, std::function<void(double)> setter) { DBGz(key.c_str()); };

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
  int m_ID;
  bool _invert;
  double _speed = 0;
  std::string _desc;
  frc::MotorSafetyHelper _safetyHelper;
};

#define DBG_SRX(a,...) DBGST("ID %d " a, m_ID, ##__VA_ARGS__)
WPI_TalonSRX::WPI_TalonSRX(int deviceNumber) { m_ID = deviceNumber; DBG_SRX(""); }
WPI_TalonSRX::~WPI_TalonSRX() { DBG_SRX(""); }

void WPI_TalonSRX::Set(double speed) { _speed = speed; DBG_SRX("speed %f", speed); };
void WPI_TalonSRX::PIDWrite(double output) { DBG_SRX("output %f", output); }
double WPI_TalonSRX::Get() const { DBG_SRX("speed %f", _speed); return _speed; };
void WPI_TalonSRX::Set(ControlMode mode, double value) { DBG_SRX(""); }
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) { DBG_SRX(""); }
void WPI_TalonSRX::SetInverted(bool isInverted) { _invert = isInverted; DBG_SRX("isInverted %d", _invert); };
bool WPI_TalonSRX::GetInverted() const { DBG_SRX("isInverted %d", _invert); return _invert; };
void WPI_TalonSRX::Disable() { DBG_SRX(""); }
void WPI_TalonSRX::StopMotor() { DBG_SRX(""); }
void WPI_TalonSRX::SetExpiration(double timeout) { DBG_SRX("timeout %f", timeout); }
double WPI_TalonSRX::GetExpiration() const { DBG_SRX(""); }
bool WPI_TalonSRX::IsAlive() const { DBG_SRX(""); }
bool WPI_TalonSRX::IsSafetyEnabled() const { DBG_SRX(""); }
void WPI_TalonSRX::SetSafetyEnabled(bool enabled) { DBG_SRX("enabled %d", enabled); }
void WPI_TalonSRX::GetDescription(llvm::raw_ostream& desc) const { DBG_SRX(""); desc << "WPI_TalonSRX"; }
void WPI_TalonSRX::InitSendable(frc::SendableBuilder& builder) { DBG_SRX(""); }

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
