/*To Do
 * WPI_TalonSRX add BaseMotorController into constructor
 * WPI_TalonSRX fill in PIDWrite
 */
/*
 * Test wrapper for Swerve Drive object
 * Compile with
 *  g++ -std=c++11 -o test test.cpp
 */
#include <string>
#include <ostream>
#include <cmath>
#include <functional>
#include <regex>
#include <chrono>
#include <libgen.h>
#include <stdio.h>
#define DBG      DBGST(" ")
#define DBGz(a)  DBGST("%s",(a))
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
#define f1f " %.6f "
#define DBGf(a)        DBGST(#a f1f                      , (a))
#define DBGf2(a,b)     DBGST(#a f1f #b f1f               , (a), (b))
#define DBGf3(a,b,c)   DBGST(#a f1f #b f1f #c f1f        , (a), (b), (c))
#define DBGf4(a,b,c,d) DBGST(#a f1f #b f1f #c f1f #d f1f , (a), (b), (c), (d))

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
#define DEFAULT_SAFETY_EXPIRATION 0.1
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
struct Timer {
  static uint32_t GetFPGATimestamp();
};
uint32_t Timer::GetFPGATimestamp() {
  using namespace std::chrono;
  steady_clock::duration dur{steady_clock::now().time_since_epoch()};
  uint64_t ticks = duration_cast<milliseconds>(dur).count();
  return (uint32_t)(ticks & 0xffffffff);
}
// -----------------------------------------------------------------
class MotorSafetyHelper {
 public:
  explicit MotorSafetyHelper(MotorSafety* safeObject);
  ~MotorSafetyHelper();
  void Feed();
  void SetExpiration(double expirationTime);
  double GetExpiration() const;
  bool IsAlive() const;
  void Check();
  void SetSafetyEnabled(bool enabled);
  bool IsSafetyEnabled() const;
  static void CheckMotors();
 private:
  double m_expiration;
  bool m_enabled;
  double m_stopTime;
  MotorSafety* m_safeObject;
};

MotorSafetyHelper::MotorSafetyHelper(MotorSafety* safeObject)
    : m_safeObject(safeObject) {
  DBG;
  m_enabled = false;
  m_expiration = DEFAULT_SAFETY_EXPIRATION;
  m_stopTime = Timer::GetFPGATimestamp();
}
MotorSafetyHelper::~MotorSafetyHelper() { DBG; }

void MotorSafetyHelper::Feed() { DBG; m_stopTime = Timer::GetFPGATimestamp() + m_expiration; }
void MotorSafetyHelper::SetExpiration(double expirationTime) { DBGST("expiration %f", expirationTime);
  m_expiration = expirationTime; }
double MotorSafetyHelper::GetExpiration() const { DBGST("expiration %f", m_expiration); return m_expiration; }
bool MotorSafetyHelper::IsAlive() const { bool b = !m_enabled || m_stopTime > Timer::GetFPGATimestamp();
  DBGST("isAlive %d ", b); return b; }
void MotorSafetyHelper::Check() {
  bool enabled;
  double stopTime;
  DBG;
  enabled = m_enabled;
  stopTime = m_stopTime;
  if (!enabled) return;
  if (stopTime < Timer::GetFPGATimestamp()) { m_safeObject->StopMotor(); }
}
void MotorSafetyHelper::SetSafetyEnabled(bool enabled) { DBGST("safety %d", enabled); m_enabled = enabled; }
bool MotorSafetyHelper::IsSafetyEnabled() const { DBGST("isSafety %d", m_enabled); return m_enabled; }
void MotorSafetyHelper::CheckMotors() { DBG; }

// -----------------------------------------------------------------
enum class ControlMode { PercentOutput };
// -----------------------------------------------------------------
#define DBG_SRX(a,...) DBGST("ID %d " a, m_ID, ##__VA_ARGS__)
class TalonSRX {
 public:
  TalonSRX(int deviceNumber);
  virtual ~TalonSRX();
  TalonSRX() = delete;
  TalonSRX(TalonSRX const*) = delete;
  TalonSRX* operator=(TalonSRX const*) = delete;
 protected:
  int m_ID;
};
TalonSRX::TalonSRX(int deviceNumber) { m_ID = deviceNumber; DBG_SRX(""); }
TalonSRX::~TalonSRX() { DBG_SRX(""); }
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
  bool _invert;
  double _speed = 0;
  std::string _desc;
  frc::MotorSafetyHelper _safetyHelper;
};

WPI_TalonSRX::WPI_TalonSRX(int deviceNumber) :
TalonSRX(deviceNumber),
//need SpeedController and SendableBase?
//BaseMotorController()
_safetyHelper(this)
 { DBG_SRX("");
    std::stringstream work;
    work << "Talon SRX " << deviceNumber;
    _desc = work.str();
    /* prep motor safety */
    _safetyHelper.SetExpiration(0.0);
    _safetyHelper.SetSafetyEnabled(false);
    SetName(_desc);
 }
WPI_TalonSRX::~WPI_TalonSRX() { DBG_SRX(""); }

void WPI_TalonSRX::Set(double speed) { _speed = speed; DBG_SRX("speed %f", speed); };
void WPI_TalonSRX::PIDWrite(double output) { DBG_SRX("output %f", output); }
double WPI_TalonSRX::Get() const { DBG_SRX("speed %f", _speed); return _speed; };
void WPI_TalonSRX::Set(ControlMode mode, double value) { DBG_SRX(""); }
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) { DBG_SRX(""); }
void WPI_TalonSRX::SetInverted(bool isInverted) { _invert = isInverted; DBG_SRX("invert %d", _invert); };
bool WPI_TalonSRX::GetInverted() const { DBG_SRX("isInverted %d", _invert); return _invert; };
void WPI_TalonSRX::Disable() { DBG_SRX(""); }
void WPI_TalonSRX::StopMotor() { DBG_SRX(""); }
void WPI_TalonSRX::SetExpiration(double timeout) { DBG_SRX("timeout %f", timeout); _safetyHelper.SetExpiration(timeout); }
double WPI_TalonSRX::GetExpiration() const { double e = _safetyHelper.GetExpiration(); DBG_SRX("%f", e); return e; }
bool WPI_TalonSRX::IsAlive() const { bool b = _safetyHelper.IsAlive(); DBG_SRX("isAlive %d", b); return b; }
bool WPI_TalonSRX::IsSafetyEnabled() const { bool b = _safetyHelper.IsSafetyEnabled(); DBG_SRX("isSafety %d", b); return b; }
void WPI_TalonSRX::SetSafetyEnabled(bool enabled) { _safetyHelper.SetSafetyEnabled(enabled); DBG_SRX("setSafety %d", enabled); }
void WPI_TalonSRX::GetDescription(llvm::raw_ostream& desc) const { DBG_SRX("desc %s", _desc.c_str()); desc << _desc.c_str(); }
void WPI_TalonSRX::InitSendable(frc::SendableBuilder& builder) { DBG_SRX(""); }

// -----------------------------------------------------------------
class RobotDriveBase : public MotorSafety, public SendableBase {
 public:
  int instances;
  RobotDriveBase();
  ~RobotDriveBase() override = default;
  //virtual void InitSendable(SendableBuilder& builder) = 0;
  virtual void SetName(const std::string& name, int count);
  void AddChild(void *child);
  virtual void SetExpiration(double timeout);
  virtual double GetExpiration() const;
  virtual void StopMotor() = 0;
  virtual bool IsAlive() const;
  virtual bool IsSafetyEnabled() const;
  virtual void SetSafetyEnabled(bool enabled);
  virtual void GetDescription(wpi::raw_ostream& desc) const;
  virtual void Normalize(double wheelSpeeds[], size_t size);
  MotorSafetyHelper m_safetyHelper{this};
};
RobotDriveBase::RobotDriveBase() { DBG; m_safetyHelper.SetSafetyEnabled(true); };
void RobotDriveBase::SetName(const std::string& name, int count) { DBGz(name.c_str()); } ;
void RobotDriveBase::AddChild(void *child) { DBG; };

void RobotDriveBase::SetExpiration(double timeout) { DBGST("timeout %f", timeout);
  m_safetyHelper.SetExpiration(timeout); }
double RobotDriveBase::GetExpiration() const { double t = m_safetyHelper.GetExpiration(); DBGST("expiration %f", t); return t; }
bool RobotDriveBase::IsAlive() const { bool b = m_safetyHelper.IsAlive(); DBGST("isAlive %d", b); return b; }
bool RobotDriveBase::IsSafetyEnabled() const { bool b = m_safetyHelper.IsSafetyEnabled(); DBGST("isSafety %d", b); return b; }
void RobotDriveBase::SetSafetyEnabled(bool enabled) { DBGST("setSafety %d", enabled); m_safetyHelper.SetSafetyEnabled(enabled); }
void RobotDriveBase::GetDescription(wpi::raw_ostream& desc) const { DBG; }
void RobotDriveBase::Normalize(double wheelSpeeds[], size_t size = 4) {
  double maxMagnitude = std::abs(wheelSpeeds[0]);
  for (size_t i = 1; i < size; i++) {
    double temp = std::abs(wheelSpeeds[i]);
    if (maxMagnitude < temp) {
      maxMagnitude = temp;
    }
  }
  DBGST("magnitude %f", maxMagnitude);
  if (maxMagnitude > 1.0) {
    for (size_t i = 0; i < size; i++) {
      wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
    }
  }
}

} // namespace frc
// -----------------------------------------------------------------
using namespace frc;

#define LOCAL_TEST
#define SKIP_ROTATION_NORM
#include "SwerveDrive.h"
#include "SwerveDrive.cpp"

void test_wheel() {
#if 0
  // to run 'check', make Wheel::NormalizeRotation static and add signature:
  // double m_speed_prev, double m_angle_prev, double& m_speed, double& m_angle
  #define check(a,b,c,d,e,f) do {\
    double speed, angle; \
    speed = c; \
    angle = d; \
    DBGz("==="); \
    Wheel::NormalizeRotation(a,b,speed,angle); \
    if (!(speed == e) || !(angle == f)) \
    {DBGST("ASSERT FAIL prev %.1f %.1f, next %.1f %.1f, got %.1f %.1f, expected %.1f %.1f", \
    a,b,c,d,speed,angle,e,f);} \
  } while (0)
  #define test(a,b,c,d) do { \
    double res = Wheel::AngularDistance(a,b,1.,c); \
    if (res != d) \
    {DBGST("ASSERT FAIL speed_prev %.1f prev %.1f next %.1f expected %.1f got %.1f", a,b,c,d,res);} \
  } while (0)
  check(1.,1.,1.,1.,1.,1.);
  check(-1.,1.,1.,1.,1.,1.);
  check(-1.,-90.,1.,90.,-1.,-90.);
  check(1.,1.,1.,92.,-1.,-88.);
  check(1.,-45.,1.,45.,1.,45.);
  check(1.,-45.,1.,46.,-1.,-134.);
  test(1., -1., 1., 2.);
  test(1.,  1., 1., 0.);
  test(-1.,  1., 1., 180.);
  test(-1.,  1., 180., -1.);
  test(-1.,  -90., 90., 0.);
#endif
  constexpr double l = 36./2.;
  constexpr double w = 24./2.;
  constexpr double xxx = 1. / std::sqrt(l*l+w*w); // 0.046225 -conversion factor to make comparison using swerve tester
  Wheel *wheel[4] = {
                      new Wheel( l,  w, xxx),
                      new Wheel( l, -w, xxx),
                      new Wheel(-l, -w, xxx),
                      new Wheel(-l,  w, xxx)
                    };
  for (int i = 0; i < kWheels; i++) {
    DBGz("---");
    //wheel[i]->ApplyTranslationAndRotation(1., -1., 1.);
    //wheel[i]->ApplyTranslationAndRotation(1., 1.1, 1.);
    //wheel[i]->ApplyTranslationAndRotation(0., 0., -1.);
    //wheel[i]->ApplyTranslationAndRotation(0., -1., 0.);
    //wheel[i]->ApplyTranslationAndRotation(0., 1., 0.);
    //wheel[i]->ApplyTranslationAndRotation(0., 0., -1.);
    //wheel[i]->ApplyTranslationAndRotation(0., 0., 1.);
    //wheel[i]->ApplyTranslationAndRotation(1., 1., -1.);
    //wheel[i]->ApplyTranslationAndRotation(0., 1., 0.);
    //wheel[i]->ApplyTranslationAndRotation(-1., 0., 0.);
    //wheel[i]->ApplyTranslationAndRotation(0.5, 0.5, 0.);
    //wheel[i]->ApplyTranslationAndRotation(0., 0., 0.5);
    //wheel[i]->ApplyTranslationAndRotation(0.5, 0.5, 0.25);
    //wheel[i]->ApplyTranslationAndRotation(0.5, 0., 0.25);
    //wheel[i]->ApplyTranslationAndRotation(-0.5, 0.5, 0.5);
    //wheel[i]->ApplyTranslationAndRotation(-0.25, 0.75, 0.75);
    //wheel[i]->ApplyTranslationAndRotation(0.3, 0.4,-0.5);

    double distance;
    double angle;
    if (i==0) {Wheel::CalculateAckermanCG(1., 0., 4., 4., distance, angle);}
    wheel[i]->ApplyAckermann(1., 0., distance, angle);
    //wheel[i]->ApplyAckermann(1., -1.);
    //wheel[i]->ApplyAckermann(0., -1.);
    //wheel[i]->ApplyAckermann(-1., -1.);
  }
#if 1
  double norm = 1.0;
  for (size_t i = 0; i < kWheels; i++) { double temp = std::abs(wheel[i]->Speed()); if (norm < temp) { norm = temp; } }
  if (norm > 1.0) { for (size_t i = 0; i < kWheels; i++) { wheel[i]->NormalizeSpeed(norm); } }
#endif
  for (int i = 0; i < kWheels; i++) {
    delete wheel[i];
  }
}

int main()
{
#if 1
  test_wheel();
#else
  WPI_TalonSRX *m1 = new WPI_TalonSRX(1);
  WPI_TalonSRX *m2 = new WPI_TalonSRX(2);
  WPI_TalonSRX *m3 = new WPI_TalonSRX(3);
  WPI_TalonSRX *m4 = new WPI_TalonSRX(4);
  WPI_TalonSRX *m5 = new WPI_TalonSRX(5);
  WPI_TalonSRX *m6 = new WPI_TalonSRX(6);
  WPI_TalonSRX *m7 = new WPI_TalonSRX(7);
  WPI_TalonSRX *m8 = new WPI_TalonSRX(8);
  SwerveDrive *s = new SwerveDrive(*m1,*m2,*m3,*m4,*m5,*m6,*m7,*m8,8.,8.);
  //s->DriveCartesian(1.,1.,90.,90.);
  //s->DriveCartesian(0.,0.,1.,0.);
  //s->DriveCartesian(1.,0.,0.,0.);
  //s->DriveCartesian(1.,1.,0.,0.);
  //s->DriveCartesian(1.,1.,0.,45.);
  s->DriveCartesian(-1.,0.,0.,0.);
#if 1
  s->StopMotor();
  delete s;
  delete m1;
  delete m2;
  delete m3;
  delete m4;
  delete m5;
  delete m6;
  delete m7;
  delete m8;
#endif
#endif
}

