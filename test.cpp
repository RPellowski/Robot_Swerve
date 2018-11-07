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
std::regex re1("\\([^\\)]*\\)");
std::regex re2("(.*)::(.*)");
std::regex re3("(.*)[ :]");
std::regex re4(".*::(?!.*::)");
#define xDBGST(a,...) do{printf("%5d %-20.20s %-50.50s : " a "\n",__LINE__,__FILE__,__PRETTY_FUNCTION__,##__VA_ARGS__);}while(0)
// Note: valgrind reports memory leaks with regex_replace() and basename()
bool DBGon = true;
#define DBGST(a,...) \
  do { \
if (DBGon) {\
    std::string _f,_fclss,_fmeth;    \
    _f = regex_replace(__PRETTY_FUNCTION__,    re1, "");   \
    _fclss = regex_replace(_f,                 re2, "$1"); \
    _fclss = regex_replace(_fclss,             re3, "");   \
    _fmeth = _fclss + "::" + regex_replace(_f, re4, "");   \
    fprintf(stdout, "%5d %-20.20s %-40.40s : " a "\n",     \
     __LINE__, basename((char *)__FILE__),                 \
     _fmeth.c_str(), ##__VA_ARGS__);                       \
}\
  } while (0)
#define f1f " %.6f "
#define DBGv(a)        DBGST(#a " %d"                    , (a))
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
  typedef std::string Twine;
  typedef std::string& StringRef;

  template<typename T>
  class ArrayRef {
    using size_type = size_t;
    using value_type = T;
  private:
    const T *Data = nullptr;
    size_type Length = 0;
  public:
    /*implicit*/ ArrayRef() { DBG; } // = default;
    template <size_t N> /*implicit*/ /*constexpr*/ ArrayRef(const std::array<T, N> &Arr) : Data(Arr.data()), Length(N) { DBG; }
    template<typename A> /*implicit*/ ArrayRef(const std::vector<T, A> &Vec) : Data(Vec.data()), Length(Vec.size()) { DBG; }
    size_t size() const { DBG; return Length; }
  };
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
  void SetName(const std::string& moduleType, int channel);
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
void SendableBase::SetName(const std::string& moduleType, int channel) { SetName(moduleType + '[' + std::to_string(channel) + ']'); }
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
  void AddBooleanProperty(const std::string& key,
      std::function<bool()> getter, std::function<void(bool)> setter);
  void AddDoubleProperty(const std::string& key,
      std::function<double()> getter, std::function<void(double)> setter);
  void AddStringProperty(
      const std::string& key, std::function<std::string()> getter,
      std::function<void(wpi::StringRef)> setter);
};
SendableBuilder::SendableBuilder() { DBG; };
void SendableBuilder::SetSmartDashboardType(const std::string& name) { DBGz(name.c_str()); };
void SendableBuilder::AddDoubleProperty(const std::string& key,
    std::function<double()> getter, std::function<void(double)> setter) { DBGz(key.c_str()); };
void SendableBuilder::AddBooleanProperty(const std::string& key,
    std::function<bool()> getter, std::function<void(bool)> setter) { DBGz(key.c_str()); };
void SendableBuilder::AddStringProperty(
    const std::string& key, std::function<std::string()> getter,
    std::function<void(wpi::StringRef)> setter) { DBGz(key.c_str()); };

// -----------------------------------------------------------------
class PIDOutput {
 public:
  PIDOutput();
  virtual ~PIDOutput();
  virtual void PIDWrite(double output) = 0;
};
PIDOutput::PIDOutput() { DBG; }
PIDOutput::~PIDOutput() { DBG; }
// -----------------------------------------------------------------
class SpeedController : public PIDOutput {
 public:
  virtual ~SpeedController();
  virtual void Set(double speed) = 0;
  virtual double Get() const = 0;
  virtual void SetInverted(bool isInverted) = 0;
  virtual bool GetInverted() const = 0;
  virtual void Disable() = 0;
  virtual void StopMotor() = 0;
};
SpeedController::~SpeedController() { DBG; }
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
void WPI_TalonSRX::Set(ControlMode mode, double value) { DBG_SRX("add more DBG"); }
void WPI_TalonSRX::Set(ControlMode mode, double demand0, double demand1) { DBG_SRX("add more DBG"); }
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
enum class PIDSourceType { kDisplacement, kRate };

class PIDSource {
 public:
  PIDSource();
  virtual ~PIDSource();
  virtual void SetPIDSourceType(PIDSourceType pidSource);
  virtual PIDSourceType GetPIDSourceType() const;
  virtual double PIDGet() = 0;
 protected:
  PIDSourceType m_pidSource = PIDSourceType::kDisplacement;
};
PIDSource::PIDSource() { DBG; }
PIDSource::~PIDSource() { DBG; }
void PIDSource::SetPIDSourceType(PIDSourceType pidSource) { DBG; m_pidSource = pidSource; }
PIDSourceType PIDSource::GetPIDSourceType() const {
  DBGz((m_pidSource == PIDSourceType::kDisplacement ? "kDisplacement": "kRate"));
return m_pidSource; }

// -----------------------------------------------------------------
class Controller {
 public:
  Controller();
  virtual ~Controller(); // = default;
  virtual void Enable() = 0;
  virtual void Disable() = 0;
};
Controller::Controller() { DBG; }
Controller::~Controller() { DBG; }

// -----------------------------------------------------------------
class PIDInterface {
 public:
  PIDInterface();
  virtual ~PIDInterface();
  virtual void SetPID(double p, double i, double d) = 0;
  virtual double GetP() const = 0;
  virtual double GetI() const = 0;
  virtual double GetD() const = 0;
  virtual void SetSetpoint(double setpoint) = 0;
  virtual double GetSetpoint() const = 0;
  virtual void Reset() = 0;
};
PIDInterface::PIDInterface() { DBG; }
PIDInterface::~PIDInterface() { DBG; };

// -----------------------------------------------------------------
template <class T>
struct NullDeleter {
  void operator()(T*) const noexcept {};
};

// -----------------------------------------------------------------
class Filter : public PIDSource {
 public:
  explicit Filter(PIDSource& source);
  explicit Filter(std::shared_ptr<PIDSource> source);
  virtual ~Filter(); // = default;
  void SetPIDSourceType(PIDSourceType pidSource) override;
  PIDSourceType GetPIDSourceType() const override;
  double PIDGet() override = 0;
  virtual double Get() const = 0;
  virtual void Reset() = 0;
 protected:
  double PIDGetSource();
 private:
  std::shared_ptr<PIDSource> m_source;
};
Filter::Filter(PIDSource& source) : m_source(std::shared_ptr<PIDSource>(&source, NullDeleter<PIDSource>())) { DBG; }
Filter::Filter(std::shared_ptr<PIDSource> source) : m_source(std::move(source)) { DBG; }
void Filter::SetPIDSourceType(PIDSourceType pidSource) { DBG; m_source->SetPIDSourceType(pidSource); }
PIDSourceType Filter::GetPIDSourceType() const { DBG; return m_source->GetPIDSourceType(); }
double Filter::PIDGetSource() { DBG; return m_source->PIDGet(); }
Filter::~Filter() { DBG; }

// -----------------------------------------------------------------
class LinearDigitalFilter : public Filter {
 public:
  LinearDigitalFilter(PIDSource& source, wpi::ArrayRef<double> ffGains, wpi::ArrayRef<double> fbGains);
  LinearDigitalFilter(std::shared_ptr<PIDSource> source, wpi::ArrayRef<double> ffGains, wpi::ArrayRef<double> fbGains);
  ~LinearDigitalFilter();
  static LinearDigitalFilter MovingAverage(std::shared_ptr<PIDSource> source, int taps);
  double Get() const override;
  void Reset() override;
  double PIDGet() override;
 private:
  double m_val;
};
LinearDigitalFilter::LinearDigitalFilter(PIDSource& source, wpi::ArrayRef<double> ffGains, wpi::ArrayRef<double> fbGains)
    : Filter(source),
      m_val(0) { DBG; }
LinearDigitalFilter::LinearDigitalFilter(std::shared_ptr<PIDSource> source, wpi::ArrayRef<double> ffGains, wpi::ArrayRef<double> fbGains)
    : Filter(source),
      m_val(0) { DBG; }
LinearDigitalFilter::~LinearDigitalFilter() { DBG; }
LinearDigitalFilter LinearDigitalFilter::MovingAverage(
    std::shared_ptr<PIDSource> source, int taps) {
  DBG;
  std::vector<double> gains(taps, 1.0 / taps);
  return LinearDigitalFilter(std::move(source), gains, {});
}
double LinearDigitalFilter::Get() const { DBGf(m_val); return m_val; }
void LinearDigitalFilter::Reset() { DBG; m_val = 0; }
double LinearDigitalFilter::PIDGet() { m_val = PIDGetSource(); DBGf(m_val); return m_val; }

// -----------------------------------------------------------------
#define WPI_DEPRECATED(a)
class PIDBase : public SendableBase, public PIDInterface, public PIDOutput {
 public:
  PIDBase(double p, double i, double d, PIDSource& source, PIDOutput& output);
  PIDBase(double p, double i, double d, double f, PIDSource& source, PIDOutput& output);
  ~PIDBase() override;
  PIDBase(const PIDBase&) = delete;
  PIDBase& operator=(const PIDBase) = delete;
  //virtual double Get() const;
  virtual void SetContinuous(bool continuous = true);
  virtual void SetInputRange(double minimumInput, double maximumInput);
  virtual void SetOutputRange(double minimumOutput, double maximumOutput);
  void SetPID(double p, double i, double d) override;
  virtual void SetPID(double p, double i, double d, double f);
  void SetP(double p);
  void SetI(double i);
  void SetD(double d);
  void SetF(double f);
  double GetP() const override;
  double GetI() const override;
  double GetD() const override;
  virtual double GetF() const;
  void SetSetpoint(double setpoint) override;
  double GetSetpoint() const override;
  double GetDeltaSetpoint() const;
  //virtual double GetError() const;
  //WPI_DEPRECATED("Use a LinearDigitalFilter as the input and GetError().")
  //virtual double GetAvgError() const;
  //virtual void SetPIDSourceType(PIDSourceType pidSource);
  //virtual PIDSourceType GetPIDSourceType() const;
  //WPI_DEPRECATED("Use SetPercentTolerance() instead.")
  //virtual void SetTolerance(double percent);
  //virtual void SetAbsoluteTolerance(double absValue);
  //virtual void SetPercentTolerance(double percentValue);
  //WPI_DEPRECATED("Use a LinearDigitalFilter as the input.")
  //virtual void SetToleranceBuffer(int buf = 1);
  //virtual bool OnTarget() const;
  void Reset() override;
  void PIDWrite(double output) override;
  //void InitSendable(SendableBuilder& builder) override;
 //protected:
  bool m_enabled = false;
  //mutable wpi::mutex m_thisMutex;
  //mutable wpi::mutex m_pidWriteMutex;
  PIDSource* m_pidInput;
  PIDOutput* m_pidOutput;
  //Timer m_setpointTimer;
  virtual void Calculate();
  virtual double CalculateFeedForward();
  double GetContinuousError(double error) const;
 private:
  double m_P;
  double m_I;
  double m_D;
  double m_F;
  double m_maximumOutput = 1.0;
  double m_minimumOutput = -1.0;
  double m_maximumInput = 0;
  double m_minimumInput = 0;
  double m_inputRange = 0;
  bool m_continuous = false;
  double m_prevError = 0;
  double m_totalError = 0;
  enum { kAbsoluteTolerance, kPercentTolerance, kNoTolerance } m_toleranceType = kNoTolerance;
  double m_tolerance = 0.05;
  double m_setpoint = 0;
  double m_prevSetpoint = 0;
  double m_error = 0;
  double m_result = 0;
  std::shared_ptr<PIDSource> m_origSource;
  LinearDigitalFilter m_filter{nullptr, {}, {}};
};

template <class T>
constexpr const T& clamp(const T& value, const T& low, const T& high) {
  return std::max(low, std::min(value, high));
}

PIDBase::PIDBase(double Kp, double Ki, double Kd, PIDSource& source, PIDOutput& output)
    : PIDBase(Kp, Ki, Kd, 0.0, source, output) { DBG; }

PIDBase::PIDBase(double Kp, double Ki, double Kd, double Kf, PIDSource& source, PIDOutput& output)
    : SendableBase(false) {
  DBGf4(Kp,Ki,Kd,Kf);
  m_P = Kp;
  m_I = Ki;
  m_D = Kd;
  m_F = Kf;
  m_origSource = std::shared_ptr<PIDSource>(&source, NullDeleter<PIDSource>());
  m_filter = LinearDigitalFilter::MovingAverage(m_origSource, 1);
  m_pidInput = &m_filter;
  m_pidOutput = &output;
  //m_setpointTimer.Start();
  static int instances = 0;
  instances++;
  SetName("PIDController", instances);
}
PIDBase::~PIDBase() { DBG; }
/*
double PIDBase::Get() const {
  return m_result;
}
*/
void PIDBase::SetContinuous(bool continuous) {
  m_continuous = continuous;
}
void PIDBase::SetInputRange(double minimumInput, double maximumInput) {
  {
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
    m_inputRange = maximumInput - minimumInput;
  }
  SetSetpoint(m_setpoint);
}
void PIDBase::SetOutputRange(double minimumOutput, double maximumOutput) {
  DBGf2(minimumOutput, maximumOutput); m_minimumOutput = minimumOutput; m_maximumOutput = maximumOutput; }
void PIDBase::SetPID(double p, double i, double d) { DBG; { m_P = p; m_I = i; m_D = d; } }
void PIDBase::SetPID(double p, double i, double d, double f) {  DBG; m_P = p; m_I = i; m_D = d; m_F = f; }
void PIDBase::SetP(double p) { DBG;  m_P = p; }
void PIDBase::SetI(double i) { DBG; m_I = i; }
void PIDBase::SetD(double d) { DBG; m_D = d; }
void PIDBase::SetF(double f) { DBG; m_F = f; }
double PIDBase::GetP() const { DBG; return m_P; }
double PIDBase::GetI() const { DBG; return m_I; }
double PIDBase::GetD() const { DBG; return m_D; }
double PIDBase::GetF() const { DBG; return m_F; }
void PIDBase::SetSetpoint(double setpoint) {
  {
    DBGf(setpoint);
    if (m_maximumInput > m_minimumInput) {
      if (setpoint > m_maximumInput)
        m_setpoint = m_maximumInput;
      else if (setpoint < m_minimumInput)
        m_setpoint = m_minimumInput;
      else
        m_setpoint = setpoint;
    } else {
      m_setpoint = setpoint;
    }
  }
}
double PIDBase::GetSetpoint() const { DBGf(m_setpoint); return m_setpoint; }
double PIDBase::GetDeltaSetpoint() const { double rc = (m_setpoint - m_prevSetpoint) /*/ m_setpointTimer.Get()*/ ;
  DBGf(rc); return rc; }
/*
double PIDBase::GetError() const { double setpoint = GetSetpoint(); { return GetContinuousError(setpoint - m_pidInput->PIDGet()); } }
double PIDBase::GetAvgError() const { return GetError(); }
void PIDBase::SetPIDSourceType(PIDSourceType pidSource) { m_pidInput->SetPIDSourceType(pidSource); }
PIDSourceType PIDBase::GetPIDSourceType() const { return m_pidInput->GetPIDSourceType(); }
void PIDBase::SetTolerance(double percent) { m_toleranceType = kPercentTolerance; m_tolerance = percent; }
void PIDBase::SetAbsoluteTolerance(double absTolerance) { m_toleranceType = kAbsoluteTolerance; m_tolerance = absTolerance; }
void PIDBase::SetPercentTolerance(double percent) { m_toleranceType = kPercentTolerance; m_tolerance = percent; }
void PIDBase::SetToleranceBuffer(int bufLength) { m_filter = LinearDigitalFilter::MovingAverage(m_origSource, bufLength); m_pidInput = &m_filter; }
bool PIDBase::OnTarget() const {
  double error = GetError();
  switch (m_toleranceType) {
    case kPercentTolerance:
      return std::fabs(error) < m_tolerance / 100 * m_inputRange;
      break;
    case kAbsoluteTolerance:
      return std::fabs(error) < m_tolerance;
      break;
    case kNoTolerance:
      return false;
  }
  return false;
}
*/
void PIDBase::Reset() {
  DBG;
  m_prevError = 0;
  m_totalError = 0;
  m_result = 0;
}
void PIDBase::PIDWrite(double output) { DBGf(output); SetSetpoint(output); }
/*
void PIDBase::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("PIDBase");
  builder.SetSafeState([=]() { Reset(); });
  builder.AddDoubleProperty("p", [=]() { return GetP(); },
                            [=](double value) { SetP(value); });
  builder.AddDoubleProperty("i", [=]() { return GetI(); },
                            [=](double value) { SetI(value); });
  builder.AddDoubleProperty("d", [=]() { return GetD(); },
                            [=](double value) { SetD(value); });
  builder.AddDoubleProperty("f", [=]() { return GetF(); },
                            [=](double value) { SetF(value); });
  builder.AddDoubleProperty("setpoint", [=]() { return GetSetpoint(); },
                            [=](double value) { SetSetpoint(value); });
}
*/
void PIDBase::Calculate() {
  DBG;
  if (m_origSource == nullptr || m_pidOutput == nullptr) return;
  bool enabled;
    enabled = m_enabled;
  if (enabled) {
    double input;
    PIDSourceType pidSourceType;
    double P;
    double I;
    double D;
    double feedForward = CalculateFeedForward();
    double minimumOutput;
    double maximumOutput;
    double prevError;
    double error;
    double totalError;
double Pterm,Iterm,Dterm,Fterm;
Pterm=Iterm=Dterm=Fterm=0;
    {
      input = m_pidInput->PIDGet();
      pidSourceType = m_pidInput->GetPIDSourceType();
      P = m_P;
      I = m_I;
      D = m_D;
      minimumOutput = m_minimumOutput;
      maximumOutput = m_maximumOutput;
      prevError = m_prevError;
      error = GetContinuousError(m_setpoint - input);
      totalError = m_totalError;
    }
    double result;
    if (pidSourceType == PIDSourceType::kRate) {
      if (P != 0) {
        totalError =
            clamp(totalError + error, minimumOutput / P, maximumOutput / P);
      }
      result = D * error + P * totalError + feedForward;
Pterm = P * totalError;
Iterm = 0;
Dterm = D * error;
Fterm = feedForward;
DBGf4(Pterm, Iterm, Dterm, Fterm);
    } else {
      if (I != 0) {
        totalError =
            clamp(totalError + error, minimumOutput / I, maximumOutput / I);
      }
      result =
          P * error + I * totalError + D * (error - prevError) + feedForward;
Pterm = P * error;
Iterm = I * totalError;
Dterm = D * (error - prevError);
Fterm = feedForward;
DBGf4(Pterm, Iterm, Dterm, Fterm);
    }
    result = clamp(result, minimumOutput, maximumOutput);
    {
      if (m_enabled) {
        m_pidOutput->PIDWrite(result);
      }
    }
    m_prevError = m_error;
    m_error = error;
    m_totalError = totalError;
    m_result = result;
  }
  DBGf4(m_prevError, m_error, m_totalError, m_result);
}
double PIDBase::CalculateFeedForward() {
  double feedForward;
  if (m_pidInput->GetPIDSourceType() == PIDSourceType::kRate) {
    feedForward = m_F * GetSetpoint();
  } else {
    double temp = m_F * GetDeltaSetpoint();
    m_prevSetpoint = m_setpoint;
    //m_setpointTimer.Reset();
    feedForward = temp;
  }
  DBGf(feedForward);
  return feedForward;
}
double PIDBase::GetContinuousError(double error) const {
  double rc = error;
  if (m_continuous && m_inputRange != 0) {
    error = std::fmod(error, m_inputRange);
    if (std::fabs(error) > m_inputRange / 2) {
      if (error > 0) {
        rc = error - m_inputRange;
      } else {
        rc = error + m_inputRange;
      }
    }
  }
  DBGf3(error, m_inputRange, rc);
  return rc;
}

// -----------------------------------------------------------------
/*
#include "Base.h"
#include "Controller.h"
#include "Filters/LinearDigitalFilter.h"
#include "Notifier.h"
#include "PIDBase.h"
#include "PIDSource.h"
#include "Timer.h"
*/
class PIDController : public PIDBase, public Controller {
 public:
  PIDController(double p, double i, double d,           PIDSource* source, PIDOutput* output, double period = 0.05);
  PIDController(double p, double i, double d, double f, PIDSource* source, PIDOutput* output, double period = 0.05);
  PIDController(double p, double i, double d,           PIDSource& source, PIDOutput& output, double period = 0.05);
  PIDController(double p, double i, double d, double f, PIDSource& source, PIDOutput& output, double period = 0.05);
  ~PIDController() override;
  PIDController(const PIDController&) = delete;
  PIDController& operator=(const PIDController) = delete;
  void Enable() override;
  void Disable() override;
  void SetEnabled(bool enable);
  bool IsEnabled() const;
  void Reset() override;
  void InitSendable(SendableBuilder& builder) override;
 private:
  //std::unique_ptr<Notifier> m_controlLoop;
};
/*
#include "PIDController.h"
#include "Notifier.h"
#include "PIDOutput.h"
#include "SmartDashboard/SendableBuilder.h"
*/
PIDController::PIDController(double Kp, double Ki, double Kd,            PIDSource* source, PIDOutput* output, double period)
             : PIDController(Kp, Ki, Kd, 0.0, *source, *output, period ) { DBG; }
PIDController::PIDController(double Kp, double Ki, double Kd, double Kf, PIDSource* source, PIDOutput* output, double period)
             : PIDController(Kp, Ki, Kd, Kf,  *source, *output, period ) { DBG; }
PIDController::PIDController(double Kp, double Ki, double Kd,            PIDSource& source, PIDOutput& output, double period)
             : PIDController(Kp, Ki, Kd, 0.0, source, output, period   ) { DBG; }
PIDController::PIDController(double Kp, double Ki, double Kd, double Kf, PIDSource& source, PIDOutput& output, double period)
             : PIDBase(Kp, Ki, Kd, Kf,        source, output           ) { DBG;
  //m_controlLoop = std::make_unique<Notifier>(&PIDController::Calculate, this);
  //m_controlLoop->StartPeriodic(period);
}

PIDController::~PIDController()             { DBG; /*m_controlLoop->Stop();*/ }
void PIDController::Enable()                { DBG; m_enabled = true; }
void PIDController::Disable()               { DBG; m_enabled = false; m_pidOutput->PIDWrite(0); }
void PIDController::SetEnabled(bool enable) { DBG; if (enable) { Enable(); } else { Disable(); } }
bool PIDController::IsEnabled() const       { DBG; return m_enabled; }
void PIDController::Reset()                 { DBG; Disable(); PIDBase::Reset(); }
void PIDController::InitSendable(SendableBuilder& builder) {
  PIDBase::InitSendable(builder); DBG;
  builder.AddBooleanProperty("enabled", [=]() { return IsEnabled(); }, [=](bool value) { SetEnabled(value); });
}

// -----------------------------------------------------------------
class ErrorBase {
 public:
  ErrorBase();
  virtual ~ErrorBase() = default;
  ErrorBase(const ErrorBase&) = delete;
  ErrorBase& operator=(const ErrorBase&) = delete;
};
ErrorBase::ErrorBase() { DBG; }

// -----------------------------------------------------------------
class CounterBase {
 public:
  enum EncodingType { k1X, k2X, k4X };

  virtual ~CounterBase() = default;
  virtual int Get() const = 0;
  virtual void Reset() = 0;
  virtual double GetPeriod() const = 0;
  virtual void SetMaxPeriod(double maxPeriod) = 0;
  virtual bool GetStopped() const = 0;
  virtual bool GetDirection() const = 0;
};

// -----------------------------------------------------------------
class DigitalSource;

// -----------------------------------------------------------------
class Encoder : public ErrorBase, public SendableBase, public CounterBase, public PIDSource {
 public:
  enum IndexingType { kResetWhileHigh, kResetWhileLow, kResetOnFallingEdge, kResetOnRisingEdge };
  Encoder(int aChannel, int bChannel, bool reverseDirection = false, EncodingType encodingType = k4X);
  //Encoder(DigitalSource* aSource, DigitalSource* bSource, bool reverseDirection = false, EncodingType encodingType = k4X);
  //Encoder(DigitalSource& aSource, DigitalSource& bSource, bool reverseDirection = false, EncodingType encodingType = k4X);
  //Encoder(std::shared_ptr<DigitalSource> aSource, std::shared_ptr<DigitalSource> bSource, bool reverseDirection = false,
  //        EncodingType encodingType = k4X);
  ~Encoder() override;
  int Get() const override;
  void Reset() override;
  double GetPeriod() const override;
  void SetMaxPeriod(double maxPeriod) override;
  bool GetStopped() const override;
  bool GetDirection() const override;
  int GetRaw() const;
  void SetRaw(int32_t raw); // Added for debugging
  int GetEncodingScale() const;
  double GetDistance() const;
  double GetRate() const;
  void SetMinRate(double minRate);
  void SetDistancePerPulse(double distancePerPulse);
  double GetDistancePerPulse() const;
  void SetReverseDirection(bool reverseDirection);
  void SetSamplesToAverage(int samplesToAverage);
  int GetSamplesToAverage() const;
  double PIDGet() override;
  //void SetIndexSource(int channel, IndexingType type = kResetOnRisingEdge);
  //void SetIndexSource(const DigitalSource& source, IndexingType type = kResetOnRisingEdge);
  //int GetFPGAIndex() const;
  void InitSendable(SendableBuilder& builder) override;
 private:
  void InitEncoder(bool reverseDirection, EncodingType encodingType);
  double DecodingScaleFactor() const;
  int m_samplesToAverage;
  EncodingType m_encodingType;
  int m_encodingScale;
  double m_distancePerPulse;
  double m_maxPeriod;
  bool m_reverseDirection;
  double m_period;
  int32_t m_raw;
};
Encoder::Encoder(int aChannel, int bChannel, bool reverseDirection, EncodingType encodingType) {
  m_encodingType = encodingType;
  m_raw = 0.;
  switch (encodingType) {
    case k4X: {
      m_encodingScale = 4;
      SetMaxPeriod(.5);
      break;
    }
    case k1X:
      m_encodingScale = 1;
      break;
    case k2X: {
      m_encodingScale = 2;
      break;
    }
    default:
      break;
  }
  DBGST("aChannel %d bChannel %d reverseDirection %d encodingScale %d", aChannel, bChannel, reverseDirection, m_encodingScale);
}

Encoder::~Encoder() { DBG; }

int32_t Encoder::Get() const {
  int32_t v = static_cast<int32_t>(GetRaw() * DecodingScaleFactor());
  DBGv(v);
  return v;
}

int32_t Encoder::GetRaw() const                            { DBGv(m_raw); return m_raw; }
void Encoder::SetRaw(int32_t raw)                          { DBGv(raw); m_raw = raw; }
int32_t Encoder::GetEncodingScale() const                  { DBGv(m_encodingScale); return m_encodingScale; }
void Encoder::Reset()                                      { DBG; }
double Encoder::GetPeriod() const                          { DBGf(m_period); return m_maxPeriod; }
void Encoder::SetMaxPeriod(double maxPeriod)               { DBGf(maxPeriod); m_maxPeriod = maxPeriod; }
bool Encoder::GetStopped() const                           { DBG; return true; }
bool Encoder::GetDirection() const                         { DBGv(m_reverseDirection); return m_reverseDirection; }
double Encoder::GetDistance() const                        { DBG; return GetRaw() * DecodingScaleFactor() * m_distancePerPulse; }
double Encoder::GetRate() const                            { double r = m_distancePerPulse / GetPeriod(); DBGf(r); return r;}
void Encoder::SetMinRate(double minRate)                   { DBGf(minRate); SetMaxPeriod(m_distancePerPulse / minRate); }
void Encoder::SetDistancePerPulse(double distancePerPulse) { DBGf(distancePerPulse); m_distancePerPulse = distancePerPulse; }
void Encoder::SetReverseDirection(bool reverseDirection)   { DBGv(reverseDirection); m_reverseDirection = reverseDirection; }
void Encoder::SetSamplesToAverage(int32_t samplesToAverage) {
  DBGv(samplesToAverage);
  if (samplesToAverage < 1 || samplesToAverage > 127) { return; }
  m_samplesToAverage = samplesToAverage;
}
int32_t Encoder::GetSamplesToAverage() const { DBGv(m_samplesToAverage); return m_samplesToAverage; }
double Encoder::DecodingScaleFactor() const {
   double rc;
   switch (m_encodingType) {
    case k1X: rc = 1.0; break;
    case k2X: rc = 0.5; break;
    case k4X: rc = 0.25; break;
    default: rc = 0.0; break;
  }
  DBGf(rc);
  return rc;
}
double Encoder::PIDGet() {
   double rc;
   switch (GetPIDSourceType()) {
    case PIDSourceType::kDisplacement: rc = GetDistance(); break;
    case PIDSourceType::kRate: rc = GetRate(); break;
    default: rc = 0.0; break;
  }
  DBGf(rc);
  return rc;
}
void Encoder::InitSendable(SendableBuilder& builder) { DBG; }
void Encoder::InitEncoder(bool reverseDirection, EncodingType encodingType) {
  m_encodingType = encodingType;
  m_reverseDirection = reverseDirection;
  DBGST("encodingType %d reverseDirection %d", encodingType, reverseDirection);
}
#define xFOOBAR
#ifdef FOOBAR
} // namespace frc
// -----------------------------------------------------------------
using namespace frc;
class PS :  public PIDSource {
 public:
  virtual ~PS();
  virtual double PIDGet();
};
double PS::PIDGet() {DBG; return 1.0;}
PS::~PS() { DBG; };

class PO : public PIDOutput {
  public:
   virtual ~PO();
   virtual void PIDWrite(double d);
};
void PO::PIDWrite(double d) { DBGf(d); }
PO::~PO() { DBG; };

int main()
{
#if 1
  DBG;
  Encoder e(1,2);
  e.SetDistancePerPulse(5);
  e.SetReverseDirection(true);
  e.SetSamplesToAverage(127);
  e.SetMinRate(3.);
  e.SetMaxPeriod(0.5);
  e.GetRaw();
  e.GetEncodingScale();
  e.Reset();
  e.GetPeriod();
  e.GetStopped();
  e.GetDirection();
  e.GetDistance();
  e.GetRate();
  e.GetSamplesToAverage();
  //e.DecodingScaleFactor();
  e.PIDGet();

#else
DBGz("===================");
  PS *ps = new PS();
  ps->SetPIDSourceType(PIDSourceType::kRate);
DBGz("===================");
  PO *po = new PO();
DBGz("===================");
  PIDBase *pb = new PIDBase(1.,0.,0.,*ps,*po);
  pb->SetPID(3.,2.,1.);
  pb->SetPID(3.,2.,1.,10.);
  double p = pb->GetP(); DBGf(p);
  double i = pb->GetI(); DBGf(i);
  double d = pb->GetD(); DBGf(d);
  double f = pb->GetF(); DBGf(f);
  pb->SetSetpoint(7.);
  double s = pb->GetSetpoint(); DBGf(s);
DBGz("===================");
  pb->m_enabled = true;
  pb->Calculate();
DBGz("===================");
  delete pb;
  delete po;
  delete ps;
#endif
}
#else //FOOBAR

// -----------------------------------------------------------------
//class CommandGroup;
class Command : public ErrorBase, public SendableBase {
  //friend class CommandGroup;
  //friend class Scheduler;
 public:
  Command();
  //explicit Command(const wpi::Twine& name);
  //explicit Command(double timeout);
  //Command(const wpi::Twine& name, double timeout);
  ~Command() override = default;
  //double TimeSinceInitialized() const;
  //void Requires(Subsystem* s);
  //void Start();
  //bool Run();
  //void Cancel();
  //bool IsRunning() const;
  //bool IsInitialized() const;
  //bool IsCompleted() const;
  //bool IsCanceled() const;
  //bool IsInterruptible() const;
  //void SetInterruptible(bool interruptible);
  //bool DoesRequire(Subsystem* subsystem) const;
  //typedef std::set<Subsystem*> SubsystemSet;
  //const SubsystemSet& GetRequirements() const;
  //CommandGroup* GetGroup() const;
  //void SetRunWhenDisabled(bool run);
  //bool WillRunWhenDisabled() const;
  //int GetID() const;
 protected:
  //void SetTimeout(double timeout);
  //bool IsTimedOut() const;
  //bool AssertUnlocked(const std::string& message);
  //void SetParent(CommandGroup* parent);
  //bool IsParented() const;
  //void ClearRequirements();
  //virtual void Initialize();
  //virtual void Execute();
  //virtual bool IsFinished() = 0;
  //virtual void End();
  //virtual void Interrupted();
  //virtual void _Initialize();
  //virtual void _Interrupted();
  //virtual void _Execute();
  //virtual void _End();
  //virtual void _Cancel();
  //friend class ConditionalCommand;
 private:
  //void LockChanges();
  //void Removed();
  //void StartRunning();
  //void StartTiming();
  //double m_startTime = -1;
  //double m_timeout;
  //bool m_initialized = false;
  //SubsystemSet m_requirements;
  //bool m_running = false;
  //bool m_interruptible = true;
  //bool m_canceled = false;
  //bool m_locked = false;
  //bool m_runWhenDisabled = false;
  //CommandGroup* m_parent = nullptr;
  //bool m_completed = false;
  //int m_commandID = m_commandCounter++;
  //static int m_commandCounter;
 public:
  //void InitSendable(SendableBuilder& builder) override;
};

// -----------------------------------------------------------------

class Subsystem : public ErrorBase, public SendableBase {
  friend class Scheduler;
 public:
  explicit Subsystem(const wpi::Twine& name);
  void SetDefaultCommand(Command* command);
  Command* GetDefaultCommand();
  std::string GetDefaultCommandName();
  void SetCurrentCommand(Command* command);
  Command* GetCurrentCommand() const;
  std::string GetCurrentCommandName() const;
  virtual void Periodic();
  virtual void InitDefaultCommand();
  void AddChild(const std::string& name, std::shared_ptr<Sendable> child);
  void AddChild(const std::string& name, Sendable* child);
  void AddChild(const std::string& name, Sendable& child);
  void AddChild(std::shared_ptr<Sendable> child);
  void AddChild(Sendable* child);
  void AddChild(Sendable& child);
 private:
  void ConfirmCommand();
  Command* m_currentCommand = nullptr;
  bool m_currentCommandChanged = true;
  Command* m_defaultCommand = nullptr;
  bool m_initializedDefaultCommand = false;
 public:
  void InitSendable(SendableBuilder& builder) override;
};
Subsystem::Subsystem(const std::string& name) {
  SetName(name);
  //Scheduler::GetInstance()->RegisterSubsystem(this);
}
void Subsystem::SetDefaultCommand(Command* command) {
/*
  if (command == nullptr) {
    m_defaultCommand = nullptr;
  } else {
    const auto& reqs = command->GetRequirements();
    if (std::find(reqs.begin(), reqs.end(), this) == reqs.end()) {
      wpi_setWPIErrorWithContext(
          CommandIllegalUse, "A default command must require the subsystem");
      return;
    }
*/
    m_defaultCommand = command;
//  }
}
Command* Subsystem::GetDefaultCommand() {
  if (!m_initializedDefaultCommand) {
    m_initializedDefaultCommand = true;
    InitDefaultCommand();
  }
  return m_defaultCommand;
}
std::string Subsystem::GetDefaultCommandName() {
  Command* defaultCommand = GetDefaultCommand();
  if (defaultCommand) {
    return defaultCommand->GetName();
  } else {
    std::string *s = new std::string("");
    return *s;
  }
}
void Subsystem::SetCurrentCommand(Command* command) {
  m_currentCommand = command;
  m_currentCommandChanged = true;
}
Command* Subsystem::GetCurrentCommand() const { return m_currentCommand; }
std::string Subsystem::GetCurrentCommandName() const {
  Command* currentCommand = GetCurrentCommand();
  if (currentCommand) {
    return currentCommand->GetName();
  } else {
    std::string *s = new std::string("");
    return *s;
  }
}

void Subsystem::Periodic() {}
void Subsystem::InitDefaultCommand() {}
void Subsystem::AddChild(const wpi::Twine& name, std::shared_ptr<Sendable> child) { AddChild(name, *child); }
void Subsystem::AddChild(const wpi::Twine& name, Sendable* child) { AddChild(name, *child); }
void Subsystem::AddChild(const wpi::Twine& name, Sendable& child) {
  child.SetName(GetSubsystem(), name);
  //LiveWindow::GetInstance()->Add(&child);
}
void Subsystem::AddChild(std::shared_ptr<Sendable> child) { AddChild(*child); }
void Subsystem::AddChild(Sendable* child) { AddChild(*child); }
void Subsystem::AddChild(Sendable& child) {
  child.SetSubsystem(GetSubsystem());
  //LiveWindow::GetInstance()->Add(&child);
}
void Subsystem::ConfirmCommand() {
  if (m_currentCommandChanged) m_currentCommandChanged = false;
}
void Subsystem::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("Subsystem");
  builder.AddBooleanProperty(
      ".hasDefault", [=]() { return m_defaultCommand != nullptr; }, nullptr);
  builder.AddStringProperty(".default",
                            [=]() { return GetDefaultCommandName(); }, nullptr);

  builder.AddBooleanProperty(
      ".hasCommand", [=]() { return m_currentCommand != nullptr; }, nullptr);
  builder.AddStringProperty(".command",
                            [=]() { return GetCurrentCommandName(); }, nullptr);
}

// -----------------------------------------------------------------

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
    double omega;
    double north = -1;
    double east = 1;
    if (i==0) {Wheel::CalculateAckermanCG(north, east, std::abs(l),
               distance, omega);}
    wheel[i]->ApplyAckermann(north, distance, omega);
    //wheel[i]->ApplyAckermann(1., -1.);
    //wheel[i]->ApplyAckermann(0., -1.);
    //wheel[i]->ApplyAckermann(-1., -1.);
  }
  DBGz("---");
#if 1
  double norm = 1.0;
  for (size_t i = 0; i < kWheels; i++) { double temp = std::abs(wheel[i]->Speed()); if (norm < temp) { norm = temp; } }
  //if (norm > 1.0) { for (size_t i = 0; i < kWheels; i++) { wheel[i]->NormalizeSpeed(norm); } }
  if (norm > 1.0) { for (size_t i = 0; i < kWheels; i++) { wheel[i]->Speed(wheel[i]->Speed()/norm); } }
#endif
  for (int i = 0; i < kWheels; i++) {
    delete wheel[i];
  }
}

int main()
{
#if 0
  test_wheel();
#else
//  WPI_TalonSRX *m1 = new WPI_TalonSRX(1);
//  WPI_TalonSRX *m2 = new WPI_TalonSRX(2);
//  WPI_TalonSRX *m3 = new WPI_TalonSRX(3);
//  WPI_TalonSRX *m4 = new WPI_TalonSRX(4);
//  WPI_TalonSRX *m5 = new WPI_TalonSRX(5);
//  WPI_TalonSRX *m6 = new WPI_TalonSRX(6);
//  WPI_TalonSRX *m7 = new WPI_TalonSRX(7);
//  WPI_TalonSRX *m8 = new WPI_TalonSRX(8);
  SwerveDrive *s = new SwerveDrive(); //*m1,*m2,*m3,*m4,*m5,*m6,*m7,*m8,24.,30.);
  //s->DriveCartesian(1.,1.,90.,90.);
  //s->DriveCartesian(0.,0.,1.,0.);
  //s->DriveCartesian(1.,0.,0.,0.);
  //s->DriveCartesian(1.,1.,0.,0.);
  //s->DriveCartesian(1.,1.,0.,45.);
  //s->DriveCartesian(-1.,0.,0.,0.);
#ifdef VELOCITY_PID
int counter = 0;
while (counter < 140) {
  s->DriveCartesian(1.,1.,1.,0.);
  counter++;
  DBGz("==========");
}
#else
  s->DriveCartesian(1.,1.,1.,0.);
#endif
  delete(s);
#if 0
int counter = 0;
while (not drive_done && counter < 10) {
  s->DriveCartesian(1.,1.,1.,0.);
  counter++;
  DBGz("==========");
}
bool DBGon = false;
bool drive_done = false;
    if (DBGon) {
  // Set steering motor angles first
  for (size_t i = 0; i < kWheels; i++) {
    if (i == 0) {DBGon = true;}
    DBGz("-------------------------------");
    m_pid[i]->SetSetpoint(m_wheel[i]->Angle());
    m_pid[i]->Calculate();
    // Estimate degrees in 50 ms
    int steerTravel = int(m_steer[i]->Get() * 45. / 360. * 4000);
    m_angle[i]->SetRaw(m_angle[i]->GetRaw()+steerTravel);
    DBGon = false;
    //drive_done = true;

#endif
#if 0
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
#endif // FOOBAR
