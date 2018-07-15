#include <string>
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
  //virtual void GetDescription() const = 0;
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
  std::string GetName() const override;
  virtual void SetSubsystem(const std::string& subsystem);
  virtual void InitSendable(SendableBuilder& builder);
 private:
  std::string m_name;
  std::string m_subsystem = "Ungrouped";
};
SendableBase::SendableBase(bool addLiveWindow) { DBG; }
SendableBase::~SendableBase() { DBG; }
std::string SendableBase::GetName() const { return m_name; }
void  SendableBase::SetSubsystem(const std::string& subsystem) { m_subsystem = subsystem; DBG; };
void  SendableBase::InitSendable(SendableBuilder& builder) { DBG; };
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
class WPI_TalonSRX : public MotorSafety, public SpeedController {
 public:
  virtual ~WPI_TalonSRX() = default;
  virtual void Set(double speed);
  virtual double Get() const;
  virtual void SetInverted(bool isInverted);
  virtual bool GetInverted() const;
  virtual void Disable();
  virtual void StopMotor();
 private:
  double m_speed;
  bool m_inverted;
};
void WPI_TalonSRX::Set(double speed) { m_speed = speed; DBG; };
double WPI_TalonSRX::Get() const { DBG; return m_speed; };
void WPI_TalonSRX::SetInverted(bool isInverted) { m_inverted = isInverted; DBG; };
bool WPI_TalonSRX::GetInverted() const { DBG; return m_inverted; };
void WPI_TalonSRX::Disable() { DBG; };
void WPI_TalonSRX::StopMotor() { DBG; };
// -----------------------------------------------------------------
class RobotDriveBase : public MotorSafety, public SendableBase {
public:
  int instances;
  RobotDriveBase();
  ~RobotDriveBase() override = default;
  virtual void InitSendable(SendableBuilder& builder) = 0;
  virtual void StopMotor() = 0;
  void SetName(const char *name, int count);
  void AddChild(void *child);
};
RobotDriveBase::RobotDriveBase() { DBG; };
//RobotDriveBase::~RobotDriveBase() { DBG; };
void RobotDriveBase::SetName(const char *name, int count) { DBG; } ;
void RobotDriveBase::AddChild(void *child) { DBG; };
// -----------------------------------------------------------------

#include "SwerveDrive.h"
#include "SwerveDrive.cpp"

#include <stdio.h>

int main()
{
  //SendableBase s = SendableBase(true);
  printf("Hello, World!\n");
  DBG;
}
