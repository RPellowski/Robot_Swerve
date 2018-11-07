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

class PIDSource {
 public:
  double m_value;
  PIDSource() {DBG; m_value = 0;}
  ~PIDSource() {DBG;}
  double PIDGet() {DBGf(m_value); return m_value; }
};

class PIDOutput {
 public:
  PIDOutput() {DBG;}
  ~PIDOutput() {DBG;}
  void PIDWrite(double result) {DBGf(result);}
};

#define clamp(_value, _low, _high) std::max(_low, std::min(_value, _high))

void CalculateRate(PIDSource *m_origSource, PIDOutput *m_pidOutput,
  //double m_setpoint, double m_minimumOutput, double m_maximumOutput,
  //double &m_prevError, double &m_totalError, double &m_error, double &m_result,
  double m_P, double m_I, double m_D, double m_F) {
  DBG;
#if 0
    double input;
    double P;
    double I;
    double D;
    double feedForward =  return m_F * m_setpoint;

    double minimumOutput;
    double maximumOutput;
    double prevError;
    double error;
    double totalError;
double Pterm,Iterm,Dterm,Fterm;
Pterm=Iterm=Dterm=Fterm=0;
    {
      input = m_pidInput->PIDGet();
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
    result = clamp(result, minimumOutput, maximumOutput);
        m_pidOutput->PIDWrite(result);
    m_prevError = m_error;
    m_error = error;
    m_totalError = totalError;
    m_result = result;
  DBGf4(m_prevError, m_error, m_totalError, m_result);
#endif
}

#define deltaTime 0.05

void CalculatePosition(PIDSource *m_pidInput, PIDOutput *m_pidOutput,
  double m_setpoint, double &m_prevSetpoint,
  double minimumOutput, double maximumOutput, double m_inputRange,
  double &m_prevError, double &m_totalError, double &m_error, double &m_result,
  double P, double I, double D, double F) {

  DBG;
  double input;
  double feedForward;
  m_prevSetpoint = m_setpoint;
  feedForward = F * (m_setpoint - m_prevSetpoint) / deltaTime;
  double prevError;
  double error;
  double totalError;
  double Pterm,Iterm,Dterm,Fterm;

  input = m_pidInput->PIDGet();
  prevError = m_prevError;
  error = std::fmod(m_setpoint - input, m_inputRange);
  if (std::fabs(error) > m_inputRange / 2) {
    if (error > 0) {
      error -= m_inputRange;
    } else {
      error += m_inputRange;
    }
  }

  totalError = m_totalError;

  double result;
  if (I != 0) {
    totalError = clamp(totalError + error, minimumOutput / I, maximumOutput / I);
  }
  result = P * error + I * totalError + D * (error - prevError) + feedForward;
  Pterm = P * error;
  Iterm = I * totalError;
  Dterm = D * (error - prevError);
  Fterm = feedForward;
  DBGf4(Pterm, Iterm, Dterm, Fterm);

  result = clamp(result, minimumOutput, maximumOutput);
  m_pidOutput->PIDWrite(result);

  m_prevError = m_error;
  m_error = error;
  m_totalError = totalError;
  m_result = result;
  DBGf4(m_prevError, m_error, m_totalError, m_result);
}

PIDSource *m_origSource = new PIDSource();
PIDOutput *m_pidOutput = new PIDOutput();
double m_setpoint = 100;
double m_minimumOutput = -1;
double m_maximumOutput = 1;
double m_inputRange = 360;
double m_prevSetpoint = 0;
double m_prevError = 0;
double m_totalError = 0;
double m_error = 0;
double m_result = 0;
double kP = 1;
double kI = 0.1;
double kD = 0.2;
double kF = 0.3;

int main() {
  DBG;
  for (int i = 0; i < 100; i += 1) {
    DBGz("--------------------");
    CalculatePosition(m_origSource, m_pidOutput,
      m_setpoint, m_prevSetpoint,
      m_minimumOutput, m_maximumOutput, m_inputRange,
      m_prevError, m_totalError, m_error, m_result,
      kP, kI, kD, kF);
    double steerTravel = m_result * 2.0;
    m_origSource->m_value += steerTravel;
  }
  //CalculateRate();
  delete m_origSource;
  delete m_pidOutput;
}
