constexpr double T1 = 0.2;      // Measured time, s: 0->full velocity (filter 1)
constexpr double T2 = T1 / 2.0; // Jerk time, s: 0->full acceleration (filter 2)
constexpr double kfV = 0.0;     // Velocity feedforward gain
constexpr double kfA = 0.0;     // Acceleration feedforward gain
constexpr double kfJ = 0.0;     // Jerk feedforward gain
constexpr double kP  = 0.0;     // Proportional gain
constexpr double kD  = 0.0;     // Derivative gain
constexpr double kDD = 0.0;     // Double derivative gain
constexpr double kI  = 0.0;     // Integral gain
constexpr double itp = 0.025;   // Loop time, ms

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
#define f1f " %.3f "
#define DBGv(a)        DBGST(#a " %d"                    , (a))
#define DBGf(a)        DBGST(#a f1f                      , (a))
#define DBGf2(a,b)     DBGST(#a f1f #b f1f               , (a), (b))
#define DBGf3(a,b,c)   DBGST(#a f1f #b f1f #c f1f        , (a), (b), (c))
#define DBGf4(a,b,c,d) DBGST(#a f1f #b f1f #c f1f #d f1f , (a), (b), (c), (d))

#ifdef SKIP
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
  feedForward = F * (m_setpoint - m_prevSetpoint) / deltaTime;
  m_prevSetpoint = m_setpoint;
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
/*
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
//double kP = 1;
//double kI = 0.1;
//double kD = 0.2;
double kF = 0.3;

int old_main() {
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

int calculate_forward() {
}
*/
#endif // SKIP

#define xDBG
class TargetGenerator {
/*
 * Refs:
 *   https://www.chiefdelphi.com/forums/showthread.php?t=98358&page=2
 *   https://www.controleng.com/articles/the-velocity-of-pid/
 *   https://www.controleng.com/articles/feed-forwards-augment-pid-control/
 *   https://en.wikipedia.org/wiki/Step_response
 *
 * Steps in the use of this object
 *     1. Measure open loop operation on step input
 *     2. Determine closed loop (feedback) parameters
 *     3. Measure closed loop operation on step input
 *     4. Determine feedforward parameters
 *     repeat 2-4
 */
public:
 /*
  * Measured robot system parameters
  *  - Open loop (no control system in place)
  *  - Robot system under test has final weight configuration
  *  - Input a step function at full value to the robot
  *  - Measure the robot response function (position, velocity) for the entire
  *    range from 0 to full velocity
  *  - Determine T1 and T2 from the plots (or calculate T1, and T2 = T1 / 2)
  */
 double m_T1; // Time in ms for filter 1 (measured time to acc 0 to full vel)
 double m_T2; // Time in ms for filter 2 (jerk or time to increase acc to max)

 // Sofware system loop time
 double m_itp; // Iteration time (loop time)

 // Calculated from measured parameters
 int m_FL1; // Filter 1 length = T1/itp
 int m_FL2; // Filter 2 length = T2/idp

 // Dynamic setpoint defined parameters
 double m_V; // Target Velocity
 double m_P; // Target Position
 double m_T4; // Time in ms to get to destination
 int m_N; // Total number of inputs to filter

 // Dynamic generator output calculated parameters
 double m_Vout; // Output Velocity
 double m_Pout; // Output Position
 double m_Aout; // Output Acceleration
 double m_Jout; // Output Jerk

 /*
  * Feedforward control system tuning parameters
  *   These are determined by measuring the step response of robot after
  *   feedforward and feedback have been implemented as the control system.
  *   Measure output of robot and compare to desired (generator) output.
  *   Adjust the feedforward gains so that the difference between measured
  *   and desired is small, resulting in feedback terms being minimized.
  */
 double m_kPff; // FeedForward Pos Gain
 double m_kVff; // FeedForward Vel Gain
 double m_kAff; // FeedForward Acc Gain
 double m_kJff; // FeedForward Jerk Gain

 TargetGenerator(double T1, double T2, double itp) {
  DBGf3(T1, T2, itp);
  m_kPff = 0.0; m_kVff = 0.0; m_kAff = 0.0; m_kJff = 0.0;
  m_T1 = T1; m_T2 = T2; m_itp = itp;
  m_FL1 = int(T1 / itp);
  m_FL2 = int(T2 / itp);
  m_N = m_FL1 + m_FL2;
  //create ring buffer, length is FL1 + FL2
  //set up
  };
 ~TargetGenerator() {DBG;};
 void SetGains(double kPff, double kVff, double kAff, double kJff) {
   DBGf4(kPff, kVff, kAff, kJff);
   m_kPff = kPff; m_kVff = kVff; m_kAff = kAff; m_kJff = kJff;
   };
 void Generate(double V, double P) {DBGf2(V,P);
//fill ring buffer with lookahead only to N
   m_V = V; m_P = P;
   };
 void Calculate() {xDBG;
//shift ring buffer for f1 and f2
//based on in/out, adjust sums (+in, -out)
//calculate feedforward
 m_Vout=
 m_Pout=
 m_Aout=
 m_Jout=0;
 };
};

/*
 * This simulation maintains a buffer of calculated values of position, velocity,
 * acceleration and jerk as a feedforward target generator. It is used in
 * conjunction with the measured values in a complete PID feedback generator.
 * The input value, the setpoint, is assumed to be far enough in the future to
 * clear out the two FIR filters (T1 = time to accelerate to full velocity +
 * T2 = time to achieve full acceleration)
 * As the setpoint is updated, the filters are adjusted.
 */
#define ff " %9.3f "
#define PRd \
do { \
  printf(f1f "|", dval); \
  for (int i = 0; i < d.size(); i += 1) { \
    printf(f1f " ", d[i]); \
  } \
  printf("|" f1f "|", dsum); \
  for (int i = 0; i < e.size(); i += 1) { \
    printf(f1f " ", e[i]); \
  } \
  printf("|" f1f "|", esum); \
  printf(ff, vel); \
  printf(ff, pos); \
  printf(ff, acc); \
  printf(ff, jrk); \
  printf("\n"); \
} while(0)

#define DSIZE 4
#define ESIZE 2

int main() {
  DBG;
  double vprog = 10;
  double feed = 8;
  double dval = 0;
  double dsum = 0;
  double esum = 0;
  double vel = 0;
  double pvel = 0;
  double pos = 0;
  double acc = 0;
  double pacc = 0;
  double jrk = 0;
  double itp = 0.05;
  std::deque<double> d;
  std::deque<double> e;
  for (int i = 0; i < DSIZE; i += 1) {
    d.push_front(0);
  }
  for (int i = 0; i < ESIZE; i += 1) {
    e.push_front(0);
  }
  PRd;
  for (int i = 0; i < 16; i += 1) {
    dval = std::max(0., std::min(feed, 1.));
    feed -= dval;
    d.push_front(dval);
    dsum += dval - d.back();
    d.pop_back();
    e.push_front(dsum);
    esum += dsum - e.back();
    e.pop_back();
    pvel = vel;
    vel = esum / ESIZE / DSIZE * vprog;
    pos += (pvel + vel) / 2 * itp;
    pacc = acc;
    acc = (vel - pvel) / itp;
    jrk = (acc - pacc) / itp;
    PRd;
  }
}

int foo() {
  TargetGenerator *tg = new TargetGenerator(0.2, 0.1, 0.25);
  DBG;
  tg->Generate(10.0, 4.0);
  for (int i = 0; i < 10; i += 1) {
    tg->Calculate();
    DBGf4(tg->m_Vout, tg->m_Pout, tg->m_Aout, tg->m_Jout);
  }
}
