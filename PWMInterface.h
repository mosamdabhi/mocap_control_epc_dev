#ifndef PWM_INTERFACE
#define PWM_INTERFACE

#include <cstring>
#include "ParameterUtils.h"

class PWMInterface
{
public:
  PWMInterface();
  ~PWMInterface();

  typedef struct PWMCommand
  {
    unsigned short motor[4];
    PWMCommand()
    {
      memset(&motor, 0, sizeof(motor));
    }
  } pwm_cmd_t;

  bool initialize();
  void update();

  bool armMotors();
  bool disarmMotors();

  void printCommand(const pwm_cmd_t& p);

  void getZeroCommand(pwm_cmd_t& cmd);
  void getMaxCommand(pwm_cmd_t& cmd);

  unsigned short getZeroCommand();
  unsigned short getMaxCommand();

  unsigned short convertRPMToPWMVoltageCompensation(float rpm);

  bool sendCommand(const pwm_cmd_t& cmd);

private:
  bool loadParameters();
  bool registerCallbacks();
  bool openDevice();
  bool sendIOCTLCommand(int cmd, unsigned long arg);

  unsigned short saturatePWMCommand(unsigned short v);

  void printParameters();

  int fd;
  int pwm_map[4];

  int battery_status_sub;

  unsigned short pwm_zero;
  unsigned short pwm_min;
  unsigned short pwm_max;

  int pwm_rate;

  float current_voltage;
  float p00;
  float p10;
  float p01;
  float p20;
  float p11;
  float p02;
  float p30;
  float p21;
  float p12;
  float p03;
  float p40;
  float p31;
  float p22;
  float p13;
  float p04;
  float p50;
  float p41;
  float p32;
  float p23;
  float p14;
  float p05;
  float RPM_mean;
  float RPM_sd;
  float battery_voltage_mean;
  float battery_voltage_sd;
};
#endif
