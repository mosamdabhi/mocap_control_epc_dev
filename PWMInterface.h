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

  unsigned short convertRPMToPWM(float rpm);

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

  // Voltage compensation
  bool rpm_control;
  float rpm_coeff;
  float voltage_coeff;
  float affine_coeff;
  float current_voltage;
};
#endif
