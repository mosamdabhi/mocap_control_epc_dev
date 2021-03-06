#include <cstdio>
#include <cmath>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_pwm_output.h>
#include <uORB/topics/battery_status.h>

#include "PWMInterface.h"

namespace pu = parameter_utils;

PWMInterface::PWMInterface() : current_voltage(0.0f) { }
PWMInterface::~PWMInterface() { }

bool PWMInterface::initialize()
{
  if (!loadParameters())
  {
    puts("[PWMInterface] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[PWMInterface] failed to register callbacks");
    return false;
  }

  if (!openDevice())
  {
    puts("[PWMInterface] failed to open device");
    return false;
  }

#if 1
  printParameters();
#endif

  return true;
}

void PWMInterface::printCommand(const pwm_cmd_t& p)
{
  printf("[PWMInterface] pwm cmd = %hu, %hu, %hu, %hu\n",
         p.motor[0], p.motor[1],
         p.motor[2], p.motor[3]);

  char buf[32];
  sprintf(buf, "%0.2f", (double)current_voltage);
  printf("[PWMInterface] current voltage = %s\n", buf);
}

unsigned short PWMInterface::convertRPMToPWMVoltageCompensation(float rpm)
{
  //float t = voltage_coeff*current_voltage + rpm_coeff*rpm + affine_coeff;
  float x = (rpm - RPM_mean)/RPM_sd;
  float y = (current_voltage - battery_voltage_mean)/battery_voltage_sd;
  float t = (p00 + p10*x + p01*y + p20*powf(x,2) + p11*x*y + p02*powf(y,2) + p30*powf(x,3)
            + p21*powf(x,2)*y + p12*x*powf(y,2) + p03*powf(y,3) + p40*powf(x,4) + p31*powf(x,3)*y
            + p22*powf(x,2)*powf(y,2) + p13*x*powf(y,3) + p04*powf(y,4) + p50*powf(x,5) + p41*powf(x,4)*y
            + p32*powf(x,3)*powf(y,2) + p23*powf(x,2)*powf(y,3) + p14*x*powf(y,4) + p05*powf(y,5));
  if (t < 0.0f) t = 0.0f;
  return saturatePWMCommand((unsigned short)roundf(t));
}

bool PWMInterface::sendIOCTLCommand(int cmd, unsigned long arg)
{
  int ret = ioctl(fd, cmd, arg);
  if (ret != OK)
  {
    puts("[PWMInterface] ioctl failed");
    return false;
  }

  return true;
}

void PWMInterface::getZeroCommand(pwm_cmd_t& cmd)
{
  for (unsigned int i = 0; i < 4; i++)
    cmd.motor[i] = pwm_zero;
}

void PWMInterface::getMaxCommand(pwm_cmd_t& cmd)
{
  for (unsigned int i = 0; i < 4; i++)
    cmd.motor[i] = pwm_max;
}

unsigned short PWMInterface::getZeroCommand()
{
  return pwm_zero;
}

unsigned short PWMInterface::getMaxCommand()
{
  return pwm_max;
}

unsigned short PWMInterface::saturatePWMCommand(unsigned short v)
{
  unsigned short t;
  if (v > pwm_max)
    t = pwm_max;
  else if (v < pwm_min)
    t = pwm_min;
  else
    t = v;

  return t;
}

bool PWMInterface::sendCommand(const pwm_cmd_t& cmd)
{
  for (unsigned int i = 0; i < 4; i++)
  {
    if (!sendIOCTLCommand(PWM_SERVO_SET(pwm_map[i]), cmd.motor[i]))
    {
      printf("[PWMInterface] failed to set PWM_SERVO_SET(%d)", i);
      return false;
    }
  }

  return true;
}

bool PWMInterface::loadParameters()
{
  // Use the values associated with main
  pwm_zero = pu::getUShortParam("PWM_AUX_DISARMED");
  pwm_min = pu::getUShortParam("PWM_AUX_MIN");
  pwm_max = pu::getUShortParam("PWM_AUX_MAX");

  // Shift to index value
  pwm_map[0] = pu::getIntParam("MCC_PWM_MAP1") - 1;
  pwm_map[1] = pu::getIntParam("MCC_PWM_MAP2") - 1;
  pwm_map[2] = pu::getIntParam("MCC_PWM_MAP3") - 1;
  pwm_map[3] = pu::getIntParam("MCC_PWM_MAP4") - 1;

  pwm_rate = pu::getIntParam("MCC_PWM_RATE");

  p00 = pu::getFloatParam("MCC_RPM2PWM_00");
  p10 = pu::getFloatParam("MCC_RPM2PWM_10");
  p01 = pu::getFloatParam("MCC_RPM2PWM_01");
  p20 = pu::getFloatParam("MCC_RPM2PWM_20");
  p11 = pu::getFloatParam("MCC_RPM2PWM_11");
  p02 = pu::getFloatParam("MCC_RPM2PWM_02");
  p30 = pu::getFloatParam("MCC_RPM2PWM_30");
  p21 = pu::getFloatParam("MCC_RPM2PWM_21");
  p12 = pu::getFloatParam("MCC_RPM2PWM_12");
  p03 = pu::getFloatParam("MCC_RPM2PWM_03");
  p40 = pu::getFloatParam("MCC_RPM2PWM_40");
  p31 = pu::getFloatParam("MCC_RPM2PWM_31");
  p22 = pu::getFloatParam("MCC_RPM2PWM_22");
  p13 = pu::getFloatParam("MCC_RPM2PWM_13");
  p04 = pu::getFloatParam("MCC_RPM2PWM_04");
  p50 = pu::getFloatParam("MCC_RPM2PWM_50");
  p41 = pu::getFloatParam("MCC_RPM2PWM_41");
  p32 = pu::getFloatParam("MCC_RPM2PWM_32");
  p23 = pu::getFloatParam("MCC_RPM2PWM_23");
  p14 = pu::getFloatParam("MCC_RPM2PWM_14");
  p05 = pu::getFloatParam("MCC_RPM2PWM_05");
  RPM_mean =  pu::getFloatParam("MCC_RPM_M");
  RPM_sd =  pu::getFloatParam("MCC_RPM_SD");
  battery_voltage_mean =  pu::getFloatParam("MCC_V_M");
  battery_voltage_sd = pu::getFloatParam("MCC_V_SD");
  //rpm_coeff = 1.0f / rpm_per_pwm;
  //voltage_coeff = -rpm_per_volt / rpm_per_pwm;
  //affine_coeff = -rpm_at_zero_pwm_and_volts / rpm_per_pwm;
  return true;
}

void PWMInterface::update()
{
  bool updated;
  orb_check(battery_status_sub, &updated);

  if (updated)
  {
    struct battery_status_s bs;
    orb_copy(ORB_ID(battery_status), battery_status_sub, &bs);
    current_voltage = bs.voltage_filtered_v;
  }

  return;
}

bool PWMInterface::registerCallbacks()
{
  battery_status_sub = orb_subscribe(ORB_ID(battery_status));
  if (battery_status_sub < 0)
  {
    puts("battery_status_sub failed");
    return false;
  }

  return true;
}

void PWMInterface::printParameters()
{
  printf("[PWMInterface] pwm_zero = %hu\n", pwm_zero);
  printf("[PWMInterface] pwm_min = %hu\n", pwm_min);
  printf("[PWMInterface] pwm_max = %hu\n", pwm_max);

  // Hack to deal with NuttX printf of float/double values
  //char buf[32];
  //sprintf(buf, "%0.2f", (double)rpm_coeff);
  //printf("[PWMInterface] rpm_coeff = %s\n", buf);
  //memset(buf, 0, sizeof(buf));

  //sprintf(buf, "%0.2f", (double)voltage_coeff);
  //printf("[PWMInterface] voltage_coeff = %s\n", buf);
  //memset(buf, 0, sizeof(buf));

  //sprintf(buf, "%0.2f", (double)affine_coeff);
  //printf("[PWMInterface] affine_coeff = %s\n", buf);
  //memset(buf, 0, sizeof(buf));
}

bool PWMInterface::openDevice()
{
  const char *pwm_dev = PWM_OUTPUT0_DEVICE_PATH;
  fd = open(pwm_dev, 0);
  if (fd < 0)
  {
    printf("[PWMInterface] can't open %s", pwm_dev);
    return false;
  }

  int ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, pwm_rate);
  if (ret != OK)
  {
    puts("[PWMInterface] ioctl set pwm update rate failed");
    return false;
  }

  return true;
}

bool PWMInterface::armMotors()
{
  if (!sendIOCTLCommand(PWM_SERVO_SET_ARM_OK, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_SET_ARM_OK");
    return false;
  }

  if (!sendIOCTLCommand(PWM_SERVO_ARM, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_ARM");
    return false;
  }

  return true;
}

bool PWMInterface::disarmMotors()
{
  if (!sendIOCTLCommand(PWM_SERVO_DISARM, 0))
  {
    puts("[PWMInterface] failed to set PWM_SERVO_DISARM");
    return false;
  }

  return true;
}
