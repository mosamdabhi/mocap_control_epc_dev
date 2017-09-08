/****************************************************************************
 *
 * Copyright (C) 2016 Nathan Michael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <cmath>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "MocapAttitudeController.h"

namespace pu = parameter_utils;

MocapAttitudeController::MocapAttitudeController() :
  enable_motors(false),
  cmd_time(0),
  ctrl_state_set(false),
  motor_mode(OFF),
  input_mode(RPM),
  motor_start_set(false),
  cascaded_command_pub(nullptr)
{
  inertia.zero();
}

MocapAttitudeController::~MocapAttitudeController() { }

bool MocapAttitudeController::initialize()
{
  if (!loadParameters())
  {
    puts("[MAC] failed to load parameters");
    return false;
  }

  if (!registerCallbacks())
  {
    puts("[MAC] failed to register callbacks");
    return false;
  }

  if (!mm.initialize())
  {
    puts("[MAC] failed to initialize motor manager");
    return false;
  }

  l1_att_observer.loadParameters();

  return true;
}

void MocapAttitudeController::finalize()
{
  mm.finalize();

  closeSubscriptions();
}

void MocapAttitudeController::update(bool cmd_updated_ext)
{
  // Update the motor manager
  mm.update();

  bool cmd_updated = cmd_updated_ext;
  bool updated;

  if (!cmd_updated_ext)
  {
    orb_check(rpm_cmd_sub, &updated);
    if (updated)
    {
      struct mocap_rpm_command_s cmd_in;
      orb_copy(ORB_ID(mocap_rpm_command), rpm_cmd_sub, &cmd_in);
      cmd_updated = mocapRPMCommandMessageCallback(cmd_in);
    }

    orb_check(cascaded_cmd_sub, &updated);
    if (updated)
    {
      struct cascaded_command_s cmd_in;
      orb_copy(ORB_ID(cascaded_command), cascaded_cmd_sub, &cmd_in);
      cmd_updated = cascadedCommandMessageCallback(cmd_in);
    }
  }

  orb_check(cascaded_cmd_gains_sub, &updated);
  if (updated)
  {
    struct cascaded_command_gains_s cmd_in;
    orb_copy(ORB_ID(cascaded_command_gains), cascaded_cmd_gains_sub, &cmd_in);
    cascadedCommandGainsMessageCallback(cmd_in);
  }

  orb_check(mocap_motor_state_sub, &updated);
  if (updated)
  {
    struct mocap_motor_state_s mms;
    orb_copy(ORB_ID(mocap_motor_state), mocap_motor_state_sub, &mms);
    mocapMotorStateMessageCallback(mms);
  }

  bool cmd_received = false;
  uint64_t tnow = hrt_absolute_time();

  if (cmd_updated)
  {
    cmd_time = tnow;
    cmd_received = true;
  }

  if (enable_motors && (motor_mode == OFF))
    setMotorMode(START);

  if (!enable_motors)
    setMotorMode(OFF);

  switch (motor_mode)
  {
    case START:
    {
      if (!motor_start_set)
      {
        motor_start_set = true;
        tmotor_start = tnow;
      }

      mm.setRPMCommand(MotorManager::START);
      if (tnow - tmotor_start > motor_start_dt_us)
      {
        motor_start_set = false;
        setMotorMode(IDLE);
      }
      break;
    }
    case OFF:
    {
      mm.setRPMCommand(MotorManager::OFF);
      att_cmd_casc = att_cmd_zero;
      break;
    }
    case IDLE:
    {
      if (cmd_received)
        setMotorMode(RUNNING);
      else
        mm.setRPMCommand(MotorManager::MIN);
      break;
    }
    case RUNNING:
    {
      if (tnow - cmd_time > cmd_ttl_us)
      {
        setMotorMode(IDLE);
        break;
      }

      switch (input_mode)
      {
        case RPM:
        {
          mm.setRPMCommand(rpm_cmd);
          break;
        }
        case CASCADED:
        {
          MotorManager::rpm_cmd_t cmd;
          if (updateCascadedAttitudeController(cmd))
            mm.setRPMCommand(cmd);
          else
            mm.setRPMCommand(MotorManager::MIN);
          break;
        }
      }
    }
  }

  mm.sendCommand();

#if 0
  static unsigned int debug_counter = 0;
  if (debug_counter++ > 100)
  {
    if (motor_mode != OFF)
      mm.printCurrentRPMCommand();
    switch (motor_mode)
    {
      case START:
        puts("[MAC] motor_mode = START");
        break;
      case OFF:
        puts("[MAC] motor_mode = OFF");
        break;
      case IDLE:
        puts("[MAC] motor_mode = IDLE");
        break;
      case RUNNING:
        puts("[MAC] motor_mode = RUNNING");
        break;
    }

    debug_counter = 0;
  }
#endif
}

float MocapAttitudeController::convertRPMToForce(float rpm)
{
  // Assumes quadratic force model
  // f_i = cT*w_i^2
  return rpm > 0.0f ? cT*rpm*rpm : 0.0f;
}

float MocapAttitudeController::convertForceToRPM(float force)
{
  // Assumes quadratic force model
  // f_i = cT*w_i^2
  float ws = force*cT_inv;
  return ws > 0.0f ? sqrtf(ws) : 0.0f;
}

void MocapAttitudeController::convertBodyForcesToRPM(const body_forces_t& b,
                                                     MotorManager::rpm_cmd_t& out)
{
  // Ensure that forces are at least at the level of the min command
  float fm_base = convertRPMToForce(rpm_min);

  // Lower bound thrust (NED frame!)
  float fbz = fminf(-4.0f*fm_base, b.fbz);

  // Force at each of the motors (NED frame!)
  math::Vector<4> ub(fbz, b.Mb[0], b.Mb[1], b.Mb[2]);
  math::Vector<4> fm = mixer_inv*ub;

  // Convert the forces to RPM commands
  for (unsigned int i = 0; i < 4; i++)
  {
    // Ensure a positive force
    fm(i) = fmaxf(fm(i), 0.0f);
    // Convert the force to RPM and ensure it is in bounds
    out.motor[i] = fminf(fmaxf(convertForceToRPM(fm(i)), rpm_min), rpm_max);
  }

#if 0
  static unsigned int counter = 0;
  if (counter++ > 100)
  {
    puts("fm:");
    fm.print();
    out.print();
    counter = 0;
  }
#endif
}

bool MocapAttitudeController::updateCascadedAttitudeController(MotorManager::rpm_cmd_t& out)
{
  if (!ctrl_state_set)
    return false;

  if (!att_cmd_casc.cmd_set || !att_cmd_casc_gains.gains_set)
  {
#if 0
    static unsigned int counter1 = 0;
    if (counter1++ > 100)
    {
      att_cmd_casc.print();
      att_cmd_casc_gains.print();
      counter1 = 0;
    }
#endif
    return false;
  }

  static uint64_t tcmd_last = 0;
  uint64_t tnow = hrt_absolute_time();
  uint64_t dt_cmd = tnow - tcmd_last;

  // HACK: Throttle the cmd update rate to ensure that it is always
  // less than 500 Hz. If this is removed, function dt
  // can get very small (e.g., < 1e-5).
  if (dt_cmd < 2000)
  {
    out = mm.getRPMCommand();
    return true;
  }

  tcmd_last = tnow;

  math::Matrix<3, 3> R(math::Quaternion(ctrl_state.q).to_dcm());
  math::Matrix<3, 3> Rt(R.transposed());
  math::Matrix<3, 3> Rd(att_cmd_casc.q.to_dcm());

  // orientation error
  math::Vector<3> eR = vee(Rd.transposed()*R - Rt*Rd)*0.5f;

  // angular velocity error
  math::Vector<3> Om(ctrl_state.roll_rate, ctrl_state.pitch_rate, ctrl_state.yaw_rate);
  math::Vector<3> Omd(att_cmd_casc.ang_vel);
  math::Vector<3> eOm = Om - Rt*Rd*Omd;

  // Desired angular acceleration
  math::Vector<3> dOmd(att_cmd_casc.ang_acc);

#if 0
  static unsigned int debug_counter = 0;
  if (debug_counter++ > 100)
  {
    att_cmd_casc.print();
    puts("eR:");
    eR.print();
    puts("eOm:");
    eOm.print();
    debug_counter = 0;
  }
#endif

  math::Vector<3> kR(att_cmd_casc_gains.kR);
  math::Vector<3> kOm(att_cmd_casc_gains.kOm);

  body_forces_t body_cmd;
  body_cmd.fbz = att_cmd_casc.thrust;

  // Notational convenience
  math::Matrix<3,3> J = inertia;

  // torque command - gains scaled by inertia
  math::Vector<3> Mb = J*(-kR.emult(eR) - kOm.emult(eOm) + Rt*Rd*dOmd);
  //+ (Om % (J*Om)) - (Om % (J*Rt*Rd*Omd)); remove - may degrade performance

  // L1 Attitude Observer (provides body-frame moments)
  MotorManager::rpm_cmd_t mm_cmd = mm.getRPMCommand();
  math::Vector<4> current_rpm_cmd;
  for (unsigned int i = 0; i < 4; i++)
    current_rpm_cmd(i) = mm_cmd.motor[i];

  math::Vector<3> l1_Mb = l1_att_observer.update(Om, current_rpm_cmd);

  // Total Moment
  Mb -= l1_Mb;

  for (unsigned int i = 0; i < 3; i++)
    body_cmd.Mb[i] = Mb(i);

  convertBodyForcesToRPM(body_cmd, out);

  return true;
}

void MocapAttitudeController::setControlState(const control_state_s& ctrl_state_)
{
  memcpy(&ctrl_state, &ctrl_state_, sizeof(ctrl_state_));
  ctrl_state_set = true;
}

bool MocapAttitudeController::mocapRPMCommandMessageCallback(const mocap_rpm_command_s& in)
{
  if (!enable_motors)
    return false;

  if (in.ninputs != 4)
    return false;

  for (unsigned int i = 0; i < 4; i++)
    rpm_cmd.motor[i] = (float)in.input[i];

  input_mode = RPM;

  return true;
}

bool MocapAttitudeController::cascadedCommandMessageCallback(const cascaded_command_s& in)
{
  if (!enable_motors)
    return false;

  if (!att_cmd_casc_gains.gains_set)
    return false;

  att_cmd_casc.thrust = in.thrust;
  att_cmd_casc.q.set(in.q);
  att_cmd_casc.ang_vel.set(in.ang_vel);
  att_cmd_casc.ang_acc.set(in.ang_acc);
  att_cmd_casc.cmd_set = true;

  input_mode = CASCADED;

  return true;
}

void MocapAttitudeController::setCascadedAttitudeCommand(const cascaded_attitude_command_t& in)
{
  att_cmd_casc = in;
  input_mode = CASCADED;

  // Publish external input to uORB for logging purposes
  int inst; // Not used
  orb_publish_auto(ORB_ID(cascaded_command), &cascaded_command_pub,
                   &att_cmd_casc, &inst, ORB_PRIO_HIGH);
}

void MocapAttitudeController::cascadedCommandGainsMessageCallback(const cascaded_command_gains_s& in)
{
  for (unsigned int i = 0; i < 3; i++)
    att_cmd_casc_gains.kR[i] = in.kR[i];

  for (unsigned int i = 0; i < 3; i++)
    att_cmd_casc_gains.kOm[i] = in.kOm[i];

  att_cmd_casc_gains.gains_set = true;
}

void MocapAttitudeController::mocapMotorStateMessageCallback(const mocap_motor_state_s& in)
{
  if (in.state == 0)
    enable_motors = false;

  if (in.state == 1)
    enable_motors = true;
}

void MocapAttitudeController::closeSubscriptions()
{
  close(cascaded_cmd_sub);
  close(cascaded_cmd_gains_sub);
  close(mocap_motor_state_sub);
  close(rpm_cmd_sub);
}

void MocapAttitudeController::setMotorMode(const motor_modes_t& m)
{
  motor_mode = m;
}

bool MocapAttitudeController::loadParameters()
{
  cT = pu::getFloatParam("MCC_CT");
  mass = pu::getFloatParam("MCC_TOTAL_MASS");
  gravity_magnitude = pu::getFloatParam("MCC_GRAVITY");

  cmd_ttl_us =
    static_cast<uint64_t>(pu::getFloatParam("MCC_CMD_TTL")*1.0e6f);

  motor_start_dt_us =
    static_cast<uint64_t>(pu::getFloatParam("MCC_MOT_START_DT")*1.0e6f);

  rpm_min = pu::getFloatParam("MCC_RPM_MIN");
  rpm_max = pu::getFloatParam("MCC_RPM_MAX");

  cT_inv = 1.0f/cT;

  float ms = pu::getFloatParam("MCC_MOMENT_SCALE");
  float length = pu::getFloatParam("MCC_ARM_LENGTH");
  float motor_spread_angle = pu::getFloatParam("MCC_MOTOR_SPREAD");
  float dsm = length*sinf(motor_spread_angle);
  float dcm = length*cosf(motor_spread_angle);

  // Mixing matrix
  math::Matrix<4,4> mixer;
  mixer.set_row(0, math::Vector<4>(-1.0f, -1.0f, -1.0f, -1.0f));
  mixer.set_row(1, math::Vector<4>(-dsm, dsm, dsm, -dsm));
  mixer.set_row(2, math::Vector<4>(dcm, -dcm, dcm, -dcm));
  mixer.set_row(3, math::Vector<4>(ms, ms, -ms, -ms));

  // Inverse mixing matrix
  mixer_inv = mixer.inversed();

  float Ixx = pu::getFloatParam("MCC_INERTIA_XX");
  float Ixy = pu::getFloatParam("MCC_INERTIA_XY");
  float Ixz = pu::getFloatParam("MCC_INERTIA_XZ");
  float Iyy = pu::getFloatParam("MCC_INERTIA_YY");
  float Iyz = pu::getFloatParam("MCC_INERTIA_YZ");
  float Izz = pu::getFloatParam("MCC_INERTIA_ZZ");

  inertia(0,0) = Ixx; inertia(0,1) = Ixy; inertia(0,2) = Ixz;
  inertia(1,0) = Ixy; inertia(1,1) = Iyy; inertia(1,2) = Iyz;
  inertia(2,0) = Ixz; inertia(2,1) = Iyz; inertia(2,2) = Izz;

  return true;
}

bool MocapAttitudeController::registerCallbacks()
{
  cascaded_cmd_sub = orb_subscribe(ORB_ID(cascaded_command));
  if (cascaded_cmd_sub < 0)
  {
    puts("cascaded_cmd_sub failed");
    return false;
  }

  cascaded_cmd_gains_sub = orb_subscribe(ORB_ID(cascaded_command_gains));
  if (cascaded_cmd_gains_sub < 0)
  {
    puts("cascaded_cmd_gains_sub failed");
    return false;
  }

  mocap_motor_state_sub = orb_subscribe(ORB_ID(mocap_motor_state));
  if (mocap_motor_state_sub < 0)
  {
    puts("mocap_motor_state_sub failed");
    return false;
  }

  rpm_cmd_sub = orb_subscribe(ORB_ID(mocap_rpm_command));
  if (rpm_cmd_sub < 0)
  {
    puts("rpm_cmd_sub failed");
    return false;
  }

  return true;
}
