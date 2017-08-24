#ifndef MOCAP_ATTITUDE_CONTROL
#define MOCAP_ATTITUDE_CONTROL
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
#include <cstdio>
#include <poll.h>
#include <px4_posix.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/cascaded_command.h>
#include <uORB/topics/cascaded_command_gains.h>
#include <uORB/topics/mocap_motor_state.h>
#include <uORB/topics/mocap_rpm_command.h>
#include <uORB/topics/control_state.h>

#include "ParameterUtils.h"
#include "MotorManager.h"
#include "CascadedAttitudeCommand.h"
#include "L1AttitudeObserver.h"

class MocapAttitudeController
{
public:
  MocapAttitudeController();
  ~MocapAttitudeController();

  bool initialize();
  void finalize();
  void update(bool cmd_updated_ext);

  void setCascadedAttitudeCommand(const cascaded_attitude_command_t& in);
  void setControlState(const control_state_s& ctrl_state);

  void getCurrentRPMCommand(MotorManager::rpm_cmd_t& out)
  {
    out = mm.getRPMCommand();
  }

private:
  typedef struct BodyForcesCommand
  {
    float fbz;
    float Mb[3];

    BodyForcesCommand() : fbz(0.0f)
    {
      memset(Mb, 0, sizeof(Mb));
    }

    void print()
    {
      printf("body cmd = fz=%0.5f, Mx=%0.5f, My=%0.5f, Mz=%0.5f\n",
            double(fbz), double(Mb[0]), double(Mb[1]), double(Mb[2]));
    }
  } body_forces_t;

  typedef enum
  {
    START = 0,
    OFF,
    IDLE,
    RUNNING
  } motor_modes_t;

  typedef enum
  {
    RPM = 0,
    CASCADED
  } input_modes_t;

  typedef struct CascadedAttitudeCommandGains
  {
    float kR[3];
    float kOm[3];

    bool gains_set;

    CascadedAttitudeCommandGains() : gains_set(false)
    {
      memset(kR, 0, sizeof(kR));
      memset(kOm, 0, sizeof(kOm));
    }

    void print()
    {
      printf("casc att cmd gains:\n"
             "\tKR gains = [%0.2f, %0.2f, %0.2f]\n"
             "\tKOm gains = [%0.2f, %0.2f, %0.2f]\n",
             double(kR[0]), double(kR[1]), double(kR[2]),
             double(kOm[0]), double(kOm[1]), double(kOm[2]));
      printf("\tgains_set = %s\n", gains_set ? "true" : "false");
    }
  } cascaded_attitude_command_gains_t;

  // Start-up/cleanup calls
  bool loadParameters();
  bool registerCallbacks();
  void closeSubscriptions();

  // RPM <-> Force
  float convertRPMToForce(float rpm);
  float convertForceToRPM(float force);

  // Convert all inputs, RPM <-> Force
  void convertBodyForcesToRPM(const body_forces_t& b,
                              MotorManager::rpm_cmd_t& cmd);

  // Controller w.r.t. cascaded attitude command inputs
  bool updateCascadedAttitudeController(MotorManager::rpm_cmd_t& cmd);

  // Input Callbacks
  bool mocapRPMCommandMessageCallback(const mocap_rpm_command_s& cmd);
  bool cascadedCommandMessageCallback(const cascaded_command_s& cmd);
  void cascadedCommandGainsMessageCallback(const cascaded_command_gains_s& in);
  void mocapMotorStateMessageCallback(const mocap_motor_state_s& in);

  void setMotorMode(const motor_modes_t& m);
  void resetDesiredSetPoints();

  math::Vector<3> vee(const math::Matrix<3,3>& in) const
  {
    return math::Vector<3>(in(2, 1), in(0, 2), in(1, 0));
  }

  math::Matrix<3,3> hat(const math::Vector<3>& in) const
  {
    float tmp[9] = {0.0f, -in(2), in(1),
                    in(2), 0.0f, -in(0),
                    -in(1), in(0), 0.0f};

    return math::Matrix<3,3>(tmp);
  }

  float unroll(float x)
  {
    x = fmodf(x, 2.0f*M_PI_F);
    if (x < 0.0f)
      x += 2.0f*M_PI_F;
    return x;
  }

  float normalize(float x)
  {
    x = fmodf(x + M_PI_F, 2.0f*M_PI_F);
    if (x < 0.0f)
      x += 2.0f*M_PI_F;
    return x - M_PI_F;
  }

  float shortest_angular_distance(float from, float to)
  {
    float result = unroll(unroll(to) - unroll(from));
    if (result > M_PI_F)
      result = -(2.0f*M_PI_F - result);
    return normalize(result);
  }

  // Model parameters
  float motor_model[3];
  float rpm_per_pwm;
  float rpm_v_zero;
  float gravity_magnitude;
  float mass;

  bool enable_motors;

  uint64_t cmd_time;
  bool ctrl_state_set;

  int cascaded_cmd_sub;
  int cascaded_cmd_gains_sub;
  int mocap_motor_state_sub;
  int rpm_cmd_sub;

  struct control_state_s ctrl_state;
  cascaded_attitude_command_t att_cmd_casc, att_cmd_zero;
  cascaded_attitude_command_gains_t att_cmd_casc_gains;

  motor_modes_t motor_mode;
  input_modes_t input_mode;

  math::Matrix<3,3> inertia;
  math::Matrix<4,4> mixer_inv;

  bool motor_start_set;
  uint64_t motor_start_dt_us;
  uint64_t tmotor_start;

  MotorManager mm;
  float rpm_min, rpm_max;
  MotorManager::rpm_cmd_t rpm_cmd;

  uint64_t cmd_ttl_us;

  L1AttitudeObserver l1_att_observer;

  orb_advert_t cascaded_command_pub;

  float sample_freq;
  float cutoff_freq_x, cutoff_freq_y, cutoff_freq_z;
  math::LowPassFilter2p Mb_lp_x, Mb_lp_y, Mb_lp_z;
};
#endif
