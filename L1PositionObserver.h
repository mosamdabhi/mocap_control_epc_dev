#ifndef L1_POSITION_OBSERVER_H
#define L1_POSITION_OBSERVER_H

#include <cstdio>
#include <mathlib/mathlib.h>

#include "ParameterUtils.h"

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/l1_linvel_debug.h>

class L1PositionObserver
{
public:
  L1PositionObserver()
  {
    t_prev = 0;
    cmd_ttl_us = 0;
    mass = 0.0f;
    gravity_magnitude = 0.0f;
    cT = 0.0f;
    l1_linvel_debug_pub = nullptr;

    enable_l1 = false;
    use_active_l1ac = false;
    in_nominal_flight = false;
    l1_initialized = false;
    enable_debug = false;

    motor_constant = 0.0f;
    l1_engage_level = 0.0f;

    bandwidth.zero();
    adaptation_gain.zero();
    observer_gain.zero();
    init_dist.zero();

    velhat.zero();
    dsthat.zero();
    lpd.zero();
    rpmhat.zero();
  }

  void loadParameters()
  {
    mass = parameter_utils::getFloatParam("MCC_TOTAL_MASS");
    gravity_magnitude = parameter_utils::getFloatParam("MCC_GRAVITY");
    cT = parameter_utils::getFloatParam("MCC_CT");
    cmd_ttl_us =
      static_cast<uint64_t>(parameter_utils::getFloatParam("MCC_CMD_TTL")*1.0e6f);

    enable_l1 = static_cast<bool>(parameter_utils::getIntParam("MCC_ENABLE_L1P"));
    if (enable_l1)
      puts("[MPC] L1 position is enabled!");

    motor_constant = parameter_utils::getFloatParam("MCC_MOTOR_CONST");

    bandwidth(0) = parameter_utils::getFloatParam("L1P_BW_X");
    bandwidth(1) = parameter_utils::getFloatParam("L1P_BW_Y");
    bandwidth(2) = parameter_utils::getFloatParam("L1P_BW_Z");

    observer_gain(0) = parameter_utils::getFloatParam("L1P_OBS_GAIN_X");
    observer_gain(1) = parameter_utils::getFloatParam("L1P_OBS_GAIN_Y");
    observer_gain(2) = parameter_utils::getFloatParam("L1P_OBS_GAIN_Z");

    adaptation_gain(0) = parameter_utils::getFloatParam("L1P_ADAPT_GAIN_X");
    adaptation_gain(1) = parameter_utils::getFloatParam("L1P_ADAPT_GAIN_Y");
    adaptation_gain(2) = parameter_utils::getFloatParam("L1P_ADAPT_GAIN_Z");

    init_dist(0) = parameter_utils::getFloatParam("L1P_INIT_DIST_X");
    init_dist(1) = parameter_utils::getFloatParam("L1P_INIT_DIST_Y");
    init_dist(2) = parameter_utils::getFloatParam("L1P_INIT_DIST_Z");

    l1_engage_level = parameter_utils::getFloatParam("L1P_ENGAGE_LEVEL");

    use_active_l1ac = static_cast<bool>(parameter_utils::getIntParam("L1P_ACTIVE"));
    enable_debug = static_cast<bool>(parameter_utils::getIntParam("L1P_ENABLE_DEBUG"));
  }

  math::Vector<3> update(const math::Matrix<3,3>& Rwb,
                         const math::Vector<3>& vel,
                         const math::Vector<4>& rpm_cmd)
  {
    hrt_abstime t = hrt_absolute_time();
    hrt_abstime dt_abs = t_prev != 0 ? (t - t_prev) : 0;
    float dt = static_cast<float>(dt_abs)*0.000001f;
    t_prev = t;

    if (enable_l1)
    {
      if (!l1_initialized || (dt_abs > cmd_ttl_us))
        initialize(vel, rpm_cmd);

      float total_force = 0.0f;
      for (unsigned int i = 0; i < 4; i++)
        total_force += convertRPMToForce(rpm_cmd(i));

      if(total_force < l1_engage_level*mass*gravity_magnitude)
        initialize(vel, rpm_cmd);
      else
        in_nominal_flight = true;

      /* Run Luenberger observer */
      if (in_nominal_flight)
      {
        math::Vector<3> verr = velhat - vel;

        math::Vector<3> body_acc;
        body_acc.zero();
        body_acc(2) = -(rpmhat*rpmhat*cT)/mass;

        math::Vector<3> velhatdot = Rwb*body_acc + dsthat - observer_gain.emult(verr);
        velhatdot(2) += gravity_magnitude;

        math::Vector<4> rpmhatdot = (rpm_cmd - rpmhat)*motor_constant;
        math::Vector<3> dsthatdot = -adaptation_gain.emult(verr);

        velhat += velhatdot*dt;
        rpmhat += rpmhatdot*dt;
        dsthat += dsthatdot*dt;
        lpd += bandwidth.emult(dsthat - lpd)*dt;

        if (enable_debug)
        {
          /* publish debug data*/
          l1_linvel_debug.timestamp = hrt_absolute_time();
          l1_linvel_debug.vel[0]  = velhat(0);
          l1_linvel_debug.vel[1]  = velhat(1);
          l1_linvel_debug.vel[2]  = velhat(2);
          l1_linvel_debug.rpm[0]  = rpmhat(0);
          l1_linvel_debug.rpm[1]  = rpmhat(1);
          l1_linvel_debug.rpm[2]  = rpmhat(2);
          l1_linvel_debug.rpm[3]  = rpmhat(3);
          l1_linvel_debug.dst[0]  = dsthat(0);
          l1_linvel_debug.dst[1]  = dsthat(1);
          l1_linvel_debug.dst[2]  = dsthat(2);
          l1_linvel_debug.lpd[0]  = lpd(0);
          l1_linvel_debug.lpd[1]  = lpd(1);
          l1_linvel_debug.lpd[2]  = lpd(2);

          int inst; // Not used
          orb_publish_auto(ORB_ID(l1_linvel_debug), &l1_linvel_debug_pub,
                           &l1_linvel_debug, &inst, ORB_PRIO_HIGH);
        }

        if (use_active_l1ac)
          return lpd*mass;
      }
    }

    math::Vector<3> zero; zero.zero();
    return zero;
  }

private:
  float convertRPMToForce(float rpm)
  {
    // Assumes quadratic force model
    // f_i = cT*w_i^2
    return rpm > 0.0f ? cT*rpm*rpm : 0.0f;
  }

  void initialize(const math::Vector<3>& vel, const math::Vector<4>& rpm_cmd)
  {
    in_nominal_flight = false;
    velhat = vel;
    dsthat = init_dist;
    lpd = dsthat;
    rpmhat = rpm_cmd;
    t_prev = 0;
    l1_initialized = true;
  }

  hrt_abstime t_prev;
  hrt_abstime cmd_ttl_us;

  float mass;
  float gravity_magnitude;
  float cT;

  bool enable_l1;
  bool use_active_l1ac;
  bool in_nominal_flight;
  bool l1_initialized;
  bool enable_debug;

  float motor_constant;
  float l1_engage_level;

  math::Vector<3> bandwidth;
  math::Vector<3> observer_gain;
  math::Vector<3> adaptation_gain;
  math::Vector<3> init_dist;

  math::Vector<3> velhat;
  math::Vector<3> dsthat;
  math::Vector<3> lpd;
  math::Vector<4> rpmhat;

  orb_advert_t l1_linvel_debug_pub;
  struct l1_linvel_debug_s l1_linvel_debug;
};

#endif
