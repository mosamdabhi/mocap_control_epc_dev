#ifndef CASCADED_ATTITUDE_COMMAND_H
#define CASCADED_ATTITUDE_COMMAND_H

#include <cstdio>
#include <mathlib/mathlib.h>

typedef struct CascadedAttitudeCommand
{
  float thrust;
  math::Quaternion q;
  math::Vector<3> ang_vel;
  math::Vector<3> ang_acc;

  bool cmd_set;

  CascadedAttitudeCommand() :
    thrust(0.0f),
    cmd_set(false)
  {
    q.zero(); q(0) = 1.0f;
    ang_vel.zero();
    ang_acc.zero();
  }

  void print()
  {
    char buf[128];
    puts("casc att cmd:");
    sprintf(buf, "T = %0.2f", double(thrust));
    printf("\t%s\n", buf);
    sprintf(buf, "Quat = [%0.2f, %0.2f, %0.2f, %0.2f]",
            double(q(0)), double(q(1)), double(q(2)), double(q(3)));
    printf("\t%s\n", buf);
    sprintf(buf, "Ang Vel= [%0.2f, %0.2f, %0.2f]",
            double(ang_vel(0)), double(ang_vel(1)), double(ang_vel(2)));
    printf("\t%s\n", buf);
    sprintf(buf, "Ang Acc= [%0.2f, %0.2f, %0.2f]",
            double(ang_acc(0)), double(ang_acc(1)), double(ang_acc(2)));
    printf("\t%s\n", buf);
    printf("\tcmd_set = %s\n", cmd_set ? "true" : "false");
  }
} cascaded_attitude_command_t;

#endif
