#include "pid.h"
#include <string.h>

static float limit(float x, float max)
{
    if (max <= 0.0f) return x;
    if (x > max)  return max;
    if (x < -max) return -max;
    return x;
}

void pid_init(pid_t *pid, const pid_param_t *param)
{
    memset(pid, 0, sizeof(*pid));
    pid->p = *param;
}

void pid_reset(pid_t *pid)
{
    pid->err = pid->last_err = 0.0f;
    pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}

float pid_calc(pid_t *pid, float set, float get)
{
    pid->set = set;
    pid->get = get;
    pid->last_err = pid->err;
    pid->err = set - get;
    if (pid->err < pid->p.deadband && pid->err > -pid->p.deadband) pid->err = 0.0f;

    pid->pout  = pid->p.kp * pid->err;
    pid->iout  = limit(pid->iout + pid->p.ki * pid->err, pid->p.max_iout);
    pid->dout  = limit(pid->p.kd * (pid->err - pid->last_err), pid->p.max_dout);
    pid->out   = limit(pid->pout + pid->iout + pid->dout, pid->p.max_out);
    return pid->out;
}
