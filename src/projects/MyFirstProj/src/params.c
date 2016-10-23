/* Based off of $PX4_FIRMWARE/src/examples/fixedwing_control/params.h
 * by Lorenz Meier
 */

#include "params.h"

PARAM_DEFINE_FLOAT(MYPR_ROLL_P, 0.2f);
PARAM_DEFINE_FLOAT(MYPR_PITCH_P, 0.2f);
PARAM_DEFINE_FLOAT(MYPR_YAW_RATE_P, 0.2f);

int parameters_init(struct param_handles *h) {
  h->roll_p = param_find("MYPR_ROLL_P");
  h->pitch_p = param_find("MYPR_PITCH_P");
  h->yaw_rate_p = param_find("MYPR_YAW_RATE_P");

  return OK;
}

int parameters_update(const struct param_handles *h, struct params *p) {
  param_get(h->roll_p, &(p->roll_p));
  param_get(h->pitch_p, &(p->pitch_p));
  param_get(h->yaw_rate_p, &(p->yaw_rate_p));

  return OK;
}
