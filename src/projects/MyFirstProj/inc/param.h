/* Based off of $PX4_FIRMWARE/src/examples/fixedwing_control/params.h
 * by Lorenz Meier
 */

/* 
 * Definition of parameters for rotorcraft
 */

#include <systemlib/param/param.h>

struct params {
  float roll_p;
  float pitch_p;
  float yaw_rate_p;
};

struct param_handles {
  param_t roll_p;
  param_t pitch_p;
  param_t yaw_rate_p;
};

/*
 * Initialize all parameter handles and values
 */
int parameters_init(struct param_handles *h);

/*
 * Update all parameters
 */
int parameters_update(const struct param_handles *h, struct params *p);
