#include "measurements.h"

typedef struct state_s {
	double pos_m,
	double vel_ms
} state_t;

void kf_init();

void kf_meas_vel(meas_velocity_t meas);

void kf_meas_pos(meas_position_t meas);

