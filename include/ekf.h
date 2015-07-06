#pragma once

#include "to_frame.h"

void ekf_init();
void ekf_step(struct mat ** map, struct mat * U, double * sightings, int count);
void test();
int get_closest(struct mat ** map, double x, double y);
