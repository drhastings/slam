#pragma once

#include <stdint.h>
#include "matrices.h"
#include <pthread.h>

pthread_mutex_t mutex;

pthread_cond_t scan_ready;

struct read_serial_out
{
  int16_t ranges[360];
  double points[360 * 2];
  struct mat ** map;
  pthread_mutex_t * mutex;
  pthread_cond_t * scan_ready;;
};

void *read_serial();
