#include <fenv.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdlib.h>
#include <semaphore.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/mman.h>
#include "include/matrices.h"
#include <pigpio.h>
#include "include/ekf.h"
#include "include/viz.h"
#include "include/hough.h"
#include "include/read_serial.h"
#include "include/read_sensors.h"
#include "include/joy.h"

#define SERIAL_PORT "/dev/ttyAMA0"

extern pthread_mutex_t mutex;

extern pthread_cond_t scan_ready;

#define WRITE_FILE "/tmp/scans"

struct read_serial_out *serial_out; 
struct hough_transformer transform;
struct hough_transformer *pTrans = &transform;

int main()
{
  struct mat ** map = malloc(sizeof(struct mat *));
  *map = mat(3, 1);  
  struct mat * pos = mat(3, 1);  
  struct mat * last_pos = mat(3, 1);  

  struct mat * here = mat(3, 1);

  serial_out = malloc(sizeof(struct read_serial_out));

  serial_out->map = map;

  serial_out->mutex = &mutex;
  serial_out->scan_ready = &scan_ready;

  memset(&transform, 0, sizeof(struct hough_transformer));

  init_hough(720, 800, &transform);

  gpioCfgClock(1, 0, 0);
  gpioInitialise();

  gpioSetMode(17, PI_OUTPUT);
  gpioSetPWMrange(17, 10000);
  gpioSetPWMfrequency(17, 40000);

  gpioPWM(17, 9500);

  sleep(1);

  ekf_init();

  pthread_t serial_thread;
  pthread_t sensor_thread;
  pthread_t joy_thread;
  pthread_t viz_thread;

  pthread_create(&serial_thread, NULL, read_serial, (void *) serial_out);
  pthread_create(&sensor_thread, NULL, sensor_read_function, NULL);
  pthread_create(&joy_thread, NULL, joy, NULL);
  pthread_create(&viz_thread, NULL, viz_write, map);

  while(1)
  {
    pthread_mutex_lock(&mutex);

    pthread_cond_wait(&scan_ready, &mutex);

    int i;

    int count = 0;

    double hough_points[360 * 2];

    memset(hough_points, 0, sizeof(hough_points));

    for (i = 0; i < 360; i++)
    {
      if (serial_out->ranges[i])
      {
        hough_points[count * 2] = serial_out->points[i * 2];
        hough_points[count * 2 + 1] = serial_out->points[i * 2 + 1];
        //printf("%f, %f\n", hough_points[count * 2], hough_points[count * 2 +1]);
        count++;
      }
    }

    pthread_mutex_unlock(&mutex);

    

    hough(hough_points, count, &transform);

    for (i = 0; i < transform.count; i++)
    {
      //printf("%f,\t%f\n", transform.results[i * 2], transform.results[(i * 2) + 1]);
    }
    //printf("I saw %d lines\n", transform.count);

    //printf("%f,\t%f\n", get_encoder_x(), get_encoder_y());
    //printf("%f,\t%f\n", get_mpu_x(), get_mpu_y());
    //printf("%f,\t%f\n", get_encoder_heading(), get_mpu_heading());

    MSET(pos, 0, 0, get_mpu_x());
    MSET(pos, 1, 0, get_mpu_y());
    MSET(pos, 2, 0, get_mpu_heading());

    struct mat * U = mat(2, 1);

    double x_dif = MGET(pos, 0, 0) - MGET(last_pos, 0, 0);
    double y_dif = MGET(pos, 1, 0) - MGET(last_pos, 1, 0);

    double dist = sqrt((x_dif * x_dif) + (y_dif * y_dif));

    MSET(U, 0, 0, dist);
    MSET(U, 1, 0, MGET(pos, 2, 0) - MGET(last_pos, 2, 0));
    ekf_step(map, U, transform.results, transform.count);

    sub_mat(here, *map, 0, 0);

    print_mat(here);
    print_mat(pos);

    copy_into(last_pos, pos, 0, 0);

    printf("\n");
  }
  
  printf("terminated\n");
  gpioTerminate();

  return 0;
}
