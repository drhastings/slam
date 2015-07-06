#include "../include/read_sensors.h"
#include "../include/joy.h"
#include "../include/motors.h"
#include "../include/quaternion.h"
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

int32_t left_count = 0;
int32_t right_count = 0;
int32_t last_left = 0;
int32_t last_right = 0;

double mpu_heading = 0;
double mpu_heading_offset = 0;
double encoder_heading = 0;

extern int16_t spin_axis, forward_axis;

double x_pos_encoders, y_pos_encoders;
double x_pos_mpu, y_pos_mpu;

int first = 1;

void *sensor_read_function()
{
  int mpu_file = open("/dev/encoders", O_RDONLY);

  char data[62];

  memset(data, 0, sizeof(data));

  while(1)
  {
    read(mpu_file, data, 62);

    int16_t temp[4];

    temp[0] = ((data[0] << 8) + data[1]);
    temp[1] = ((data[4] << 8) + data[5]);
    temp[2] = ((data[8] << 8) + data[9]);
    temp[3] = ((data[12] << 8) + data[13]);

    float w = temp[0] / 16384.0f;
    float xx = temp[1] / 16384.0f;
    float y = (float)temp[2] / 16384.0f;
    float z = (float)temp[3] / 16384.0f;

    struct quaternion orientation;

    orientation.w = w;
    orientation.x = xx;
    orientation.y = y;
    orientation.z = z;

    struct vect forward, projection;

    forward.x = 1;
    forward.y = 0;
    forward.z = 0;

    struct quaternion local = orientation;      

    rotate_vector(&projection, &forward, &local);

    projection.z = 0;

    float normal = norm(&projection);

    projection.x = projection.x / normal;
    projection.y = projection.y / normal;

    mpu_heading = atan2(projection.y, projection.x) - mpu_heading_offset;

    //printf("%f\n", mpu_heading);

    left_count = data[46];

    left_count += data[47] << 8;
    left_count += data[48] << 16;
    left_count += data[49] << 24;

    right_count = data[42];

    right_count += data[43] << 8;
    right_count += data[44] << 16;
    right_count += data[45] << 24;


    /*unsigned int sonar_range_buffer[3];

    sonar_range_buffer[2] = data[50];
    sonar_range_buffer[2] += data[51] << 8;
    sonar_range_buffer[2] += data[52] << 16;
    sonar_range_buffer[2] += data[53] << 24;

    sonar_range_buffer[0] = data[54];
    sonar_range_buffer[0] += data[55] << 8;
    sonar_range_buffer[0] += data[56] << 16;
    sonar_range_buffer[0] += data[57] << 24;

    sonar_range_buffer[1] = data[58];
    sonar_range_buffer[1] += data[59] << 8;
    sonar_range_buffer[1] += data[60] << 16;
    sonar_range_buffer[1] += data[61] << 24;

    sonar_ranges[2] = sonar_range_buffer[0] / 5.7;
    sonar_ranges[1] = sonar_range_buffer[1] / 5.7;
    sonar_ranges[0] = sonar_range_buffer[2] / 5.7;
*/

    if (first)
    {
      last_left = left_count;
      last_right = right_count;

      mpu_heading_offset = mpu_heading;

      first = 0;
    }

    float c = 0.02004;

    float sl = (left_count - last_left) * c;
    float sr = (right_count - last_right) * c;

    float s = (sl + sr) / 2;

    //dX += s;

    double theta_delta_wheels = ((sr - sl) / 317.0);

    encoder_heading += theta_delta_wheels;

    //theta_delta = mpu_heading - last_mpu_heading;

    while (mpu_heading > M_PI)
    {
      mpu_heading -= 2 * M_PI;
    }

    while (mpu_heading < -M_PI)
    {
      mpu_heading += 2 * M_PI;
    }

    /*if (fabs(theta_delta) > .016)
    {
      printf("using wheels, %f\n", theta_delta);
      theta_delta = theta_delta_wheels;
      close(mpu_file);
      mpu_file = open("/dev/encoders", O_RDONLY);

    }
    if (0)//fabs(theta_delta_wheels) < 0.000001)
    {
      theta_delta = 0;

      //printf("setting to 0\n");
    }

    last_mpu_heading = mpu_heading;

    theta += theta_delta;

    dA += theta_delta;
    printf("%f\n", mpu_heading);

    //printf("%f\n\n", theta_delta);
*/
    x_pos_encoders += s * cosf(encoder_heading);
    y_pos_encoders += s * sinf(encoder_heading);

    x_pos_mpu += s * cosf(mpu_heading);
    y_pos_mpu += s * sinf(mpu_heading);
/*
    x_delta = s * cosf(MGET(map, 0, 2));
    y_delta = s * sinf(MGET(map, 0, 2));

    MSET(thisx, 0, 0, MGET(thisx, 0, 0) + x_delta);
    MSET(thisx, 1, 0, MGET(thisx, 1, 0) + y_delta);
    MSET(thisx, 2, 0, MGET(thisx, 2, 0) + theta_delta);

    pthread_mutex_lock(&mutex);
    this_x = this_x + x_delta;
    this_y = this_y + y_delta;
    //printf("%f, %f\n", mpu_heading, theta_delta);
    //printf("smaple\n");
    this_theta = this_theta + theta_delta;

    rotation = theta_delta / (time - last_time);

    pthread_mutex_unlock(&mutex);

    if (abs(left_count - last_left) > 100 || abs(right_count - last_right) > 100)
    {
      //printf("%d, %d\n", left_count, right_count);
    }

    */

    last_left = left_count;
    last_right = right_count;
/*
    if (wandering)
    {
      wander();
    }
    else
    {
      if (!canIgo() && forward_axis > 0)
      {
        forward_axis = 0;
      }
      send_speeds(forward_axis, spin_axis);
    }

    point_y += point_move_y;
    point_x += point_move_x;

    gpioServo(7, point_y);
    gpioServo(8, point_x);

    //printf("%d, %d\n", point_x, point_y);


    //printf("[%f, %f, %f, %f]\n", w, x, y, z);
    //printf("[%d, %d]\n", left_count, right_count);
    //printf("[%f]\n", mpu_heading);

//    printf("[%f, %f, %f, %f, %f, %f]\n", x_pos, y_pos, theta, mpu_heading, offset, correction_heading);*/
    
    send_speeds(forward_axis, spin_axis);
  }
}

int get_right_count()
{
  return right_count;
}

int get_left_count()
{
  return left_count;
}

double get_mpu_heading()
{
  return mpu_heading;
}

double get_encoder_heading()
{
  return encoder_heading;
}

double get_encoder_x()
{
  return x_pos_encoders;
}

double get_encoder_y()
{
  return y_pos_encoders;
}

double get_mpu_x()
{
  return x_pos_mpu;
}

double get_mpu_y()
{
  return y_pos_mpu;
}
