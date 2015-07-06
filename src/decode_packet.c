#include "../include/decode_packet.h"
#include "../include/matrices.h"
#include <stdint.h>
#include <pigpio.h>
#include <pthread.h>

float lasterror, ierror = 88000;

float p = 1;
float D = 0;
float I = .1;

int pwm_handle;

int decode_packet(int16_t ranges[360], double readings[360 * 2], 
          struct mat ** map, char packet[22]) 
{ 
  int i; 
 
  uint32_t sum = 0; 

  struct mat * local_map = *map;
 
  for (i = 0; i < 10; i++) 
  { 
    uint16_t chunk = (uint16_t)(packet[i * 2] + (packet[(i * 2) + 1] << 8)); 
    sum = (sum << 1) + chunk; 
  } 
 
  uint16_t checksum = (uint16_t)((sum & 0x7fff) + ((sum >> 15) & 0x7fff)); 
 
  uint16_t speed = (uint16_t)(packet[2] + (packet[3] << 8)); 
 
  float rpm = speed / 64.0; 

  float guess_x = MGET(local_map, 0, 0);
  float guess_y = MGET(local_map, 1, 0);
  //float guess_theta = last_packet_theta2;
  float guess_theta = MGET(local_map, 2, 0);

  //printf("%f, %f, %f\n", rotation, elapsed, last_time);

  if (checksum == (uint16_t)(packet[20] + (packet[21] << 8))) 
  { 
    //printf("%f\n", rpm);
    for (i = 0; i < 4; i ++) 
    { 
      int this_index = (((packet[1] - 0xa0) * 4) + i - 2) % 360;

      if (this_index < 0)
      {
        this_index = 360 + this_index;
      }
      ranges[this_index] = (uint16_t)(packet[(i + 1) * 4] + ((packet[((i + 1) * 4) + 1]) << 8)); 
      if (ranges[this_index] & 0xc000)
      {
        ranges[this_index] = 0;
      }
      ranges[this_index] &= 0x3fff;
      if (ranges[this_index] > 0)
      {
        readings[(this_index) * 2] = cosf(guess_theta + ((this_index) * M_PI) / 180) * ranges[this_index];
        readings[(this_index * 2) + 1] = sinf(guess_theta + ((this_index) * M_PI) / 180) * ranges[this_index];
        readings[(this_index) * 2] += guess_x;
        readings[((this_index) * 2) + 1] += guess_y;
      }
      else
      {
        readings[this_index * 2] = 0;
        readings[(this_index * 2) + 1] = 0;
      }
      float error = 300 - rpm; 
 
      int power = error * p; 
 
      power += ierror * I; 
 
      lasterror = error; 
 
      ierror += error * 0.01; 

      gpioPWM(17, power);


    } 
  }

  return 0;
}
