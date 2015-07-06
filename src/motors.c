#include "../include/motors.h"
#include <pigpio.h>

int teensy;

#define JOYSTICK_FILE "/dev/input/js0"

union speed
{
  float floats[2];
  char chars[8];
} speeds;

void init_motors()
{
  teensy = i2cOpen(1, 0x44, 0);
}

void send_speeds(int16_t f_a, int16_t s_a)
{
  float forward = f_a / 32767.0 * 100;
  float spin = s_a / 32767.0 * 50;

  speeds.floats[0] = forward;
  speeds.floats[1] = forward;

  speeds.floats[0] += spin;
  speeds.floats[1] -= spin;

  i2cWriteI2CBlockData(teensy, 0, speeds.chars, 8);
}

void send_speeds_percent(float speed, float spin)
{
  float max_speed = 20.0;
  
  if (speed > max_speed)
  {
    speed = max_speed;
  }
  if (speed < -max_speed)
  {
    speed = -max_speed;
  }
  if (spin > max_speed / 2)
  {
    spin = max_speed;
  }
  if (spin < -max_speed / 2)
  {
    spin = -max_speed;
  }

  speeds.floats[0] = speed;
  speeds.floats[1] = speed;

  speeds.floats[0] += spin;
  speeds.floats[1] -= spin;

  i2cWriteI2CBlockData(teensy, 0, speeds.chars, 8);
}
