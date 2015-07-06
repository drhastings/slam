#include "../include/joy.h"
#include "../include/motors.h"
#include <linux/joystick.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define JOYSTICK_FILE "/dev/input/js0"

void *joy()
{
  int jfd;

  init_motors();

  if ((jfd = open(JOYSTICK_FILE, O_RDONLY)) == -1)
  {
    printf("Press the PS button\n");
    //exit(1);
  }

  int keep_going = 1;

  while(keep_going)
  {
    struct js_event e;
    read(jfd, &e, sizeof(e));

    if (e.type == 2 && e.number == 1)
    {
      forward_axis = -e.value;
    }
    if (e.type == 2 && e.number == 0)
    {
      spin_axis = e.value;
    }
  }

  return NULL;
}
