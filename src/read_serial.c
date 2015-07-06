#include "../include/read_serial.h"
#include "../include/decode_packet.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#define SERIAL_PORT "/dev/ttyAMA0"

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t scan_ready = PTHREAD_COND_INITIALIZER;

void *read_serial(void * arg)
{
  int serial_fd = open(SERIAL_PORT, O_RDONLY);

  struct read_serial_out * out = (struct read_serial_out *)arg;

  int16_t * ranges = out->ranges;

  double * points = out->points;

  char buff[22];
  char packet[22];

  int read_ahead = 1;

  int packet_index = 0;

  while(serial_fd)
  {
    memset(buff, 0, sizeof(buff));
    //printf("%d, %d\n", packet_index, read_ahead);
    int count = read(serial_fd, buff, read_ahead);

    read_ahead -= count;

    memcpy(&packet[packet_index], buff, count);

    int z;
    for (z = 0; z < count; z++)
    {
      //printf("%02x", buff[z]);
    }
    //printf("\n");
    for (z = 0; z < 22; z++)
    {
      //printf("%02x", packet[z]);
    }
    //printf("\n");
    //printf("%d, %d, %d\n", packet_index, read_ahead, count);

    packet_index += count;

    if (buff[0] == 0xfa)
    {
      read_ahead = 22 - count;
      packet_index = count;
    }

    if (packet_index == 22)
    {
      int j;
      for (j = 0; j < 22; j++)
      {
        //printf("%02x", packet[j]);
      }
      if (packet[1] == 0xf9)
      {
        pthread_mutex_lock(&mutex);
        pthread_cond_signal(&scan_ready);
        pthread_mutex_unlock(&mutex);
      }
      //printf("\n");
      pthread_mutex_lock(&mutex);
      decode_packet(ranges, points, out->map, packet);

      if (packet[1] == 0xf9)
      {
        pthread_cond_signal(&scan_ready);
      }


      pthread_mutex_unlock(&mutex);
      read_ahead = 22;
      packet_index = 0;
    }

    if (read_ahead == 0)
    {
      read_ahead = 1;
    //  packet_index = 0;
    }
  }

  printf("failed to open serial port\n");  

  return NULL;
}
