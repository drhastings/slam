#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "../include/ransac.h"

#define NUMSAMPLES 10

int ransac(float landmarks[][2], int maxreturn, float *readings, int count, float x, float y)
{
  srandom(time(NULL));

  char available[360];

  int availableCount = count;

  int iterationsMax = 1200;
  int iterations = 0;

  int consensus = 25;

  int degreesOff = 12;

  int numSamples = 4;

  float distThreshold = 10;

  int outLines = 0;

  int fconsensus = 250;

  memset(available, 0, sizeof(available));

  while (availableCount > consensus && iterations < iterationsMax)
  {
    int sample = random() % 360;

    //printf("I am looking for a line in %i points, index %i, iteration %i\n", availableCount, sample, iterations);

    while (available[sample])
    {
      sample = random() % 360;
      //printf("Bad sample, using %i instead\n", sample);
    }
    
    if (readings[sample * 2] == 0)
    {
      available[sample] = 1;
      availableCount--;
      //printf("bad sample, one less available. start over\n");
      continue;
    }

    float numbers[(NUMSAMPLES + 1) * 2];

    memset(numbers, 0, sizeof(numbers));

    int usingSamples[NUMSAMPLES];

    memset(usingSamples, 0, sizeof(usingSamples));

    int sampleCount = 0;

    int i;

    for (i = 0; i < numSamples; i++)
    {
      //printf("looking for another sample\n");
      int offset;

      int index;

      int check_if_used = 1;

      while (check_if_used)
      {
        offset = (random() % (2 * degreesOff)) - degreesOff;

        index = sample + offset;

        index %= 360;

        if (index < 0)
        {
          index += 360;
        }

        //printf("can I use %i?", index);

        if (index != sample)
        {
          //printf("it isn't the main point\n");          

          int ii;

          int in_set = 0;

          for (ii = 0; ii < i; ii ++)
          {
            if (index == usingSamples[ii])
            {
              in_set = 1;
            }
          }

          if (!in_set)
          {
            //printf("ok to use\n");
            check_if_used = 0;
            usingSamples[i] = index;
          }
        }
          
      }

      //printf("also using %i\n", index);

      if (!available[index] && readings[index * 2] != 0)
      {
        //printf("I can use it\n");
        numbers[sampleCount * 2] = readings[index * 2];
        numbers[(sampleCount * 2) + 1] = readings[(index * 2) + 1];
        sampleCount++;
      }
    }

    numbers[sampleCount * 2] = readings[sample * 2];
    numbers[(sampleCount * 2) + 2] = readings[(sample * 2) + 1];
    sampleCount++;

    struct line2d line;

    line.slope = 0;
    line.intercept = 0;

    leastsquareregression(&line, numbers, sampleCount);

    float A = line.slope;

    float B = -1;

    float C = line.intercept;


    //float norm = sqrtf(1);
    float norm = sqrtf((A * A) + (B * B));

    //printf("%f, %f, %f\n", line.slope, line.intercept, norm);

    A = A / norm;

    B = B / norm;

    C = C / norm;

    int onLine = 0;
    float fonLine = 0;

    int indexes[360];

    for (i = 0; i < count; i++)
    {
      if (!available[i])
      { 
        if (readings[i * 2] == 0)
        {
          available[i] = 1;
          availableCount--;
        
          continue;
        }

        float distance = fabs(A * readings[(i * 2)] + B * readings[(i * 2) + 1] + C);
        if (distance < distThreshold) 
        {
          indexes[onLine] = i;
          onLine++;
          fonLine += (M_PI * 2 * sqrtf((readings[i * 2] - x) * (readings[i * 2] - x) + (readings[(2 * i) + 1] - y) * (readings[(2 * i) + 1] - y))) / 360;
        }
        //printf("%f\n", distance);
      }
    }

    struct line2d foundLine;

    if (fonLine > fconsensus)
    {
      float buff[onLine * 2];
      for (i = 0; i < onLine; i++)
      {
        buff[i * 2] = readings[indexes[i] * 2];
        buff[(i * 2) + 1] = readings[(indexes[i] * 2) + 1];

        available[indexes[i]] = 1;
        availableCount--;
      }

      leastsquareregression(&foundLine, buff, onLine);

      float outx = foundLine.intercept / (-(1 / foundLine.slope) - foundLine.slope);
      float outy = -(1 / foundLine.slope) * outx;

      landmarks[outLines][0] = outx;
      landmarks[outLines][1] = outy;
      
      outLines++;

      if (outLines > maxreturn)
      {
        return outLines;
      }
    }

    iterations++;
  } 

  return outLines;
}
