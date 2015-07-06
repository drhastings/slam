#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "../include/bmp.h"
#include "../include/hough.h"

#define THETA_LEVEL 360
#define RHO_LEVEL 800

void de_hough(double *x, double *y, double rho, double theta, struct hough_transformer * transform)
{
  double temp_rho = rho - transform->rho_level / 2;
  temp_rho = temp_rho / (transform->rho_level / 2);

  temp_rho = temp_rho * 5000.0;

  double temp_theta = (theta * (M_PI / transform->theta_level)) - M_PI / 2;;

  *x = temp_rho * cos(temp_theta);
  *y = temp_rho * sin(temp_theta);

}

//takes the quantization factor for the hough map and allocates memory
int init_hough(unsigned int theta_level, 
      unsigned int rho_level, struct hough_transformer * this)
{
  this->theta_level = theta_level;
  this->rho_level = rho_level;

  if (!this->houghmap)
  {
    this->houghmap = calloc(theta_level * rho_level, sizeof(uint16_t));
  }

  this->results = calloc(100, sizeof(double));

  if (!this->houghmap)
  {
    return 1;
  }
  return 0;
}

void adaptive_threshold(double percentage, struct hough_transformer * this)
{
  int x, y;

  unsigned int most = 0;

  for (x = 0; x < this->rho_level; x++)
  {
    for (y = 0; y < this->theta_level; y++)
    {
      if (most < *(this->houghmap + (y * this->rho_level) + x))
      {
        most = *(this->houghmap + (y * this->rho_level) + x);
      }
    }
  }

  int threshhold = most * percentage;

  int count = 0;

  double outx = 0, outy = 0;

  int xx, yy;


  for (xx = 0; xx < this->rho_level; xx++)
  {
    for (yy = 0; yy < this->theta_level; yy++)
    {
      if (threshhold < *(this->houghmap + (yy * this->rho_level) + xx))
      {
        int neighbor_is_bigger = 0;

        int x_min = -3, x_max = 3;
        int y_min = -3, y_max = 3;

        int i, j;

        if(xx < -x_min)
        {
          x_min = -xx;
        }
        if(xx + x_max > this->rho_level) {
          x_max = this->rho_level - xx;
        }
        if(yy < -y_min)
        {
          y_min = -yy;
        }
        if(yy + y_max > this->theta_level)
        {
          y_max = this->theta_level - yy;
        }

        for (i = x_min; i < x_max + 1; i++)
        {
          for (j = y_min; j < y_max + 1; j++)
          {
            if (*(this->houghmap + ((yy + j) * this->rho_level) + (xx + i)) >=  *(this->houghmap + (yy * this->rho_level) + xx))
            {
              neighbor_is_bigger++;

            }
          }
        }
        
        if (neighbor_is_bigger < 2)
        {
          de_hough(&outx, &outy, xx, yy, this);

          this->results[count * 2] = outx;
          this->results[(count * 2) + 1] = outy;
          count++;
          //printf("%f, %f\n", outx, outy);
          //setPixel(outx / 25 + 350, outy / 25 + 350, 0, 255, 0);
          //draw_line_point_slope(point, -outx / outy);
        }
        else
        {
          *(this->houghmap + (yy * this->rho_level) + xx) -= 1;
        }
      }
    }
  }
  this->count = count;
}

void release_hough(struct hough_transformer * this)
{
  free(this->houghmap);
}

int hough(double points[360 * 2], int num_points, struct hough_transformer * transform)
{
  int x = 0, y = 0;

  //init_bmp();

  //setPixel(350, 350, 255, 255, 255);

  memset(transform->houghmap, 0, transform->theta_level * transform->rho_level * sizeof(uint16_t));

  for (x = 0; x < num_points; x++)
  {
    //setPixel(points[x][0] / 25 + 350, points[x][1] / 25 + 350, 255, 0, 0);
    for (y = 0; y < transform->theta_level; y++)
    {
      double temp = y, temp2 = transform->theta_level;

      double theta = (temp * (M_PI / temp2)) - M_PI / 2;

      //printf("%d, %f\n", y, theta);

      double x1 = points[x * 2], y1 = points[x * 2 + 1];

      float rhof = (x1 * cos(theta)) + (y1 * sin(theta));

      //rhof += 4000.0;

      rhof = rhof / 5000.0;

      rhof = rhof * transform->rho_level / 2;

      rhof += transform->rho_level / 2;

      //rhof = ((6000.0 / transform->rho_level) * (rhof / 6000.0) + (3000.0 / transform->rho_level));

      int rho = rhof;

      *(transform->houghmap + (y * transform->rho_level) + rho) += 1;


//      printf("%d, %d\n", rho, y);
    }
  }

  adaptive_threshold(.4, transform);

  //write_bmp();

  //free_bmp();

  return 0;
}
