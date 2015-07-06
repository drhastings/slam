#pragma once

struct hough_transformer
{
  int theta_level;
  int rho_level;
  uint16_t * houghmap;
  int count;
  double * results;
};

void de_hough(double *x, double *y, double rho, double theta, struct hough_transformer * transform);

int init_hough(unsigned int theta_level, 
      unsigned int rho_level, struct hough_transformer * this);

void adaptive_threshold(double percentage, struct hough_transformer * this);

void release_hough(struct hough_transformer * this);

int hough(double points[360 * 2], int num_points, struct hough_transformer * transform);
