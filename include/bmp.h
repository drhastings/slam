#pragma once


void clipped_line(double x0, double y0, double x1, double y1);
void init_bmp();
void write_bmp();
void free_bmp();
void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);

void draw_line_point_slope(double point[2], double slope);
void setPixelB(int x, int y, uint8_t b);

void setPixelR(int x, int y, uint8_t r);

void setPixelG(int x, int y, uint8_t g);
void clear_bmp();
