#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <malloc.h>
#include "../include/bmp.h"
#define _height 700
#define _width 700
#define _bitsperpixel 24
#define _planes 1
#define _compression 0
#define _pixelbytesize _height*_width*_bitsperpixel/8
#define _filesize _pixelbytesize+sizeof(bitmap)
#define _xpixelpermeter 0x130B //2835 , 72 DPI
#define _ypixelpermeter 0x130B //2835 , 72 DPI
#define pixel 0x00
#pragma pack(push,1)
typedef struct{
    uint8_t signature[2];
    uint32_t filesize;
    uint32_t reserved;
    uint32_t fileoffset_to_pixelarray;
} fileheader;
typedef struct{
    uint32_t dibheadersize;
    uint32_t width;
    uint32_t height;
    uint16_t planes;
    uint16_t bitsperpixel;
    uint32_t compression;
    uint32_t imagesize;
    uint32_t ypixelpermeter;
    uint32_t xpixelpermeter;
    uint32_t numcolorspallette;
    uint32_t mostimpcolor;
} bitmapinfoheader;
typedef struct {
    fileheader fileheader;
    bitmapinfoheader bitmapinfoheader;
} bitmap;
#pragma pack(pop)

uint8_t *pixelbuffer;
bitmap *pbitmap;
FILE *fp;

void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
  if (x < 0 || x >= _width || y < 0 || y >= _height)
  {
    return;
  }

  pixelbuffer[3 * (x + (y * _width))] = b;
  pixelbuffer[3 * (x + (y * _width)) + 1] = g;
  pixelbuffer[3 * (x + (y * _width)) + 2] = r;
}

void setPixelB(int x, int y, uint8_t b)
{
  if (x < 0 || x >= _width || y < 0 || y >= _height)
  {
    return;
  }
  pixelbuffer[3 * (x + (y * _width))] = b;
}

void setPixelR(int x, int y, uint8_t r)
{
  if (x < 0 || x >= _width || y < 0 || y >= _height)
  {
    return;
  }
  pixelbuffer[3 * (x + (y * _width)) + 2] = r;
}

void setPixelG(int x, int y, uint8_t g)
{
  if (x < 0 || x >= _width || y < 0 || y >= _height)
  {
    return;
  }
  pixelbuffer[3 * (x + (y * _width)) + 1] = g;
}

void line(int x0, int y0, int x1, int y1) {
 
  int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  int err = (dx>dy ? dx : -dy)/2, e2;
 
  for(;;){
    setPixelB(x0,y0, 255);
    setPixelR(x0,y0, 0);
    if (x0==x1 && y0==y1) break;
    e2 = err;
    if (e2 >-dx) { err -= dy; x0 += sx; }
    if (e2 < dy) { err += dx; y0 += sy; }
  }
}
typedef int OutCode;
 
const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000
 
// Compute the bit code for a point (x, y) using the clip rectangle
// bounded diagonally by (xmin, ymin), and (xmax, ymax)
 
// ASSUME THAT xmax, xmin, ymax and ymin are global constants.
 
OutCode ComputeOutCode(double x, double y)
{
  OutCode code;
 
  code = INSIDE;          // initialised as being inside of clip window
 
  if (x < 0)           // to the left of clip window
    code |= LEFT;
  else if (x >= _width)      // to the right of clip window
    code |= RIGHT;
  if (y < 0)           // below the clip window
    code |= BOTTOM;
  else if (y >= _height)      // above the clip window
    code |= TOP;
 
  return code;
}

void draw_line_point_slope(double point[2], double slope)
{  
  double b = (point[1] - (slope * point[0]));

  //printf("%f\t%f\t%f\t%f\n", point[0], point[1], slope, b);

  int starty = -350 * slope + b;

  int stopy = 350 * slope + b;

  clipped_line(0, starty + 350, 700, stopy + 350);
}

void clipped_line(double x0, double y0, double x1, double y1)
{
  // compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
  OutCode outcode0 = ComputeOutCode(x0, y0);
  OutCode outcode1 = ComputeOutCode(x1, y1);
  uint8_t accept = 0;
 
  while (1) {
    if (!(outcode0 | outcode1)) { // Bitwise OR is 0. Trivially accept and get out of loop
      accept = 1;
      break;
    } else if (outcode0 & outcode1) { // Bitwise AND is not 0. Trivially reject and get out of loop
      break;
    } else {
      // failed both tests, so calculate the line segment to clip
      // from an outside point to an intersection with clip edge
      double x = 0, y = 0;
 
      // At least one endpoint is outside the clip rectangle; pick it.
      OutCode outcodeOut = outcode0 ? outcode0 : outcode1;
 
      // Now find the intersection point;
      // use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
      if (outcodeOut & TOP) {           // point is above the clip rectangle
        x = x0 + (x1 - x0) * ((_height - 1) - y0) / (y1 - y0);
        y = (_height - 1);
      } else if (outcodeOut & BOTTOM) { // point is below the clip rectangle
        x = x0 + (x1 - x0) * (0 - y0) / (y1 - y0);
        y = 0;
      } else if (outcodeOut & RIGHT) {  // point is to the right of clip rectangle
        y = y0 + (y1 - y0) * ((_width - 1) - x0) / (x1 - x0);
        x = (_width -2);
      } else if (outcodeOut & LEFT) {   // point is to the left of clip rectangle
        y = y0 + (y1 - y0) * (0 - x0) / (x1 - x0);
        x = 0;
      }
 
      // Now we move outside point to intersection point to clip
      // and get ready for next pass.
      if (outcodeOut == outcode0) {
        x0 = x;
        y0 = y;
        outcode0 = ComputeOutCode(x0, y0);
      } else {
        x1 = x;
        y1 = y;
        outcode1 = ComputeOutCode(x1, y1);
      }
    }
  }
  if (accept) {
               // Following functions are left for implementation by user based on
               // their platform (OpenGL/graphics.h etc.)
               line(x0, y0, x1, y1);
  }
} 

void init_bmp()
{
    pbitmap  = (bitmap*)calloc(1,sizeof(bitmap));
    pixelbuffer = (uint8_t*)malloc(_pixelbytesize);
    strcpy((char *) pbitmap->fileheader.signature,"BM");
    pbitmap->fileheader.filesize = _filesize;
    pbitmap->fileheader.fileoffset_to_pixelarray = sizeof(bitmap);
    pbitmap->bitmapinfoheader.dibheadersize =sizeof(bitmapinfoheader);
    pbitmap->bitmapinfoheader.width = _width;
    pbitmap->bitmapinfoheader.height = _height;
    pbitmap->bitmapinfoheader.planes = _planes;
    pbitmap->bitmapinfoheader.bitsperpixel = _bitsperpixel;
    pbitmap->bitmapinfoheader.compression = _compression;
    pbitmap->bitmapinfoheader.imagesize = _pixelbytesize;
    pbitmap->bitmapinfoheader.ypixelpermeter = _ypixelpermeter ;
    pbitmap->bitmapinfoheader.xpixelpermeter = _xpixelpermeter ;
    pbitmap->bitmapinfoheader.numcolorspallette = 0;
    memset(pixelbuffer,pixel,_pixelbytesize);
}

void clear_bmp()
{
  memset(pixelbuffer,pixel,_pixelbytesize);
}

void write_bmp()
{
    fp = fopen("/tmp/map.bmp","wb");
    fwrite (pbitmap, 1, sizeof(bitmap),fp);
    fwrite(pixelbuffer,1,_pixelbytesize,fp);
    fclose(fp);
    rename("/tmp/map.bmp", "/var/www/root/map.bmp");
    printf("tried to write image\n");
}

void free_bmp()
{
    free(pbitmap);
    free(pixelbuffer);
}

