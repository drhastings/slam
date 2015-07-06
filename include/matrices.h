#pragma once

#include <math.h>
#include <stdint.h>

#define MSET(m, x, y, v) m->values[(x * m->columns) + y] = v
#define MGET(m, x, y) m->values[(x * m->columns) + y]

struct mat
{
  uint32_t rows;
  uint32_t columns;

  double values[1];
};

struct mat * mat(uint32_t rows, uint32_t columns);

void addMat(struct mat * out, struct mat * a, struct mat * b);
void subMat(struct mat * out, struct mat * a, struct mat * b);
void prodMat(struct mat * out, struct mat * a, struct mat * b);

double determinant(struct mat *in);

void transpose(struct mat * out, struct mat *in);

void invert2x2(struct mat *out, struct mat *in);

void invert(struct mat *out, struct mat *in);

void copy_into(struct mat *out, struct mat *in, uint32_t x, uint32_t y);

void sub_mat(struct mat *out, struct mat *in, uint32_t x, uint32_t y);

void print_mat(struct mat *aMat);
