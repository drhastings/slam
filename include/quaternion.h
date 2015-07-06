#pragma once

#include "vector.h"

struct quaternion
{
  float w;
  float x;
  float y;
  float z;
};

float quaternion_to_theta(struct quaternion *quat);

void quaternion_product(struct quaternion *result, struct quaternion * p,
      struct quaternion *q);

void normalize(struct quaternion *out, struct quaternion *in);

void rotate_vector(struct vect *out, struct vect *in, struct quaternion *r);

void get_conjugate(struct quaternion *result, struct quaternion *q);
