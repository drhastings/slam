#include "../include/quaternion.h"
#include <math.h>
#include <stdio.h>

float quaternion_to_theta(struct quaternion *quat)
{
  struct vect temp;

  temp.x = 2 * (quat->x * quat->y + quat->w * quat->z);

  temp.y = quat->w * quat->w - quat->x * quat->x + 
          quat->y * quat->y - quat->z * quat->z;

  temp.z = 2 * (quat->y * quat->z - quat->w * quat->x);

  return atan2f(-temp.x, -temp.z);
}

void quaternion_product(struct quaternion *result, struct quaternion * p,
      struct quaternion *q)
{
  struct quaternion temp;

  temp.w = p->w * q->w - p->x * q->x - p->y * q->y - p->z * q->z;
  temp.x = p->w * q->x + p->x * q->w + p->y * q->z - p->z * q->y;
  temp.y = p->w * q->y + p->y * q->w + p->z * q->x - p->x * q->z;
  temp.z = p->w * q->z + p->z * q->w + p->x * q->y - p->y * q->x;

  *result = temp;
}

void normalize(struct quaternion *out, struct quaternion *in)
{
  float norm = sqrtf(in->x * in->x + in->y * in->y + in->z * in->z + in->w * in->w);

  out->x = out->x / norm;
  out->y = out->y / norm;
  out->z = out->z / norm;
  out->w = out->w / norm;
}


void get_conjugate(struct quaternion *result, struct quaternion *q)
{
  result->w = q->w;
  result->x = -q->x;
  result->y = -q->y;
  result->z = -q->z;
}

void rotate_vector(struct vect *out, struct vect *in, struct quaternion *r)
{
  struct quaternion temp, sum, res, inverse;

  temp.x = in->x;
  temp.y = in->y;
  temp.z = in->z;
  temp.w = 0;

  get_conjugate(&inverse, r);

  quaternion_product(&sum, r, &temp);

  quaternion_product(&res, &sum, &inverse);

  out->x = res.x;
  out->y = res.y;
  out->z = res.z;
}
