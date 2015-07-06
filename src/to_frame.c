#include "../include/to_frame.h"
#include <stdlib.h>

void to_frame(struct mat * pf, struct mat * PF_f, struct mat * PF_p,
                struct mat * F, struct mat * P)
{
  struct mat * t = mat(2, 1);

  float a = MGET(F, 2, 0);

  sub_mat(t, F, 0, 0);

  struct mat * R = mat(2, 2);

  MSET(R, 0, 0, cos(a));
  MSET(R, 0, 1, -sin(a));
  MSET(R, 1, 0, sin(a));
  MSET(R, 1, 1, cos(a));

  struct mat * temp = mat(2, 1);

  subMat(temp, P, t);

  struct mat * temp2 = mat(2, 2);

  transpose(temp2, R);

  prodMat(pf, temp2, temp);

  free(temp);
  free(temp2);

  float px = MGET(P, 0, 0);
  float py = MGET(P, 1, 0);

  float x = MGET(t, 0, 0);
  float y = MGET(t, 1, 0);

  MSET(PF_f, 0, 0, -cos(a));
  MSET(PF_f, 0, 1, -sin(a));
  MSET(PF_f, 0, 2, cosf(a) * (py - y) - sinf(a) * (px - x));
  MSET(PF_f, 1, 0, sin(a));
  MSET(PF_f, 1, 1, -cosf(a));
  MSET(PF_f, 1, 2, -cosf(a) * (px - x) - sin(a) * (py - y));

  transpose(PF_p, R);
}
