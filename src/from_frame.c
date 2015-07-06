#include "../include/from_frame.h"
#include <string.h>
#include <stdlib.h>

void from_frame(struct mat * pw, struct mat * PW_f, struct mat * PW_pf,
                struct mat * F, struct mat * pf)
{
  struct mat * t = mat(2, 1);

  double a = MGET(F, 2, 0);

  sub_mat(t, F, 0, 0);

  struct mat * R = mat(2, 2);

  MSET(R, 0, 0, cos(a));
  MSET(R, 0, 1, -sin(a));
  MSET(R, 1, 0, sin(a));
  MSET(R, 1, 1, cos(a));

  struct mat * temp = mat(2, 1);

  prodMat(temp, R, pf);

  addMat(pw, temp, t);

  free(temp);

  double px = MGET(pf, 0, 0);
  double py = MGET(pf, 1, 0);

  MSET(PW_f, 0, 0, 1);
  MSET(PW_f, 0, 2, -py*cos(a) - px * sin(a));
  MSET(PW_f, 1, 1, 1);
  MSET(PW_f, 1, 2, px * cos(a) - py * sin(a));

  memcpy(&PW_pf->values[0], &R->values[0], sizeof(double) * (R->rows * R->columns));

  free(R);
}
