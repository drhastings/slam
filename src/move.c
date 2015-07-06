#include "../include/move.h"
#include <math.h>
#include <stdlib.h>
#include "../include/from_frame.h"

void move(struct mat * ro, struct mat * RO_r, struct mat * RO_n, struct mat * r,
          struct mat * u, struct mat * n)
{
  double a = MGET(r, 2, 0);
  double dx = MGET(u, 0, 0) + MGET(n, 0, 0);
  double da = MGET(u, 1, 0) + MGET(n, 1, 0);

  double ao = a + da;

  //printf("ao = %f\n", ao);
  if (ao > M_PI)
  {
    ao = ao - 2.0 * M_PI;
  }
  if (ao < -M_PI)
  {
    ao = ao + 2.0 * M_PI;
  }

  struct mat * dp = mat(2, 1);

  MSET(dp, 0, 0, dx);
  //MSET(dp, 1, 0, da);

  struct mat * to = mat(2, 1);
  struct mat * TO_r = mat(2, 3);
  struct mat * TO_dt = mat(2, 2);

  from_frame(to, TO_r, TO_dt, r, dp);

  float AO_a = 1;
  float AO_da = 1;

  copy_into(RO_r, TO_r, 0, 0);
  MSET(RO_r, 2, 0, 0);
  MSET(RO_r, 2, 1, 0);
  MSET(RO_r, 2, 2, AO_a);

  copy_into(RO_n, TO_dt, 0, 0);
  MSET(RO_n, 0, 1, 0);
  MSET(RO_n, 1, 1, 0);
  MSET(RO_n, 2, 1, AO_da);

  //print_mat(to);
  //printf("%f\n", ao);

  copy_into(ro, to, 0, 0);
  MSET(ro, 2, 0, ao);

  free(to);
  free(TO_r);
  free(TO_dt);
  free(dp);
}
