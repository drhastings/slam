#include "../include/inv_scan.h"

void inv_scan(struct mat * p, struct mat * P_y, struct mat * y)
{
  float d = MGET(y, 0, 0);
  float a = MGET(y, 1, 0);

  float px = d * cos(a);
  float py = d * sin(a);

  MSET(p, 0, 0, px);
  MSET(p, 1, 0, py);

  MSET(P_y, 0, 0, cos(a));
  MSET(P_y, 0, 1, -d * sin(a));
  MSET(P_y, 1, 0, sin(a));
  MSET(P_y, 1, 1, d * cos(a));
}
