#include "../include/scan.h"

void scan(struct mat * y, struct mat * Y_p, struct mat * p)
{
  float px = MGET(p, 0, 0);
  float py = MGET(p, 1, 0);

  float d = sqrt(px * px + py * py);
  float a = atan2(py, px);

  MSET(y, 0, 0, d);
  MSET(y, 1, 0, a);

  MSET(Y_p, 0, 0, px / d);
  MSET(Y_p, 0, 1, py / d);
  MSET(Y_p, 1, 0, -py/(px*px*(py*py/px*px + 1)));
  MSET(Y_p, 1, 1, 1/(px*(py*py/px*px + 1)));
}
