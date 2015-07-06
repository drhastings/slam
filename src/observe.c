#include "../include/observe.h"
#include "../include/from_frame.h"
#include "../include/to_frame.h"
#include "../include/scan.h"
#include "../include/matrices.h"
#include <stdlib.h>

void observe(struct mat * y, struct mat * Y_r, struct mat * Y_p, struct mat * r,
            struct mat * p)
{
  struct mat * pr = mat(2, 1);

  struct mat * PR_r = mat(2, 3);

  struct mat * PR_p = mat(2, 2);

  to_frame(pr, PR_r, PR_p, r, p);

  struct mat * Y_pr = mat(2, 2);

  scan(y, Y_pr, pr);

  prodMat(Y_r, Y_pr, PR_r);
  prodMat(Y_p, Y_pr, PR_p);

  free(Y_pr);
  free(pr);
  free(PR_r);
  free(PR_p);
}
