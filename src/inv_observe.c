#include "../include/inv_observe.h"
#include "../include/from_frame.h"
#include "../include/inv_scan.h"
#include <stdlib.h>

void inv_observe(struct mat * p, struct mat * P_r, struct mat * P_y, 
                struct mat * r, struct mat * y)
{
  struct mat * p_r = mat(2, 1);
  struct mat * PR_y = mat(2, 2);
  
  inv_scan(p_r, PR_y, y);

  struct mat * P_pr = mat(2, 2);

  from_frame(p, P_r, P_pr, r, p_r);

  prodMat(P_y, P_pr, PR_y);
  
  free(P_pr);
  free(p_r);
  free(PR_y);
}
