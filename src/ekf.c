#include "../include/ekf.h"
#include "../include/from_frame.h"
#include "../include/scan.h"
#include "../include/inv_scan.h"
#include "../include/move.h"
#include "../include/observe.h"
#include "../include/inv_observe.h"

#include <stdlib.h>

struct mat * P = NULL;
struct mat * Q = NULL;

void ekf_init()
{
  P = mat(3, 3);
  Q = mat(2, 2);

  MSET(P, 0, 0, .01);
  MSET(P, 1, 1, .01);
  MSET(P, 2, 2, .02);

  MSET(Q, 0, 0, .001);
  MSET(Q, 1, 1, .004);
}

void ekf_prediction(struct mat * x, struct mat * U)
{
  struct mat * R_r = mat(3, 3);

  struct mat * R_n = mat(3, 2);

  struct mat * n = mat(2, 1);

  move(x, R_r, R_n, x, U, n);

  if (P->columns > 3)
  {
    struct mat * Prm = mat(3, P->columns - 3);

    sub_mat(Prm, P, 0, 3);

    prodMat(Prm, R_r, Prm);

    struct mat * Pmr = mat(P->rows - 3, 3); 
    transpose(Pmr, Prm);

    copy_into(P, Prm, 0, 3);
    copy_into(P, Pmr, 3, 0);

    free(Prm);
    free(Pmr);
  }

  struct mat * Prr = mat(3, 3);

  sub_mat(Prr, P, 0, 0);

  //printf("Prr\n");
  //print_mat(Prr);

  //printf("\nR_r\n");
  //print_mat(R_r);

  struct mat * temp = mat(3, 3);

  struct mat * R_rt = mat(3, 3);

  transpose(R_rt, R_r);

  //printf("\nR_rt\n");
  //print_mat(R_rt);

  prodMat(temp, Prr, R_rt);

  //printf("\ntemp\n");
  //print_mat(temp);

  prodMat(Prr, R_r, temp);

  //printf("\nPrr again\n");
  //print_mat(Prr);

  free(temp);
  free(R_rt);

  struct mat * R_nt = mat(2, 3);

  //printf("\nR_n\n");
  //print_mat(R_n);

  transpose(R_nt, R_n);
  //printf("\nR_nt\n");
  //print_mat(R_nt);

  temp = mat(2, 3);

  prodMat(temp, Q, R_nt);
  //printf("\ntemp\n");
  //print_mat(temp);

  struct mat * result = mat(3, 3);

  prodMat(result, R_n, temp);  

  //printf("\nresult\n");
  //print_mat(result);

  //addMat(Prr, Prr, result);

  free(R_nt);
  free(temp);
  free(result);

  copy_into(P, Prr, 0, 0);

  free(Prr);

  free(R_r);
  free(R_n);
  free(n);
}

void ekf_step(struct mat ** map, struct mat * U, double * sightings, int count)
{
  struct mat * x = mat(3, 1);
  struct mat * local_map = *map;

  sub_mat(x, local_map, 0, 0);

  ekf_prediction(x, U);

  print_mat(x);
  copy_into(local_map, x, 0, 0);

  free(x);

  
}

void test()
{
  struct mat * pf = mat(2, 1);
  struct mat * PF_f = mat(2, 3);
  struct mat * PF_p = mat(2, 2);

  struct mat * pw = mat(2, 1);
  struct mat * PW_f = mat(2, 3);
  struct mat * PW_pf = mat(2, 2);

  struct mat * F = mat(3, 1);
  struct mat * p = mat(2, 1);

  struct mat * y = mat(2, 1);
  struct mat * Y_p = mat(2, 2);
  struct mat * P_y = mat(2, 2);

  MSET(p, 0, 0, 15);
  MSET(p, 1, 0, 15);
  MSET(F, 0, 0, 15);
  MSET(F, 1, 0, 15);
  MSET(F, 2, 0, 1.14);

  to_frame(pf, PF_f, PF_p, F, p);

  from_frame(pw, PW_f, PW_pf, F, pf);

  scan(y, Y_p, pw);

  inv_scan(p, P_y, y);

  //print_mat(pf);
  //print_mat(pw);
  //print_mat(y);
  //print_mat(p);

  free(P_y);
  free(y);
  free(Y_p);
  free(pw);
  free(PW_f);
  free(PW_pf);
  free(pf);
  free(PF_f);
  free(PF_p);
  free(F);
  free(p);
}

int get_closest(struct mat ** map, double x, double y)
{
  int ret = 0;

  int i = 0;

  double closest = 100000.0;

  struct mat * local_map = *map;

  int count = (local_map->rows - 3) / 2;

  for (i = 0; i < count; i ++)
  {
    double x_dif = x - MGET(local_map, 0, 0);
    double y_dif = y - MGET(local_map, 1, 0);

    double dist = sqrt(x_dif * x_dif + y_dif * y_dif);

    if (dist < closest)
    {
      closest = dist;

      ret = i;
    }
  }
  return ret;
}
