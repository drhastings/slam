#include "../include/ekf.h"
#include "../include/from_frame.h"
#include "../include/scan.h"
#include "../include/inv_scan.h"
#include "../include/move.h"
#include "../include/observe.h"
#include "../include/inv_observe.h"

#include <stdlib.h>
#include <stdio.h>

struct mat * P = NULL;
struct mat * Q = NULL;
struct mat * S = NULL;

void ekf_init()
{
  P = mat(3, 3);
  Q = mat(2, 2);
  S = mat(2, 2);

  MSET(P, 0, 0, .01);
  MSET(P, 1, 1, .01);
  MSET(P, 2, 2, .02);

  MSET(Q, 0, 0, .001);
  MSET(Q, 1, 1, .004);

  MSET(S, 0, 0, .01);
  MSET(S, 1, 1, M_PI / 180);
}

int ekf_correction(struct mat * map, double x, double y, int index)
{
  int ret = 0;

  struct mat *xr = mat(3, 1);

  sub_mat(xr, map, 0, 0);

  struct mat *xl = mat(2, 1);

  struct mat *landmark = mat(2, 1);
  struct mat *lmrb = mat(2, 1);

  MSET(landmark, 0, 0, x);
  MSET(landmark, 1, 0, y);

  sub_mat(xl, map, (index * 2) + 3, 0);

  struct mat *e = mat(2, 1);

  struct mat *E_r = mat(2, 3);

  struct mat *E_l = mat(2, 2);

  observe(lmrb, NULL, NULL, xr, landmark);

  observe(e, E_r, E_l, xr, xl);

  struct mat *E_rl = mat(2, 5);

  struct mat *E_rlt = mat(5, 2);

  copy_into(E_rl, E_r, 0, 0);
  copy_into(E_rl, E_l, 0, 3);

  transpose(E_rlt, E_rl);

  struct mat *Prl = mat(5, 5);

  struct mat *Pr = mat(3, 3);

  struct mat *Pl = mat(2, 2);

  sub_mat(Pr, P, 0, 0);
  sub_mat(Pl, P, (index * 2) + 3, (index * 2) + 3);

  copy_into(Prl, Pr, 0, 0);
  copy_into(Prl, Pl, 3, 3);

  struct mat *temp = mat(2, 5);

  prodMat(temp, E_rl, Prl);

  struct mat *E = mat(2, 2);

  prodMat(E, temp, E_rlt);

  struct mat *z = mat(2, 1);
  
  subMat(z, lmrb, e);

  if (MGET(z, 2, 0) > M_PI)
  {
    MSET(z, 2, 0, MGET(z, 2, 0) - 2 * M_PI);
  }

  if (MGET(z, 2, 0) < -M_PI)
  {
    MSET(z, 2, 0, MGET(z, 2, 0) + 2 * M_PI);
  }

  struct mat *Z = mat(2, 2);

  addMat(Z, S, E);

  struct mat *invZ = mat(2, 2);

  invert2x2(invZ, Z);

  free(temp);

  temp = mat(1, 2);

  struct mat *zt = mat(1, 2);

  transpose(zt, z);

  prodMat(temp, zt, invZ);

  struct mat *check = mat(1, 1);

  prodMat(check, temp, z);
  
  if (MGET(check, 0, 0) < 9)
  {
    free(temp);

    temp = mat(5, 2);

    prodMat(temp, E_rlt, invZ);

    struct mat * P_rm_rl = mat(P->rows, 5);

    struct mat * first_part = mat(P->rows, 3);

    struct mat * second_part = mat(P->rows, 2);

    sub_mat(first_part, P, 0, 0);

    sub_mat(second_part, P, 0, (index * 2) + 3);

    copy_into(P_rm_rl, first_part, 0, 0);

    copy_into(P_rm_rl, second_part, 0, 3);

    struct mat * K = mat(P->rows, 2);

    prodMat(K, P_rm_rl, temp);

    //print_mat(K);

    free(temp);

    temp = mat(P->rows, 2);

    prodMat(temp, K, z);

    addMat(map, map, temp);

    free(temp);

    temp = mat(P->rows, P->columns);

    struct mat *Kt = mat(2, K->rows);

    struct mat * temp2 = mat(K->rows, 2);

    prodMat(temp2, K, Z);

    prodMat(temp, temp2, Kt);

    subMat(P, P, temp);
   
    free(Kt);
    free(temp2);
    free(K);
    free(P_rm_rl);
    free(first_part);
    free(second_part);
  }
  else if (MGET(check, 0, 0) > 9)
  {
    ret = 1;
  }

  //print_mat(check);
  //print_mat(z);

  //printf("%f\t%f\n", x, y);

  free(check);
  free(zt);
  free(invZ);
  free(Z);
  free(z);
  free(E);
  free(temp);
  free(Pr);
  free(Pl);
  free(Prl);
  free(E_rl);
  free(E_rlt);
  free(landmark);
  free(lmrb);
  free(xr);
  free(xl);
  free(e);
  free(E_r);
  free(E_l);

  return ret;
}

void ekf_init_lm(struct mat ** map, double x, double y)
{
  struct mat * local_map = *map;

  struct mat *new_map = mat(local_map->rows + 2, 1);

  copy_into(new_map, local_map, 0, 0);

  MSET(new_map, local_map->rows, 0, x);
  MSET(new_map, local_map->rows + 1, 0, y);

  *map = new_map;

  struct mat *Yi = mat(2, 1);
  struct mat *landmark = mat(2, 1);

  MSET(landmark, 0, 0, x);
  MSET(landmark, 1, 0, y);

  struct mat *mr = mat(3, 1);

  sub_mat(mr, local_map, 0, 0);

  observe(Yi, NULL, NULL, mr, landmark);

  struct mat *xl = mat(2, 1);

  struct mat * L_r = mat(2, 3);

  struct mat * L_y = mat(2, 2);

  inv_observe(xl, L_r, L_y, mr, Yi);

  struct mat * P_r_rm = mat(3, P->columns);

  sub_mat(P_r_rm, P, 0, 0);

  struct mat *P_l_rm = mat(2, P->columns);

  prodMat(P_l_rm, L_r, P_r_rm);

  struct mat *P_rm_l = mat(P->rows, 2);

  transpose(P_rm_l, P_l_rm);

  struct mat *P_l_l = mat(2, 2);

  struct mat * L_rt = mat(3, 2);
  struct mat * L_yt = mat(2, 2);
  
  transpose(L_rt, L_r);
  transpose(L_yt, L_y);

  struct mat *Prr = mat(3, 3);

  sub_mat(Prr, P, 0, 0);

  struct mat * temp = mat(2, 3);

  prodMat(temp, L_r, Prr);

  struct mat * temp2 = mat(2, 2);

  prodMat(temp2, temp, L_rt);

  free(temp);

  temp = mat(2, 2);

  prodMat(temp, L_y, S);
  prodMat(temp, temp, L_yt);

  addMat(P_l_l, temp2, temp);

  struct mat *newP = mat(P->rows + 2, P->columns + 2);

  copy_into(newP, P, 0, 0);
  copy_into(newP, P_l_rm, P->rows, 0);
  copy_into(newP, P_rm_l, 0, P->columns);
  copy_into(newP, P_l_l, P->rows, P->columns);

  free(P);
  P = newP;

  free(Prr);
  free(temp);
  free(temp2);
  free(L_rt);
  free(P_l_l);
  free(P_l_rm);
  free(P_rm_l);
  free(P_r_rm);
  free(xl);
  free(L_r);
  free(L_y);
  free(landmark);
  free(mr);
  free(Yi);
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
  //printf("updating\n");

  //print_mat(x);
  copy_into(local_map, x, 0, 0);

  free(x);

  int i = 0;

  int add_this_one = -1;

  int using = 0;

  for (i = 0; i < count; i++)
  {
    int index = get_closest(map, sightings[i * 2], sightings[(i * 2) + 1]);
    //printf("predicting\n");
    //printf("%d\n", index);
    int ret = 0;
    if (index >= 0)
    {
      ret = ekf_correction(local_map, sightings[i * 2], sightings[(i * 2) + 1], index);
      if (!ret)
      {
        using++;
      }
    }
    else if(add_this_one < 0)
    {
      add_this_one = i;
    }
    if (add_this_one < 0 && ret)
    {
      add_this_one = i;
    }
  }  
  if (add_this_one >= 0)
  {
    ekf_init_lm(map, sightings[(add_this_one * 2)], sightings[(add_this_one * 2) + 1]);
    printf("adding %d\n", add_this_one);
  }
  //printf("%f, %f\n", sightings[(add_this_one * 2)], sightings[(add_this_one * 2) + 1]);
  printf("I think I recognize %d lines\n", using);
  //print_mat(local_map);
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
  int ret = -1;

  int i = 0;

  double closest = 100000.0;

  struct mat * local_map = *map;

  int count = (local_map->rows - 3) / 2;

  for (i = 0; i < count; i ++)
  {
    double x_dif = x - MGET(local_map, (i * 2) + 3, 0);
    double y_dif = y - MGET(local_map, (i * 2) + 4, 0);

    double dist = sqrt(x_dif * x_dif + y_dif * y_dif);

    if (dist < closest)
    {
      closest = dist;

      ret = i;
    }
  }
  return ret;
}
