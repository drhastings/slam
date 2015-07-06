#include "../include/matrices.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct mat * mat(uint32_t rows, uint32_t columns)
{
  uint64_t size = rows * columns - 1;

  struct mat * ret = calloc(1, (sizeof(double) * size) + sizeof(struct mat));

  if (ret != NULL)
  {
    ret->rows = rows;
    ret->columns = columns;
  }

  return (struct mat *) ret;
}

void addMat(struct mat * out, struct mat * a, struct mat * b)
{
  uint32_t x = 0, y = 0;

  for (x = 0; x < a->rows; x++)
  {
    for (y = 0; y < a->columns; y++)
    {
      MSET(out, x, y, MGET(a, x, y) + MGET(b, x, y));
    }
  }
}

void subMat(struct mat * out, struct mat * a, struct mat * b)
{
  uint32_t x = 0, y = 0;

  for (x = 0; x < a->rows; x++)
  {
    for (y = 0; y < a->columns; y++)
    {
      MSET(out, x, y, MGET(a, x, y) - MGET(b, x, y));
    }
  }
}

void prodMat(struct mat * out, struct mat * a, struct mat * b)
{
  uint32_t x = 0, y = 0;

  struct mat * temp = mat(out->rows, out->columns);

  for (x = 0; x < a->rows; x++)
  {
    for (y = 0; y < b->columns; y++)
    {
      double value = 0;

      uint32_t i = 0;

      for (i = 0; i < a->columns; i++) 
      {
        value += MGET(a, x, i) * MGET(b, i, y);
        //printf("%d, %d, %d\n", x, y, i);
      }

      MSET(temp, x, y, value);
    }
  }

  memcpy(&out->values[0], &temp->values[0], sizeof(double) * (temp->rows * temp->columns));

  free(temp);
}

void transpose(struct mat * out, struct mat *in)
{
  uint32_t i, j;

  for (i = 0; i < in->rows; i++)
  {
    for (j = 0; j < in->columns; j++)
    {
      MSET(out, j, i, MGET(in, i, j));
    }
  }
}

void invert2x2(struct mat *out, struct mat *in)
{
  double det = MGET(in, 0, 0) * MGET(in, 1, 1) - MGET(in, 1, 0) * MGET(in, 0, 1);
  MSET(out, 0, 0, MGET(in, 1, 1) / det);
  MSET(out, 0, 1, -MGET(in, 0, 1) / det);
  MSET(out, 1, 0, -MGET(in, 1, 0) / det);
  MSET(out, 1, 1, MGET(in, 0, 0) / det);
}

void invert(struct mat *out, struct mat *in)
{
  struct mat *temp = mat(in->rows, in->columns * 2);

  copy_into(temp, in, 0, 0);

  uint32_t i;

  for (i = 0; i < in->rows; i++)
  {
    MSET(temp, i, i + in->rows, 1);
  }

  for (i = 0; i < in->rows; i++)
  {
    double max = fabs(MGET(temp, i, i));
    uint32_t j, largest = i;

    for (j = i + 1; j < in->rows; j++)
    {
      if (max < fabs(MGET(temp, j, i)))
      {
        max = fabs(MGET(temp, j, i));

        largest = j;
      }
    }
    
    if (largest != i)
    {
      double temprow[temp->columns];

      memcpy(temprow, &temp->values[largest * temp->columns], sizeof(double) * temp->columns);
      memcpy(&temp->values[largest * temp->columns], &temp->values[i * temp->columns], sizeof(double) * temp->columns);
      memcpy(&temp->values[i * temp->columns], temprow, sizeof(double) * temp->columns);
    }

    for (j = i + 1; j < temp->rows; j++)
    {
      double factor = MGET(temp, j, i) / MGET(temp, i, i);

      uint32_t ii;

      for (ii = i; ii < temp->columns; ii++)
      {
        float new_val = MGET(temp, j, ii) - factor * MGET(temp, i, ii);

        MSET(temp, j, ii, new_val);
      }
    }

  }

  for (i = temp->rows - 1; i < temp->rows; i--)
  {
    double factor = 1 / MGET(temp, i, i); 

    uint32_t ii;

    for (ii = i; ii < temp->columns; ii++)
    {
      MSET(temp, i, ii, MGET(temp, i, ii) * factor);
    }

    for (ii = i - 1; ii < temp->rows; ii--)
    {
      double factor = MGET(temp, ii, i);

      uint32_t jj;

      for (jj = i; jj < temp->columns; jj++)
      {
        float new_val = MGET(temp, ii, jj) - factor * MGET(temp, i, jj);

        MSET(temp, ii, jj, new_val);
      }
    }

  }

  //print_mat(temp);

  sub_mat(out, temp, 0, out->columns);

  free(temp);
}

void copy_into(struct mat *out, struct mat *in, uint32_t x, uint32_t y)
{
  uint32_t i;

  for (i = 0; i < in->rows; i++)
  {
    memcpy(&out->values[((x + i) * out->columns) + y], &in->values[i * in->columns], sizeof(double) * in->columns);
  }
}

void sub_mat(struct mat *out, struct mat *in, uint32_t x, uint32_t y)
{
  uint32_t i;

  for (i = 0; i < out->rows; i++)
  {
    memcpy(&out->values[i * out->columns], &in->values[((x + i) * in->columns) + y], sizeof(double) * out->columns);
  }
}

double determinant(struct mat *in)
{
  float s=1, det=0;
  uint32_t i,j,m,n,c;
  if (in->rows == 1)
  {
    return MGET(in, 0, 0);
  }
  else
  {
    det = 0;
    for (c = 0; c < in->rows; c++)
    {
      struct mat *temp = mat(in->rows - 1, in->columns - 1);
      m = 0;
      n = 0;
     
      for (i = 0; i < in->rows; i++)
      {
        for (j=0 ; j < in->rows; j++)
        {
          if (i != 0 && j != c)
          {
            MSET(temp, m, n, MGET(in, i, j));
            if (n < (in->rows - 2))
            {
              n++;
            }
            else
            {
              n=0;
              m++;
            }
          }
        }
      }
      det = det + s * (MGET(in, 0, c) * determinant(temp));
      s = -1 * s;

      free(temp);
    }
  }

   return (det);
}


void print_mat(struct mat *aMat)
{
  uint32_t x, y;

  for (x = 0; x < aMat->rows; x++)
  {
    for (y = 0; y < aMat->columns; y++)
    {
      printf("%f\t", aMat->values[(x * aMat->columns) + y]);
    }
    printf("\n");
  }
}
