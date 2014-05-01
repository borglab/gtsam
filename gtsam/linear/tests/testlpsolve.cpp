/*
 * testlpsolve.cpp
 * @brief:
 * @date: Apr 30, 2014
 * @author: Duy-Nguyen Ta
 */

#include "lp_lib.h"
#include <gtsam/base/Matrix.h>

using namespace std;
using namespace gtsam;

// lpsolve's simple demo from its website http://lpsolve.sourceforge.net/5.5/formulate.htm#C/C++
int demo()
{
  lprec *lp;
  int Ncol, *colno = NULL, j, ret = 0;
  REAL *row = NULL;

  /* We will build the model row by row
     So we start with creating a model with 0 rows and 2 columns */
  Ncol = 2; /* there are two variables in the model */
  lp = make_lp(0, Ncol);
  if(lp == NULL)
    ret = 1; /* couldn't construct a new model... */

  if(ret == 0) {
    /* let us name our variables. Not required, but can be useful for debugging */
    //    set_col_name(lp, 1, 'x');
    //    set_col_name(lp, 2, 'y');

    /* create space large enough for one row */
    colno = (int *) malloc(Ncol * sizeof(*colno));
    row = (REAL *) malloc(Ncol * sizeof(*row));
    if((colno == NULL) || (row == NULL))
      ret = 2;
  }

  if(ret == 0) {
    set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */

    /* construct first row (120 x + 210 y <= 15000) */
    j = 0;

    colno[j] = 1; /* first column */
    row[j++] = 120;

    colno[j] = 2; /* second column */
    row[j++] = 210;

    /* add the row to lpsolve */
    if(!add_constraintex(lp, j, row, colno, LE, 15000))
      ret = 3;
  }

  if(ret == 0) {
    /* construct second row (110 x + 30 y <= 4000) */
    j = 0;

    colno[j] = 1; /* first column */
    row[j++] = 110;

    colno[j] = 2; /* second column */
    row[j++] = 30;

    /* add the row to lpsolve */
    if(!add_constraintex(lp, j, row, colno, LE, 4000))
      ret = 3;
  }

  if(ret == 0) {
    /* construct third row (x + y <= 75) */
    j = 0;

    colno[j] = 1; /* first column */
    row[j++] = 1;

    colno[j] = 2; /* second column */
    row[j++] = 1;

    /* add the row to lpsolve */
    if(!add_constraintex(lp, j, row, colno, LE, 75))
      ret = 3;
  }

  if(ret == 0) {
    set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */

    /* set the objective function (143 x + 60 y) */
    j = 0;

    colno[j] = 1; /* first column */
    row[j++] = 143;

    colno[j] = 2; /* second column */
    row[j++] = 60;

    /* set the objective in lpsolve */
    if(!set_obj_fnex(lp, j, row, colno))
      ret = 4;
  }

  if(ret == 0) {
    /* set the object direction to maximize */
    set_maxim(lp);

    /* just out of curioucity, now show the model in lp format on screen */
    /* this only works if this is a console application. If not, use write_lp and a filename */
    write_LP(lp, stdout);
    /* write_lp(lp, "model.lp"); */

    /* I only want to see important messages on screen while solving */
    set_verbose(lp, IMPORTANT);

    /* Now let lpsolve calculate a solution */
    ret = solve(lp);
    if(ret == OPTIMAL)
      ret = 0;
    else
      ret = 5;
  }

  if(ret == 0) {
    /* a solution is calculated, now lets get some results */

    /* objective value */
    printf("Objective value: %f\n", get_objective(lp));

    /* variable values */
    get_variables(lp, row);
    for(j = 0; j < Ncol; j++)
      printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);

    /* we are done now */
  }

  /* free allocated memory */
  if(row != NULL)
    free(row);
  if(colno != NULL)
    free(colno);

  if(lp != NULL) {
    /* clean up such that all used memory by lpsolve is freed */
    delete_lp(lp);
  }

  return(ret);
}

// Try to convert from gtsam's Matrix and Vector to lpsolve for a
// simple Matlab example http://www.mathworks.com/help/optim/ug/linprog.html
int demo2()
{
  Vector f(3);
  f << -5, -4, -6;
  Matrix A(3,3);
  A << 1,-1,1,3,2,4,3,2,0;
  Vector b(3);
  b << 20,42,30;

  lprec *lp;
  int Ncol, j, ret = 0;

  /* We will build the model row by row
     So we start with creating a model with 0 rows and 2 columns */
  Ncol = 3; /* there are two variables in the model */
  lp = make_lp(0, Ncol);
  if(lp == NULL)
    return 1; /* couldn't construct a new model... */

  int colno[] = {1,2,3};
  set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */
  for (int i = 0; i<A.rows(); ++i) {
    Vector r = A.row(i);
    double* data = r.data();
    for (int j= 0; j<3; ++j)
      cout << data[j] << endl;
    cout << "A.row(i): " << A.row(i) << endl;
    if(!add_constraintex(lp, A.cols(), r.data(), colno, LE, b[i]))
      return 3;
  }

  set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */

  /* set the objective function */
  /* set the objective in lpsolve */
  if(!set_obj_fnex(lp, f.size(), f.data(), colno))
    return 4;

  /* set the object direction to maximize */
  set_minim(lp);

  /* just out of curioucity, now show the model in lp format on screen */
  /* this only works if this is a console application. If not, use write_lp and a filename */
  write_LP(lp, stdout);
  /* write_lp(lp, "model.lp"); */

  /* I only want to see important messages on screen while solving */
  set_verbose(lp, IMPORTANT);

  /* Now let lpsolve calculate a solution */
  ret = solve(lp);
  if(ret == OPTIMAL)
    ret = 0;
  else
    ret = 5;

  if(ret == 0) {
    /* a solution is calculated, now lets get some results */

    /* objective value */
    printf("Objective value: %f\n", get_objective(lp));

    /* variable values */
    REAL* row = NULL;
    get_ptr_variables(lp, &row);
    for(j = 0; j < Ncol; j++)
      printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);

    /* we are done now */
  }

  /* free allocated memory */

  if(lp != NULL) {
    /* clean up such that all used memory by lpsolve is freed */
    delete_lp(lp);
  }

  return(ret);
}

int main()
{
    demo();
    demo2();
}
