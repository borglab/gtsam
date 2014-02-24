/*!
\file  gksort.c
\brief Testing module for the various sorting routines in GKlib

\date   Started 4/4/2007
\author George
\version\verbatim $Id: gksort.c 11058 2011-11-10 00:02:50Z karypis $ \endverbatim
*/

#include <GKlib.h>

#define N       10000

/*************************************************************************/
/*! Testing module for gk_?isort() routine */
/*************************************************************************/
void test_isort()
{
  gk_idx_t i;
  int array[N];

  /* test the increasing sort */
  printf("Testing iisort...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432);

  gk_isorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] > array[i+1])
      printf("gk_isorti error at index %jd [%d %d]\n", (intmax_t)i, array[i], array[i+1]);
  }


  /* test the decreasing sort */
  printf("Testing disort...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432);

  gk_isortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] < array[i+1])
      printf("gk_isortd error at index %jd [%d %d]\n", (intmax_t)i, array[i], array[i+1]);
  }

}


/*************************************************************************/
/*! Testing module for gk_?fsort() routine */
/*************************************************************************/
void test_fsort()
{
  gk_idx_t i;
  float array[N];

  /* test the increasing sort */
  printf("Testing ifsort...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432)/(1.0+RandomInRange(645323));

  gk_fsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] > array[i+1])
      printf("gk_fsorti error at index %jd [%f %f]\n", (intmax_t)i, array[i], array[i+1]);
  }


  /* test the decreasing sort */
  printf("Testing dfsort...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432)/(1.0+RandomInRange(645323));

  gk_fsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] < array[i+1])
      printf("gk_fsortd error at index %jd [%f %f]\n", (intmax_t)i, array[i], array[i+1]);
  }

}


/*************************************************************************/
/*! Testing module for gk_?idxsort() routine */
/*************************************************************************/
void test_idxsort()
{
  gk_idx_t i;
  gk_idx_t array[N];

  /* test the increasing sort */
  printf("Testing idxsorti...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432);

  gk_idxsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] > array[i+1])
      printf("gk_idxsorti error at index %zd [%zd %zd]\n", (ssize_t)i, (ssize_t)array[i], (ssize_t)array[i+1]);
  }


  /* test the decreasing sort */
  printf("Testing idxsortd...\n");
  for (i=0; i<N; i++)
    array[i] = RandomInRange(123432);

  gk_idxsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i] < array[i+1])
      printf("gk_idxsortd error at index %zd [%zd %zd]\n", (ssize_t)i, (ssize_t)array[i], (ssize_t)array[i+1]);
  }

}



/*************************************************************************/
/*! Testing module for gk_?ikvsort() routine */
/*************************************************************************/
void test_ikvsort()
{
  gk_idx_t i;
  gk_ikv_t array[N];

  /* test the increasing sort */
  printf("Testing ikvsorti...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432);
    array[i].val = i;
  }

  gk_ikvsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key > array[i+1].key)
      printf("gk_ikvsorti error at index %jd [%d %d] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }


  /* test the decreasing sort */
  printf("Testing ikvsortd...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432);
    array[i].val = i;
  }

  gk_ikvsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key < array[i+1].key)
      printf("gk_ikvsortd error at index %jd [%d %d] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }

}



/*************************************************************************/
/*! Testing module for gk_?fkvsort() routine */
/*************************************************************************/
void test_fkvsort()
{
  gk_idx_t i;
  gk_fkv_t array[N];

  /* test the increasing sort */
  printf("Testing fkvsorti...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432)/(1.0+RandomInRange(645323));
    array[i].val = i;
  }

  gk_fkvsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key > array[i+1].key)
      printf("gk_fkvsorti error at index %jd [%f %f] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }


  /* test the decreasing sort */
  printf("Testing fkvsortd...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432)/(1.0+RandomInRange(645323));
    array[i].val = i;
  }

  gk_fkvsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key < array[i+1].key)
      printf("gk_fkvsortd error at index %jd [%f %f] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }

}


/*************************************************************************/
/*! Testing module for gk_?dkvsort() routine */
/*************************************************************************/
void test_dkvsort()
{
  gk_idx_t i;
  gk_dkv_t array[N];

  /* test the increasing sort */
  printf("Testing dkvsorti...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432)/(1.0+RandomInRange(645323));
    array[i].val = i;
  }

  gk_dkvsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key > array[i+1].key)
      printf("gk_dkvsorti error at index %jd [%lf %lf] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }


  /* test the decreasing sort */
  printf("Testing dkvsortd...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432)/(1.0+RandomInRange(645323));
    array[i].val = i;
  }

  gk_dkvsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key < array[i+1].key)
      printf("gk_dkvsortd error at index %jd [%lf %lf] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }

}


/*************************************************************************/
/*! Testing module for gk_?skvsort() routine */
/*************************************************************************/
void test_skvsort()
{
  gk_idx_t i;
  gk_skv_t array[N];
  char line[256];

  /* test the increasing sort */
  printf("Testing skvsorti...\n");
  for (i=0; i<N; i++) {
    sprintf(line, "%d", RandomInRange(123432));
    array[i].key = gk_strdup(line);
    array[i].val = i;
  }

  gk_skvsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (strcmp(array[i].key, array[i+1].key) > 0)
      printf("gk_skvsorti error at index %jd [%s %s] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }


  /* test the decreasing sort */
  printf("Testing skvsortd...\n");
  for (i=0; i<N; i++) {
    sprintf(line, "%d", RandomInRange(123432));
    array[i].key = gk_strdup(line);
    array[i].val = i;
  }

  gk_skvsortd(N, array);

  for (i=0; i<N-1; i++) {
    /*printf("%s\n", array[i].key);*/
    if (strcmp(array[i].key, array[i+1].key) < 0)
      printf("gk_skvsortd error at index %jd [%s %s] [%jd %jd]\n", (intmax_t)i, array[i].key, array[i+1].key, (intmax_t)array[i].val, (intmax_t)array[i+1].val);
  }

}


/*************************************************************************/
/*! Testing module for gk_?idxkvsort() routine */
/*************************************************************************/
void test_idxkvsort()
{
  gk_idx_t i;
  gk_idxkv_t array[N];

  /* test the increasing sort */
  printf("Testing idxkvsorti...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432);
    array[i].val = i;
  }

  gk_idxkvsorti(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key > array[i+1].key)
      printf("gk_idxkvsorti error at index %zd [%zd %zd] [%zd %zd]\n", 
          (ssize_t)i, (ssize_t)array[i].key, (ssize_t)array[i+1].key, 
          (ssize_t)array[i].val, (ssize_t)array[i+1].val);
  }


  /* test the decreasing sort */
  printf("Testing idxkvsortd...\n");
  for (i=0; i<N; i++) {
    array[i].key = RandomInRange(123432);
    array[i].val = i;
  }

  gk_idxkvsortd(N, array);

  for (i=0; i<N-1; i++) {
    if (array[i].key < array[i+1].key)
      printf("gk_idxkvsortd error at index %zd [%zd %zd] [%zd %zd]\n", 
          (ssize_t)i, (ssize_t)array[i].key, (ssize_t)array[i+1].key, 
          (ssize_t)array[i].val, (ssize_t)array[i+1].val);
  }

}




int main()
{
  test_isort();
  test_fsort();
  test_idxsort();

  test_ikvsort();
  test_fkvsort();
  test_dkvsort();
  test_skvsort();
  test_idxkvsort();
}

