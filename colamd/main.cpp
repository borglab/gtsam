#include "iostream"
#include "colamd.h"

using namespace std;

#define ALEN 100

void use_colamd()
{
  int A [ALEN] = {0, 1, 4, 2, 4, 0, 1, 2, 3, 1, 3} ;
  int p [ ] = {0, 3, 5, 9, 11} ;
  int stats [COLAMD_STATS] ;
  colamd (5, 4, ALEN, A, p, (double *) NULL, stats) ;
  for(int i = 0; i < 5; i++)
    printf("%d ", p[i]);
  printf("\n");
  for(int i = 0; i < COLAMD_STATS; i++)
    printf("%d ", stats[i]);
  printf("\n");
}

int main()
{
  /*
    A:
      [0 x x 0 x
       x x 0 0 x
       x 0 0 x x
       0 0 x 0 0
       x x x 0 x
                      ]
   */
  //int A [ALEN] = {0, 3, 2, 3, 1, 2, 0, 1, 3, 4, 3} ;
  //int p [ ] = {0, 2, 4, 6, 10, 11} ;
  
  use_colamd();
  return 0;
}
