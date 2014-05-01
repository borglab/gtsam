#ifndef HEADER_lusolio
#define HEADER_lusolio

/* Include necessary libraries                                               */
/* ------------------------------------------------------------------------- */
#include "lusol.h"

MYBOOL ctf_read_A(char *filename, int maxm, int maxn, int maxnz,
                  int *m, int *n, int *nnzero, int *iA, int *jA, REAL *Aij);
MYBOOL ctf_size_A(char *filename, int *m, int *n, int *nnzero);
MYBOOL ctf_read_b(char *filename, int m, REAL *b);

MYBOOL mmf_read_A(char *filename, int maxM, int maxN, int maxnz,
                  int *M, int *N, int *nz, int *iA, int *jA, REAL *Aij);
MYBOOL mmf_size_A(char *filename, int *M, int *N, int *nz);
MYBOOL mmf_read_b(char *filename, int m, REAL *b);

MYBOOL hbf_read_A(char *filename, int maxM, int maxN, int maxnz,
                  int *M, int *N, int *nz, int *iA, int *jA, REAL *Aij);
MYBOOL hbf_size_A(char *filename, int *M, int *N, int *nz);
MYBOOL hbf_read_b(char *filename, int m, REAL *b);


#endif /* HEADER_lusolio */
