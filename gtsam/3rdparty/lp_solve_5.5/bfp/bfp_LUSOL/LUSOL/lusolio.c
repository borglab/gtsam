
#include <stdio.h>
#include <string.h>
#include "mmio.h"
#include "hbio.h"
#include "lusolio.h"

/* Utility routines to read matrix files in the Coordinate Text File format*/

MYBOOL ctf_read_A(char *filename, int maxm, int maxn, int maxnz,
                  int *m, int *n, int *nnzero, int *iA, int *jA, REAL *Aij)
{
  FILE *iofile;
  int  eof;
  char buffer[100];
  int  k, i, j;
  REAL Ak;
  MYBOOL filldata;

  *nnzero = 0;
  *m      = 0;
  *n      = 0;

  iofile = fopen(filename, "r" );
  if(iofile == NULL) {
    printf("A file %s does not exist\n", filename);
    return( FALSE );
  }

  filldata = (MYBOOL) !((iA == NULL) && (jA == NULL) && (Aij == NULL));
  eof = TRUE;
  for (k = 1; k <= maxnz; k++) {
    eof = feof(iofile);
    if(eof)
      break;
    eof = fscanf(iofile, "%d %d %s", &i, &j, buffer);
    if(eof == 0 || eof == EOF || i <= 0 || j <= 0 || strlen(buffer) == 0)
      break;
    Ak = atof(buffer);
    (*nnzero)++;
    if (filldata) {
      iA[k]  = i;
      jA[k]  = j;
      Aij[k] = Ak;
    }
    if (i > *m) *m = i;
    if (j > *n) *n = j;
  }
  fclose( iofile );
  if(!eof) {
    printf("Too much data in A file.  Increase maxnz\n");
    printf("Current maxnz = %d\n", maxnz);
    return( FALSE );
  }
  printf("A  read successfully\n");
  printf("m      = %d   n      = %d   nnzero = %d\n",
          *m, *n, *nnzero);
  if (*m > maxm  ||  *n > maxn) {
    printf("However, matrix dimensions exceed maxm or maxn\n");
    return( FALSE );
  }
  return( TRUE );
}

MYBOOL ctf_size_A(char *filename, int *m, int *n, int *nnzero)
{
  int maxint = 200000000;

  return( ctf_read_A(filename, maxint, maxint, maxint,
                     m, n, nnzero, NULL, NULL, NULL) );
}

MYBOOL ctf_read_b(char *filename, int m, REAL *b)
{
  FILE *iofile;
  int  eof;
  char buffer[100];
  int  i;

  iofile = fopen(filename, "r");
  if(iofile == NULL) {
    printf("b file %s does not exist\n", filename);
    return( FALSE );
  }

  for (i = 1; i <= m; i++) {
    if(feof(iofile))
      goto x350;
    eof = fscanf(iofile, "%s", buffer);
    if(eof == 0 || eof == EOF)
      goto x350;
    b[i] = atof(buffer);
  }

  fclose( iofile );
  printf("b  read successfully\n");
  return( TRUE );

x350:
  fclose( iofile );
  printf("Not enough data in b file.\n");
  return( FALSE );
}


/* Utility routines to read matrix files in the MatrixMarket format*/
#define mmf_recsize 255
MYBOOL mmf_read_A(char *filename, int maxM, int maxN, int maxnz,
                  int *M, int *N, int *nz, int *iA, int *jA, REAL *Aij)
{
  MM_typecode matcode;
  FILE   *f;
  int    i, k, ret_code, ival, jval;
  REAL   Aval;
  MYBOOL status = FALSE, ispat, filldata;
  char   buf[mmf_recsize];

  f = fopen(filename, "r");
  if(f == NULL) 
    return( status );

  if(mm_read_banner(f, &matcode) != 0) {
    printf("Could not process Matrix Market banner.\n");
    goto x900;
  }

  /*  Screen matrix types since LUSOL only supports a 
      subset of the Matrix Market data types. */
  if(mm_is_complex(matcode) || mm_is_pattern(matcode)) {
    printf("Sorry, this application does not support ");
    printf("Market Market type: [%s]\n", mm_typecode_to_str(matcode));
    goto x900;
  }

  /* Verify that we have sufficient array storage */
  filldata = (MYBOOL) !((iA == NULL) && (jA == NULL) && (Aij == NULL));
  if(filldata && maxN > 1 && jA == NULL) {
    printf("Market Market insufficient array storage specified\n");
    goto x900;
  }

  /* Find out size of sparse matrix .... */
  ret_code = mm_read_mtx_crd_size(f, M, N, nz);
  if(ret_code != 0 || !filldata || (*M > maxM) || (*N > maxN) || (*nz > maxnz)) {
    status = !filldata;
    goto x900;
  }


  /* NOTE: when reading in doubles, ANSI C requires the use of the "l"  */
  /* specifier as in "%lg", "%lf", "%le", otherwise errors will occur   */
  /*  (ANSI C X3.159-1989, Sec. 4.9.6.2, p. 136 lines 13-15)            */

  /* Read dense matrix in column order */
  ispat = (MYBOOL) mm_is_pattern(matcode);
  k = 1;
  if(mm_is_dense(matcode)) {
    maxN = MIN(maxN, *N);
    for (jval = 1; jval <= maxN; jval++) {
      for (i = 1; i <= *M; i++) {
        if(fgets(buf, mmf_recsize-1, f) == NULL)
          break;
        if(sscanf(buf, "%lg\n", &Aval) == 0)
          break;
        if(Aval != 0) {
          if(iA != NULL)
            iA[k] = i;
          if(jA != NULL)
            jA[k] = jval;

          /* Make sure we handle dense vector reading properly */
          if(iA == NULL && jA == NULL)
            Aij[i] = Aval;
          else
            Aij[k] = Aval;
          k++;
        }
      }
    }
  }
  /* Read sparse matrix by coordinate */
  else {
    for (i = 1; i <= *nz; i++) {
      if(fgets(buf, mmf_recsize-1, f) == NULL)
        break;
      if(buf[0] == '%')
        continue;
      if(ispat) {
        if(sscanf(buf, "%d %d\n", &ival, &jval) == 0)
          continue;
        Aij[k] = 1.0;
      }
      else
        if(sscanf(buf, "%d %d %lg\n", &ival, &jval, &Aval) == 0)
          continue;

      /* Check if it is a nonzero and we are within column dimension */
      if(Aval != 0 && jval <= maxN) {
        Aij[k] = Aval;
        if(iA != NULL)
          iA[k] = ival;
        if(jA != NULL)
          jA[k] = jval;
        k++;
      }
    }
  }
  *nz = k - 1;

  /* Handle case where only the lower triangular parts are given */
  if(!mm_is_general(matcode)) {
    if((M != N) || (maxN != maxM) || (2*(*nz) > maxnz)) {
      printf("Market Market cannot fill in symmetry data\n");
      goto x900;
    }
    ispat = mm_is_skew(matcode);
    for(i = 1; i <= *nz; i++) {
      iA[k] = jA[i];
      jA[k] = iA[i];
      if(ispat)
        Aij[k] = -Aij[i];
      else
        Aij[k] = Aij[i];
      k++;
    }
    *nz = k - 1;
  }
  status = TRUE;

  /* Finish up */
x900:
  fclose( f );
  return( status );
}

MYBOOL mmf_size_A(char *filename, int *M, int *N, int *nz)
{
  int maxint = 200000000;

  return( mmf_read_A(filename, maxint, maxint, maxint,
                     M, N, nz, NULL, NULL, NULL) );
}

MYBOOL mmf_read_b(char *filename, int m, REAL *b)
{
  int im, jn, nnzero;

  return( mmf_read_A(filename, m, 1, m,
                     &im, &jn, &nnzero, NULL, NULL, b));
}


/* Utility routines to read matrix files in Harwell-Boeing format*/

MYBOOL hbf_read_A(char *filename, int maxM, int maxN, int maxnz,
                  int *M, int *N, int *nz, int *iA, int *jA, REAL *Aij)
{
  MYBOOL success;

  success = hbf_size_A(filename, M, N, nz);
  if(!success)
    return( success );

  Aij[1] = 0;
  success = (MYBOOL) readHB_mat_double(filename, jA, iA-1, Aij-1);

  /* Test if we have a pattern matrix and fill it with all zeros */
  if(Aij[1] == 0) {
    int i;
    for(i = 1; i <= *nz; i++)
      Aij[i] = 1;
  }

  /* Expand the column nz counts to triplet format */
  if(success) {
    int i, j, ii, k;
    k = *nz;
    for(j = *N; j > 0; j--) {
      ii = jA[j];
      for(i = jA[j-1]; i < ii; i++, k--)
        jA[k] = j;
    }
  }
  return( success );
}

MYBOOL hbf_size_A(char *filename, int *M, int *N, int *nz)
{
  int  Nrhs;
  char *Type;

  return( (MYBOOL) readHB_info(filename, M, N, nz, &Type, &Nrhs) );
}

MYBOOL hbf_read_b(char *filename, int m, REAL *b)
{
  return( (MYBOOL) readHB_aux_double(filename, 'F', b) ); /* Same format as matrix */
}
