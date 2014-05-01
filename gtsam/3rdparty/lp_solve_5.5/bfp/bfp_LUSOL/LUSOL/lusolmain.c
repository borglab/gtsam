
/*     This program solves a sparse linear system Ax = b using LUSOL. */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "commonlib.h"
#include "myblas.h"
#include "lusol.h"
#include "lusolio.h"
#include "lusolmain.h"

#if (defined WIN32) || (defined WIN64)
void _strupr_(char *s)
{
  _strupr(s);
}
#else
/* Yin Zhang noted that _strupr() is not available on many Unix platforms */
void _strupr_(char *s)
{
  int  i;
  char c;
  for (i = 0; i < strlen(s); i++) {
    c = s[i];
    if (c <= 'z' && c >= 'a') {
      s[i] = c - 'a' + 'A';
    }
  }
}
#endif

MYBOOL getFileName(char *filename, char *test)
{
  MYBOOL status;
  status = (MYBOOL) (('-' != test[0]) && (strlen(test) > 1));
  if(status)
    strcpy(filename, test);
  return(status);
}
MYBOOL isNum(char val)
{
  int ord;
  ord = (int) val - 48;
  return( (MYBOOL) ((ord >= 0) && (ord <= 9)) );
}

void main( int argc, char *argv[], char *envp[] )
{
/* Output device */
  FILE *outunit = stdout;

/* Overall dimensions allocated */
  int    maxm = MAXROWS, maxn = MAXCOLS, maxnz = MAXNZ,
         replace = 0, randcol = 0;
  MYBOOL ftran = TRUE;

/* Storage for A, b */
  REAL   *Aij = NULL, *b = NULL, *xexact = NULL;
  int    *iA = NULL, *jA = NULL;

/* Storage for LUSOL */
  LUSOLrec *LUSOL = NULL;

/* Define local storage variables */
  int  i     , inform, j     , k     , i1   ,
       m     , useropt, lenb, lenx,
       n     , nelem , nnzero;
  REAL Amax  , test  ,
       bnorm , rnorm , xnorm,
       *rhs = NULL, *r = NULL, *x = NULL;
  char fileext[50], filename[255], blasname[255];
  MYBOOL printsolution = FALSE, success = FALSE;

/* Check if we have input parameters */
  useropt = argc;
  if(useropt < 2) {
    printf("LUSOL v%d.%d.%d.%d - Linear equation solver - Development edition.\n",
           LUSOL_VERMAJOR, LUSOL_VERMINOR, LUSOL_RELEASE, LUSOL_BUILD);
    printf("Usage:   lusol [-p <type>] [-t <tolerance>] [-m <memscalar>]\n");
    printf("               [-s] <MatrixFile> [<RhsFile>]\n");
    printf("Options: -p <type>        <0..3>    Selects pivot type.\n");
    printf("         -t <tolerance>   <1..100>  Selects diagonal tolerance.\n");
    printf("         -m <memscalar>   <%d..100> Memory allocation scalar.\n", LUSOL_MULT_nz_a);
    printf("         -b               Solves using btran (rather than ftran).\n");
    printf("         -r <times>       Randomly replace a column and resolve.\n");
    printf("         -s               Show the computed solution.\n");
    printf("         -blas <lib>      Activates external optimized BLAS library.\n");
    printf("Formats: Conventional RCV .TXT, MatrixMarket .MTX, or Harwell-Boeing .RUA\n");
    printf("Author:  Michael A. Saunders (original Fortran version)\n");
    printf("         Kjell Eikland       (modified C version)\n");
    return;
  }

/* Create the LUSOL object and set user options */
  LUSOL = LUSOL_create(outunit, 0, LUSOL_PIVMOD_TPP, 0);
#if 1
  LUSOL->luparm[LUSOL_IP_ACCELERATION] = LUSOL_OTHERORDER | LUSOL_ACCELERATE_L0;
#elif 0
  LUSOL->luparm[LUSOL_IP_ACCELERATION] = LUSOL_AUTOORDER | LUSOL_ACCELERATE_L0;
#endif
  LUSOL->luparm[LUSOL_IP_SCALAR_NZA] = 10;
  i = 1;
  n = 0;
  filename[0] = '\0';
  blasname[0] = '\0';
  while((n == 0) && (i < argc)) {
    if(strcmp("-p", argv[i]) == 0) {
      i1 = i+1;
      if((i1 < argc) && isNum(argv[i1][0])) {
        i = i1;
        m = atoi(argv[i]);
        if(m < 0 || m > LUSOL_PIVMOD_MAX)
          continue;
        LUSOL->luparm[LUSOL_IP_PIVOTTYPE] = m;
      }
    }
    else if(strcmp("-t", argv[i]) == 0) {
      i1 = i+1;
      if((i1 < argc) && isNum(argv[i1][0])) {
        i = i1;
        Amax = atof(argv[i]);
        if(Amax < 1 || Amax > 100)
          continue;
        LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = Amax;
      }
    }
    else if(strcmp("-m", argv[i]) == 0) {
      i1 = i+1;
      if((i1 < argc) && isNum(argv[i1][0])) {
        i = i1;
        m = atoi(argv[i]);
        if(m < LUSOL_MULT_nz_a || m > 100)
          continue;
        LUSOL->luparm[LUSOL_IP_SCALAR_NZA] = atoi(argv[i]);
      }
    }
    else if(strcmp("-s", argv[i]) == 0)
      printsolution = TRUE;
    else if(strcmp("-b", argv[i]) == 0)
      ftran = FALSE;
    else if(strcmp("-r", argv[i]) == 0) {
      i1 = i+1;
      if((i1 < argc) && isNum(argv[i1][0])) {
        i = i1;
        m = atoi(argv[i]);
        if(m < 0 || m > 10)
          continue;
      }
      else
        m = 1;
      srand((unsigned) time( NULL ));
      replace = 2*m;
    }
    else if(strcmp("-blas", argv[i]) == 0) {
      i1 = i+1;
      if((i1 < argc) && getFileName(blasname, argv[i1])) {
        if(!load_BLAS(blasname))
          fprintf(outunit, "Could not load external BLAS library '%s'\n", blasname);
        i = i1;
      }
      else
        fprintf(outunit, "Ignoring incomplete parameter %d '%s'\n", i, argv[i]);
    }
    else {
      if(getFileName(filename, argv[i])) {
        useropt = i;
        break;
      }
      else
        fprintf(outunit, "Ignoring unknown parameter %d '%s'\n", i, argv[i]);
    }
    i++;
  }

/* Obtain file extension and see if we must estimate matrix data size */
  strcpy(fileext, strchr(argv[useropt], '.'));
  /* Yin Zhang noted that _strupr() is not available on many Unix platforms. */
  _strupr_(fileext);
/*  _strupr(fileext);*/

  /* Read conventional text file format */
  if(strcmp(fileext, ".TXT") == 0) {
    if(!ctf_size_A(filename, &maxm, &maxn, &maxnz))
      goto x900;
  }
  /* Read MatrixMarket file format */
  else if(strcmp(fileext, ".MTX") == 0) {
    if(!mmf_size_A(filename, &maxm, &maxn, &maxnz))
      goto x900;
  }
  /* Read Harwell-Boeing file format */
  else if(strcmp(fileext, ".RUA") == 0) {
    if(!hbf_size_A(filename, &maxm, &maxn, &maxnz))
      goto x900;
  }
  else {
    fprintf(outunit, "Unrecognized matrix file extension %s\n", fileext);
    goto x900;
  }

/* Create the arrays */

  Aij = (REAL *) LUSOL_CALLOC(maxnz + BLAS_BASE, sizeof(REAL));
  iA = (int *)   LUSOL_CALLOC(maxnz + BLAS_BASE, sizeof(int));
  jA = (int *)   LUSOL_CALLOC(maxnz + BLAS_BASE, sizeof(int));
  if(ftran)
    lenb = maxm;
  else
    lenb = maxn;
  b   = (REAL *) LUSOL_CALLOC(lenb+BLAS_BASE, sizeof(REAL));
  rhs = (REAL *) LUSOL_CALLOC(lenb+BLAS_BASE, sizeof(REAL));

  if(ftran)
    lenx = maxn;
  else
    lenx = maxm;
  xexact = (REAL *) LUSOL_CALLOC(lenx+BLAS_BASE, sizeof(REAL));
  r = (REAL *) LUSOL_CALLOC(lenx+BLAS_BASE, sizeof(REAL));
  x = (REAL *) LUSOL_CALLOC(lenx+BLAS_BASE, sizeof(REAL));

/* -----------------------------------------------------------------
   Read data files.
   ----------------------------------------------------------------- */
  fprintf(stdout, "\n========================================\n");
  fprintf(stdout,   "LUSOL v%d.%d.%d.%d - Linear equation solver",
                    LUSOL_VERMAJOR, LUSOL_VERMINOR, LUSOL_RELEASE, LUSOL_BUILD);
  fprintf(stdout, "\n========================================\n");

/* -----------------------------------------------------------------
   Read data for A
   ----------------------------------------------------------------- */
  /* Read conventional text file format */
  if(strcmp(fileext, ".TXT") == 0) {
    if(!ctf_read_A(argv[useropt], maxm, maxn, maxnz,
                   &m, &n, &nnzero, iA, jA, Aij))
      goto x900;
  }
  /* Read MatrixMarket file format */
  else if(strcmp(fileext, ".MTX") == 0) {
    if(!mmf_read_A(argv[useropt], maxm, maxn, maxnz,
                   &m, &n, &nnzero, iA, jA, Aij))
      goto x900;
  }
  /* Read Harwell-Boeing file format */
  else if(strcmp(fileext, ".RUA") == 0) {
    if(!hbf_read_A(argv[useropt], maxm, maxn, maxnz,
                   &m, &n, &nnzero, iA, jA, Aij))
      goto x900;
  }
  else {
    fprintf(outunit, "Error: Unrecognized matrix file extension %s\n", fileext);
    goto x900;
  }

/* -----------------------------------------------------------------
   Read data for b
   ----------------------------------------------------------------- */
  /* Handle Harwell-Boeing case where the file contains a RHS */
  i = strcmp(fileext, ".RUA");

  if((useropt == argc-1) && (i != 0)) {
    srand(timeNow());
    i1 = m;
    while(i1 > 0) {
      test = rand();
      i = RAND_MAX;
      i = (int) ((test/i)*(m-1));
/*      b[i+1] = 1.0; */
      b[i+1] = i - 5;
      i1--;
    }
    if(printsolution)
      blockWriteREAL(outunit, "\nGenerated RHS vector", b, 1, lenb);
  }
  else {
    if(i != 0)
      useropt++;
    strcpy(fileext, strchr(argv[useropt], '.'));
    _strupr_(fileext);

    /* Read conventional text file format */
    if(strcmp(fileext, ".TXT") == 0) {
      if(!ctf_read_b(argv[useropt], lenb, b))
        goto x900;
    }
    /* Read MatrixMarket file format */
    else if(strcmp(fileext, ".MTX") == 0) {
      if(!mmf_read_b(argv[useropt], lenb, b))
        goto x900;
    }
  /* Read Harwell-Boeing file format */
    else if(strcmp(fileext, ".RUA") == 0) {
      if(!hbf_read_b(argv[useropt], lenb, b))
        goto x900;
    }
    else {
      fprintf(outunit, "Error: Unrecognized vector file extension %s\n", fileext);
      goto x900;
    }
  }
  success = TRUE;

/* -----------------------------------------------------------------
   Show data on input
   ----------------------------------------------------------------- */
  fprintf(outunit, "\nData read from:\n%s\n", filename);
  test = (double) nnzero / ((double) m * (double) n);
  test *= 100.0;
  fprintf(outunit, "Rows = %d   Columns = %d   Non-zeros = %d  Density =%8.4f%%\n",
                   m, n, nnzero, test);

/* -----------------------------------------------------------------
   Load A into (a, indc, indr).
   ----------------------------------------------------------------- */
#if 0 /* BUG !!! */
  if(n != m)
    LUSOL->luparm[LUSOL_IP_KEEPLU] = FALSE;
#endif
#ifdef LegacyTesting
  LUSOL->luparm[LUSOL_IP_SCALAR_NZA] = LUSOL_MULT_nz_a;
  LUSOL_sizeto(LUSOL, MAXROWS, MAXCOLS, MAXNZ*LUSOL_MULT_nz_a);
#endif

  if(!LUSOL_assign(LUSOL, iA, jA, Aij, nnzero, TRUE)) {
    fprintf(outunit, "Error: LUSOL failed due to insufficient memory.\n");
    goto x900;
  }

/* ------------------------------------------------------------------
   Factor  A = L U.
   ------------------------------------------------------------------ */
  nelem = nnzero;
  inform = LUSOL_factorize( LUSOL);
  if(inform > LUSOL_INFORM_SERIOUS) {
    fprintf(outunit, "Error:\n%s\n", LUSOL_informstr(LUSOL, inform));
    goto x900;
  }
  if(n != m)
    goto x800;

  /* Get the largest element in A; we use it below as an estimate
     of ||A||_inf, even though it isn't a proper norm. */
  Amax = LUSOL->parmlu[LUSOL_RP_MAXELEM_A];

/* ------------------------------------------------------------------
   SOLVE  A x = b  or  x'A = b'.
   Save b first because lu6sol() overwrites the rhs.
   ------------------------------------------------------------------ */
  MEMCOPY(x, b, lenb+BLAS_BASE);

Resolve:
  if(ftran)
    inform = LUSOL_ftran(LUSOL, x, NULL, FALSE);
  else
    inform = LUSOL_btran(LUSOL, x, NULL);
  if(inform > LUSOL_INFORM_SERIOUS) {
    fprintf(outunit, "Error:\n%s\n", LUSOL_informstr(LUSOL, inform));
    goto x900;
  }
  if(printsolution)
    blockWriteREAL(outunit, "\nSolution vector", x, 1, lenb);

/* ------------------------------------------------------------------
   Set r = b - Ax.
   Find norm of r and x.
   ------------------------------------------------------------------ */
  MEMCOPY(r, b, lenb+BLAS_BASE);
  for(k = 1; k <= nnzero; k++) {
    i    = iA[k];  /* Row number    */
    j    = jA[k];  /* Column number */
    if(ftran)
      r[i] -= Aij[k]*x[j];
    else
      r[j] -= Aij[k]*x[i];
  }
/*  blockWriteREAL(outunit, "\nResidual vector", r, 1, lenb);*/
  bnorm  = dnormi( lenb, b );
  rnorm  = dnormi( lenb, r );
  xnorm  = dnormi( lenx, x );

/* ------------------------------------------------------------------
   Report the findings.
   ------------------------------------------------------------------ */
  if(randcol > 0)
    fprintf(outunit, "\n\nColumn %d was %s\n",
                      randcol,
                      (mod(replace,2) == 1 ? "replaced with random data" : "restored"));

x800:
  fprintf(outunit, "\nLU size statistics (%d reallocations):\n",
                   LUSOL->expanded_a);
  test = LUSOL->luparm[LUSOL_IP_NONZEROS_U0]+LUSOL->luparm[LUSOL_IP_NONZEROS_L0];
  fprintf(outunit, "L0-size = %d   U0-size = %d   LU-nonzeros = %d   Fill-in = %.1fx\n",
                   LUSOL->luparm[LUSOL_IP_NONZEROS_L0],
                   LUSOL->luparm[LUSOL_IP_NONZEROS_U0],
                   (int) test, test/nnzero);
  if(n != m) {
    fprintf(outunit, "%s with a factor tol. of %g identified %d singularities.\n",
                     LUSOL_pivotLabel(LUSOL), LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij],
                     LUSOL->luparm[LUSOL_IP_SINGULARITIES]);
    goto x900;
  }

  test   = rnorm / (Amax*xnorm);
  fprintf(outunit, "\nAccuracy statistics:\n");
  fprintf(outunit, "%s with a factor tol. of %g gave a rel.error of %g and %d singularities.\n",
                   LUSOL_pivotLabel(LUSOL), LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij], test,
                   LUSOL->luparm[LUSOL_IP_SINGULARITIES]);
  fprintf(outunit, "Amax = %g   bnorm = %g   rnorm = %g   xnorm = %g\n",
                   Amax, bnorm, rnorm, xnorm);

  fprintf(outunit, "\n");

  if (test <= 1.0e-8)
    fprintf(outunit, "The equations were solved with very high accuracy.\n");
  else if (test <= 1.0e-6)
    fprintf(outunit, "The equations were solved with reasonably good accuracy.\n");
  else {
    if (test <= 1.0e-4)
      fprintf(outunit, "Questionable accuracy; the LU factors may not be good enough.\n");
    else
      fprintf(outunit, "Poor accuracy; the LU factorization probably failed.\n");
    if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TRP)
      fprintf(outunit, "Try a smaller factor tolerance (current is %g).\n",
                       LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij]);
    else
      fprintf(outunit, "Try a smaller factor tolerance and/or TRP pivoting.\n");
  }

 /* Check if we should replace a column and resolve */
  if(replace > 0) {
    replace--;
    MEMCLEAR(x, lenb+BLAS_BASE);
    if(mod(replace, 2) == 1) {
      /* Randomly find a column and replace the column values with random data */
      rnorm   = rand();
      randcol = (int) (n * rnorm / (RAND_MAX+1.0)) + 1;
      for(i = 1; i < m; i++)
        x[i] = Amax * rand() / RAND_MAX;
    }
    else {
      /* Put the previously replaced column back and resolve */
      for (k = 1; k <= nnzero; k++) {
        i    = iA[k];
        j    = jA[k];
        if(j == randcol)
          x[i] = Aij[k];
      }
    }
    inform = LUSOL_replaceColumn(LUSOL, randcol, x);
    MEMCOPY(b, x, lenb+BLAS_BASE);
    if(inform != LUSOL_INFORM_LUSUCCESS)
      fprintf(outunit, "Error:\n%s\n", LUSOL_informstr(LUSOL, inform));
    else
      goto Resolve;
  }


/* Free memory */
x900:
  if(!success)
    fprintf(outunit, "Insufficient memory or data file not found.\n");

  LUSOL_FREE(Aij);
  LUSOL_FREE(b);
  LUSOL_FREE(xexact);
  LUSOL_FREE(iA);
  LUSOL_FREE(jA);

  LUSOL_FREE(rhs);
  LUSOL_FREE(r);
  LUSOL_FREE(x);

#if 0
  LUSOL_dump(NULL, LUSOL);
 -blas "atlas_AXP_512_360.dll" -b -p 1 "STP3D_A.MTX"
 "C:\Shared Files\Visual Studio Projects\LU\MatrixMarket\sherman5.mtx" "C:\Shared Files\Visual Studio Projects\LU\MatrixMarket\sherman5_rhs1.mtx"
 A6805.txt b6805.txt
 A10009.txt b10009.txt
 fidap005.mtx fidap005_rhs1.mtx
 fidapm05.mtx fidapm05_rhs1.mtx
 -b -p 1 "basis.mtx"
 -b -p 1 "LU-test3.mtx"
#endif

  LUSOL_free(LUSOL);

/*     End of main program for Test of LUSOL */
}

