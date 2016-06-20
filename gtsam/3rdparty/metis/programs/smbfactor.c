/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * smbfactor.c
 *
 * This file performs the symbolic factorization of a matrix
 *
 * Started 8/1/97
 * George
 *
 * $Id: smbfactor.c 10154 2011-06-09 21:27:35Z karypis $
 *
 */

#include "metisbin.h"


/*************************************************************************/
/*! This function sets up data structures for fill-in computations */
/*************************************************************************/
void ComputeFillIn(graph_t *graph, idx_t *perm, idx_t *iperm, 
         size_t *r_maxlnz, size_t *r_opc)
{
  idx_t i, j, k, nvtxs, maxlnz, maxsub;
  idx_t *xadj, *adjncy;
  idx_t *xlnz, *xnzsub, *nzsub;
  size_t opc;

/*
  printf("\nSymbolic factorization... --------------------------------------------\n");
*/

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  maxsub = 8*(nvtxs+xadj[nvtxs]);

  /* Relabel the vertices so that it starts from 1 */
  for (i=0; i<xadj[nvtxs]; i++)
    adjncy[i]++;
  for (i=0; i<nvtxs+1; i++)
    xadj[i]++;
  for (i=0; i<nvtxs; i++) {
    iperm[i]++;
    perm[i]++;
  }

  /* Allocate the required memory */
  xlnz   = imalloc(nvtxs+2, "ComputeFillIn: xlnz");
  xnzsub = imalloc(nvtxs+2, "ComputeFillIn: xnzsub");
  nzsub  = imalloc(maxsub+1, "ComputeFillIn: nzsub");

  
  /* Call sparspak's routine. */
  if (smbfct(nvtxs, xadj, adjncy, perm, iperm, xlnz, &maxlnz, xnzsub, nzsub, &maxsub)) {
    printf("Realocating nzsub...\n");
    gk_free((void **)&nzsub, LTERM);

    maxsub *= 2;
    nzsub  = imalloc(maxsub+1, "ComputeFillIn: nzsub");
    if (smbfct(nvtxs, xadj, adjncy, perm, iperm, xlnz, &maxlnz, xnzsub, nzsub, &maxsub)) 
      errexit("MAXSUB is too small!");
  }

  for (i=0; i<nvtxs; i++)
    xlnz[i]--;
  for (opc=0, i=0; i<nvtxs; i++)
    opc += (xlnz[i+1]-xlnz[i])*(xlnz[i+1]-xlnz[i]) - (xlnz[i+1]-xlnz[i]);

  *r_maxlnz = maxlnz;
  *r_opc    = opc;

  gk_free((void **)&xlnz, &xnzsub, &nzsub, LTERM);

  /* Relabel the vertices so that it starts from 0 */
  for (i=0; i<nvtxs; i++) {
    iperm[i]--;
    perm[i]--;
  }
  for (i=0; i<nvtxs+1; i++)
    xadj[i]--;
  for (i=0; i<xadj[nvtxs]; i++)
    adjncy[i]--;

}



/*************************************************************************/
/*!
  PURPOSE - THIS ROUTINE PERFORMS SYMBOLIC FACTORIZATION               
  ON A PERMUTED LINEAR SYSTEM AND IT ALSO SETS UP THE               
  COMPRESSED DATA STRUCTURE FOR THE SYSTEM.                         

  INPUT PARAMETERS -                                               
     NEQNS - NUMBER OF EQUATIONS.                                 
     (XADJ, ADJNCY) - THE ADJACENCY STRUCTURE.                   
     (PERM, INVP) - THE PERMUTATION VECTOR AND ITS INVERSE.     

  UPDATED PARAMETERS -                                         
     MAXSUB - SIZE OF THE SUBSCRIPT ARRAY NZSUB.  ON RETURN,  
            IT CONTAINS THE NUMBER OF SUBSCRIPTS USED        

  OUTPUT PARAMETERS -                                       
     XLNZ - INDEX INTO THE NONZERO STORAGE VECTOR LNZ.   
     (XNZSUB, NZSUB) - THE COMPRESSED SUBSCRIPT VECTORS. 
     MAXLNZ - THE NUMBER OF NONZEROS FOUND.             
*/
/*************************************************************************/
idx_t smbfct(idx_t neqns, idx_t *xadj, idx_t *adjncy, idx_t *perm, idx_t *invp, 
	       idx_t *xlnz, idx_t *maxlnz, idx_t *xnzsub, idx_t *nzsub, 
               idx_t *maxsub)
{
  /* Local variables */
  idx_t node, rchm, mrgk, lmax, i, j, k, m, nabor, nzbeg, nzend;
  idx_t kxsub, jstop, jstrt, mrkflg, inz, knz, flag;
  idx_t *mrglnk, *marker, *rchlnk;

  rchlnk = ismalloc(neqns+1, 0, "smbfct: rchlnk");
  marker = ismalloc(neqns+1, 0, "smbfct: marker");
  mrglnk = ismalloc(neqns+1, 0, "smbfct: mgrlnk");

  /* Parameter adjustments */
  --marker;
  --mrglnk;
  --rchlnk;
  --nzsub;
  --xnzsub;
  --xlnz;
  --invp;
  --perm;
  --adjncy;
  --xadj;

  /* Function Body */
  flag    = 0;
  nzbeg   = 1;
  nzend   = 0;
  xlnz[1] = 1;

  /* FOR EACH COLUMN KNZ COUNTS THE NUMBER OF NONZEROS IN COLUMN K ACCUMULATED IN RCHLNK. */
  for (k=1; k<=neqns; k++) {
    xnzsub[k] = nzend;
    node      = perm[k];
    knz       = 0;
    mrgk      = mrglnk[k];
    mrkflg    = 0;
    marker[k] = k;
    if (mrgk != 0) {
      assert(mrgk > 0 && mrgk <= neqns);
      marker[k] = marker[mrgk];
    }

    if (xadj[node] >= xadj[node+1]) {
      xlnz[k+1] = xlnz[k];
      continue;
    }

    /* USE RCHLNK TO LINK THROUGH THE STRUCTURE OF A(*,K) BELOW DIAGONAL */
    assert(k <= neqns && k > 0);
    rchlnk[k] = neqns+1;
    for (j=xadj[node]; j<xadj[node+1]; j++) {
      nabor = invp[adjncy[j]];
      if (nabor <= k) 
        continue;
      rchm = k;

      do {
        m    = rchm;
        assert(m > 0 && m <= neqns);
        rchm = rchlnk[m];
      } while (rchm <= nabor); 

      knz++;
      assert(m > 0 && m <= neqns);
      rchlnk[m]     = nabor;
      assert(nabor > 0 && nabor <= neqns);
      rchlnk[nabor] = rchm;
      assert(k > 0 && k <= neqns);
      if (marker[nabor] != marker[k]) 
        mrkflg = 1;
    }


    /* TEST FOR MASS SYMBOLIC ELIMINATION */
    lmax = 0;
    assert(mrgk >= 0 && mrgk <= neqns);
    if (mrkflg != 0 || mrgk == 0 || mrglnk[mrgk] != 0) 
      goto L350;
    xnzsub[k] = xnzsub[mrgk] + 1;
    knz = xlnz[mrgk + 1] - (xlnz[mrgk] + 1);
    goto L1400;


L350:
    /* LINK THROUGH EACH COLUMN I THAT AFFECTS L(*,K) */
    i = k;
    assert(i > 0 && i <= neqns);
    while ((i = mrglnk[i]) != 0) {
      assert(i > 0 && i <= neqns);
      inz   = xlnz[i+1] - (xlnz[i]+1);
      jstrt = xnzsub[i] + 1;
      jstop = xnzsub[i] + inz;

      if (inz > lmax) { 
        lmax      = inz;
        xnzsub[k] = jstrt;
      }

      /* MERGE STRUCTURE OF L(*,I) IN NZSUB INTO RCHLNK. */ 
      rchm = k;
      for (j=jstrt; j<=jstop; j++) {
        nabor = nzsub[j];
        do {
          m    = rchm;
          assert(m > 0 && m <= neqns);
          rchm = rchlnk[m];
        } while (rchm < nabor);

        if (rchm != nabor) {
          knz++;
          assert(m > 0 && m <= neqns);
          rchlnk[m]     = nabor;
          assert(nabor > 0 && nabor <= neqns);
          rchlnk[nabor] = rchm;
          rchm = nabor;
        }
      }
    }


    /* CHECK IF SUBSCRIPTS DUPLICATE THOSE OF ANOTHER COLUMN */
    if (knz == lmax) 
      goto L1400;

    /* OR IF TAIL OF K-1ST COLUMN MATCHES HEAD OF KTH */
    if (nzbeg > nzend) 
      goto L1200;

    assert(k > 0 && k <= neqns);
    i = rchlnk[k];
    for (jstrt = nzbeg; jstrt <= nzend; ++jstrt) {
      if (nzsub[jstrt] < i) 
        continue;

      if (nzsub[jstrt] == i) 
        goto L1000;
      else 
        goto L1200;
    }
    goto L1200;


L1000:
    xnzsub[k] = jstrt;
    for (j = jstrt; j <= nzend; ++j) {
      if (nzsub[j] != i) 
        goto L1200;
      
      assert(i > 0 && i <= neqns);
      i = rchlnk[i];
      if (i > neqns) 
        goto L1400;
    }
    nzend = jstrt - 1;


    /* COPY THE STRUCTURE OF L(*,K) FROM RCHLNK TO THE DATA STRUCTURE (XNZSUB, NZSUB) */
L1200:
    nzbeg = nzend + 1;
    nzend += knz;

    if (nzend >= *maxsub) {
      flag = 1; /* Out of memory */
      break;
    }

    i = k;
    for (j=nzbeg; j<=nzend; j++) {
      assert(i > 0 && i <= neqns);
      i = rchlnk[i];
      nzsub[j]  = i;
      assert(i > 0 && i <= neqns);
      marker[i] = k;
    }
    xnzsub[k] = nzbeg;
    assert(k > 0 && k <= neqns);
    marker[k] = k;

    /*
     * UPDATE THE VECTOR MRGLNK.  NOTE COLUMN L(*,K) JUST FOUND   
     * IS REQUIRED TO DETERMINE COLUMN L(*,J), WHERE              
     * L(J,K) IS THE FIRST NONZERO IN L(*,K) BELOW DIAGONAL.      
     */
L1400:
    if (knz > 1) { 
      kxsub = xnzsub[k];
      i = nzsub[kxsub];
      assert(i > 0 && i <= neqns);
      assert(k > 0 && k <= neqns);
      mrglnk[k] = mrglnk[i];
      mrglnk[i] = k;
    }

    xlnz[k + 1] = xlnz[k] + knz;
  }

  if (flag == 0) {
    *maxlnz = xlnz[neqns] - 1;
    *maxsub = xnzsub[neqns];
    xnzsub[neqns + 1] = xnzsub[neqns];
  }


  marker++;
  mrglnk++;
  rchlnk++;
  nzsub++;
  xnzsub++;
  xlnz++;
  invp++;
  perm++;
  adjncy++;
  xadj++;

  gk_free((void **)&rchlnk, &mrglnk, &marker, LTERM);

  return flag;
  
} 

