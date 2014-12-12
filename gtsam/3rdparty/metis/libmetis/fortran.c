/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * fortran.c
 *
 * This file contains code for the fortran to C interface
 *
 * Started 8/19/97
 * George
 *
 */

#include "metislib.h"


/*************************************************************************/
/*! This function changes the numbering to start from 0 instead of 1 */
/*************************************************************************/
void Change2CNumbering(idx_t nvtxs, idx_t *xadj, idx_t *adjncy)
{
  idx_t i;

  for (i=0; i<=nvtxs; i++)
    xadj[i]--;

  for (i=0; i<xadj[nvtxs]; i++)
    adjncy[i]--;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumbering(idx_t nvtxs, idx_t *xadj, idx_t *adjncy, idx_t *vector)
{
  idx_t i;

  for (i=0; i<nvtxs; i++)
    vector[i]++;

  for (i=0; i<xadj[nvtxs]; i++)
    adjncy[i]++;

  for (i=0; i<=nvtxs; i++)
    xadj[i]++;
}

/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumbering2(idx_t nvtxs, idx_t *xadj, idx_t *adjncy)
{
  idx_t i, nedges;

  nedges = xadj[nvtxs];
  for (i=0; i<nedges; i++)
    adjncy[i]++;

  for (i=0; i<=nvtxs; i++)
    xadj[i]++;
}



/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void Change2FNumberingOrder(idx_t nvtxs, idx_t *xadj, idx_t *adjncy, 
         idx_t *v1, idx_t *v2)
{
  idx_t i, nedges;

  for (i=0; i<nvtxs; i++) {
    v1[i]++;
    v2[i]++;
  }

  nedges = xadj[nvtxs];
  for (i=0; i<nedges; i++)
    adjncy[i]++;

  for (i=0; i<=nvtxs; i++)
    xadj[i]++;

}



/*************************************************************************/
/*! This function changes the numbering to start from 0 instead of 1 */
/*************************************************************************/
void ChangeMesh2CNumbering(idx_t n, idx_t *ptr, idx_t *ind)
{
  idx_t i;

  for (i=0; i<=n; i++)
    ptr[i]--;
  for (i=0; i<ptr[n]; i++)
    ind[i]--;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void ChangeMesh2FNumbering(idx_t n, idx_t *ptr, idx_t *ind, idx_t nvtxs, 
         idx_t *xadj, idx_t *adjncy)
{
  idx_t i;

  for (i=0; i<ptr[n]; i++)
    ind[i]++;
  for (i=0; i<=n; i++)
    ptr[i]++;

  for (i=0; i<xadj[nvtxs]; i++)
    adjncy[i]++;
  for (i=0; i<=nvtxs; i++)
    xadj[i]++;
}


/*************************************************************************/
/*! This function changes the numbering to start from 1 instead of 0 */
/*************************************************************************/
void ChangeMesh2FNumbering2(idx_t ne, idx_t nn, idx_t *ptr, idx_t *ind, 
         idx_t *epart, idx_t *npart)
{
  idx_t i;

  for (i=0; i<ptr[ne]; i++)
    ind[i]++;
  for (i=0; i<=ne; i++)
    ptr[i]++;

  for (i=0; i<ne; i++)
    epart[i]++;

  for (i=0; i<nn; i++)
    npart[i]++;
}

