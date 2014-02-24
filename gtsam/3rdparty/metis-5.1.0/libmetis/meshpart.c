/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * meshpart.c
 *
 * This file contains routines for partitioning finite element meshes.
 *
 * Started 9/29/97
 * George
 *
 * $Id: meshpart.c 13931 2013-03-29 16:48:48Z karypis $
 *
 */

#include "metislib.h"


/*************************************************************************
* This function partitions a finite element mesh by partitioning its nodal
* graph using KMETIS and then assigning elements in a load balanced fashion.
**************************************************************************/
int METIS_PartMeshNodal(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind, 
          idx_t *vwgt, idx_t *vsize, idx_t *nparts, real_t *tpwgts, 
          idx_t *options, idx_t *objval, idx_t *epart, idx_t *npart)
{
  int sigrval=0, renumber=0, ptype;
  idx_t *xadj=NULL, *adjncy=NULL;
  idx_t ncon=1, pnumflag=0;
  int rstatus=METIS_OK;

  /* set up malloc cleaning code and signal catchers */
  if (!gk_malloc_init()) 
    return METIS_ERROR_MEMORY;

  gk_sigtrap();

  if ((sigrval = gk_sigcatch()) != 0) 
    goto SIGTHROW;

  renumber = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
  ptype    = GETOPTION(options, METIS_OPTION_PTYPE, METIS_PTYPE_KWAY);

  /* renumber the mesh */
  if (renumber) {
    ChangeMesh2CNumbering(*ne, eptr, eind);
    options[METIS_OPTION_NUMBERING] = 0;
  }

  /* get the nodal graph */
  rstatus = METIS_MeshToNodal(ne, nn, eptr, eind, &pnumflag, &xadj, &adjncy);
  if (rstatus != METIS_OK)
    raise(SIGERR);

  /* partition the graph */
  if (ptype == METIS_PTYPE_KWAY) 
    rstatus = METIS_PartGraphKway(nn, &ncon, xadj, adjncy, vwgt, vsize, NULL, 
                  nparts, tpwgts, NULL, options, objval, npart);
  else 
    rstatus = METIS_PartGraphRecursive(nn, &ncon, xadj, adjncy, vwgt, vsize, NULL, 
                  nparts, tpwgts, NULL, options, objval, npart);

  if (rstatus != METIS_OK)
    raise(SIGERR);

  /* partition the other side of the mesh */
  InduceRowPartFromColumnPart(*ne, eptr, eind, epart, npart, *nparts, tpwgts);


SIGTHROW:
  if (renumber) {
    ChangeMesh2FNumbering2(*ne, *nn, eptr, eind, epart, npart);
    options[METIS_OPTION_NUMBERING] = 1;
  }

  METIS_Free(xadj);
  METIS_Free(adjncy);

  gk_siguntrap();
  gk_malloc_cleanup(0);

  return metis_rcode(sigrval);
}



/*************************************************************************
* This function partitions a finite element mesh by partitioning its dual
* graph using KMETIS and then assigning nodes in a load balanced fashion.
**************************************************************************/
int METIS_PartMeshDual(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind, 
          idx_t *vwgt, idx_t *vsize, idx_t *ncommon, idx_t *nparts, 
          real_t *tpwgts, idx_t *options, idx_t *objval, idx_t *epart, 
          idx_t *npart) 
{
  int sigrval=0, renumber=0, ptype;
  idx_t i, j;
  idx_t *xadj=NULL, *adjncy=NULL, *nptr=NULL, *nind=NULL;
  idx_t ncon=1, pnumflag=0;
  int rstatus = METIS_OK;

  /* set up malloc cleaning code and signal catchers */
  if (!gk_malloc_init()) 
    return METIS_ERROR_MEMORY;

  gk_sigtrap();

  if ((sigrval = gk_sigcatch()) != 0) 
    goto SIGTHROW;

  renumber = GETOPTION(options, METIS_OPTION_NUMBERING, 0);
  ptype    = GETOPTION(options, METIS_OPTION_PTYPE, METIS_PTYPE_KWAY);

  /* renumber the mesh */
  if (renumber) {
    ChangeMesh2CNumbering(*ne, eptr, eind);
    options[METIS_OPTION_NUMBERING] = 0;
  }

  /* get the dual graph */
  rstatus = METIS_MeshToDual(ne, nn, eptr, eind, ncommon, &pnumflag, &xadj, &adjncy);
  if (rstatus != METIS_OK)
    raise(SIGERR);

  /* partition the graph */
  if (ptype == METIS_PTYPE_KWAY) 
    rstatus = METIS_PartGraphKway(ne, &ncon, xadj, adjncy, vwgt, vsize, NULL, 
                  nparts, tpwgts, NULL, options, objval, epart);
  else 
    rstatus = METIS_PartGraphRecursive(ne, &ncon, xadj, adjncy, vwgt, vsize, NULL, 
                  nparts, tpwgts, NULL, options, objval, epart);

  if (rstatus != METIS_OK)
    raise(SIGERR);


  /* construct the node-element list */
  nptr = ismalloc(*nn+1, 0, "METIS_PartMeshDual: nptr");
  nind = imalloc(eptr[*ne], "METIS_PartMeshDual: nind");

  for (i=0; i<*ne; i++) {
    for (j=eptr[i]; j<eptr[i+1]; j++)
      nptr[eind[j]]++;
  }
  MAKECSR(i, *nn, nptr);

  for (i=0; i<*ne; i++) {
    for (j=eptr[i]; j<eptr[i+1]; j++)
      nind[nptr[eind[j]]++] = i;
  }
  SHIFTCSR(i, *nn, nptr);

  /* partition the other side of the mesh */
  InduceRowPartFromColumnPart(*nn, nptr, nind, npart, epart, *nparts, tpwgts);

  gk_free((void **)&nptr, &nind, LTERM);


SIGTHROW:
  if (renumber) {
    ChangeMesh2FNumbering2(*ne, *nn, eptr, eind, epart, npart);
    options[METIS_OPTION_NUMBERING] = 1;
  }

  METIS_Free(xadj);
  METIS_Free(adjncy);

  gk_siguntrap();
  gk_malloc_cleanup(0);

  return metis_rcode(sigrval);
}



/*************************************************************************/
/*! Induces a partitioning of the rows based on a a partitioning of the
    columns. It is used by both the Nodal and Dual routines. */
/*************************************************************************/
void InduceRowPartFromColumnPart(idx_t nrows, idx_t *rowptr, idx_t *rowind,
         idx_t *rpart, idx_t *cpart, idx_t nparts, real_t *tpwgts)
{
  idx_t i, j, k, me;
  idx_t nnbrs, *pwgts, *nbrdom, *nbrwgt, *nbrmrk;
  idx_t *itpwgts;

  pwgts  = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: pwgts");
  nbrdom = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: nbrdom");
  nbrwgt = ismalloc(nparts, 0, "InduceRowPartFromColumnPart: nbrwgt");
  nbrmrk = ismalloc(nparts, -1, "InduceRowPartFromColumnPart: nbrmrk");

  iset(nrows, -1, rpart);

  /* setup the integer target partition weights */
  itpwgts = imalloc(nparts, "InduceRowPartFromColumnPart: itpwgts");
  if (tpwgts == NULL) {
    iset(nparts, 1+nrows/nparts, itpwgts);
  }
  else {
    for (i=0; i<nparts; i++)
      itpwgts[i] = 1+nrows*tpwgts[i];
  }

  /* first assign the rows consisting only of columns that belong to 
     a single partition. Assign rows that are empty to -2 (un-assigned) */
  for (i=0; i<nrows; i++) {
    if (rowptr[i+1]-rowptr[i] == 0) {
      rpart[i] = -2;
      continue;
    }

    me = cpart[rowind[rowptr[i]]];
    for (j=rowptr[i]+1; j<rowptr[i+1]; j++) {
      if (cpart[rowind[j]] != me)
        break;
    }
    if (j == rowptr[i+1]) {
      rpart[i] = me;
      pwgts[me]++;
    }
  }

  /* next assign the rows consisting of columns belonging to multiple
     partitions in a  balanced way */
  for (i=0; i<nrows; i++) {
    if (rpart[i] == -1) { 
      for (nnbrs=0, j=rowptr[i]; j<rowptr[i+1]; j++) {
        me = cpart[rowind[j]];
        if (nbrmrk[me] == -1) {
          nbrdom[nnbrs] = me; 
          nbrwgt[nnbrs] = 1; 
          nbrmrk[me] = nnbrs++;
        }
        else {
          nbrwgt[nbrmrk[me]]++;
        }
      }
      ASSERT(nnbrs > 0);

      /* assign it first to the domain with most things in common */
      rpart[i] = nbrdom[iargmax(nnbrs, nbrwgt)];

      /* if overweight, assign it to the light domain */
      if (pwgts[rpart[i]] > itpwgts[rpart[i]]) {
        for (j=0; j<nnbrs; j++) {
          if (pwgts[nbrdom[j]] < itpwgts[nbrdom[j]] ||
              pwgts[nbrdom[j]]-itpwgts[nbrdom[j]] < pwgts[rpart[i]]-itpwgts[rpart[i]]) {
            rpart[i] = nbrdom[j];
            break;
          }
        }
      }
      pwgts[rpart[i]]++;

      /* reset nbrmrk array */
      for (j=0; j<nnbrs; j++) 
        nbrmrk[nbrdom[j]] = -1;
    }
  }

  gk_free((void **)&pwgts, &nbrdom, &nbrwgt, &nbrmrk, &itpwgts, LTERM);

}
