/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * macros.h
 *
 * This file contains macros used in multilevel
 *
 * Started 9/25/94
 * George
 *
 * $Id: macros.h 10060 2011-06-02 18:56:30Z karypis $
 *
 */

#ifndef _LIBMETIS_MACROS_H_
#define _LIBMETIS_MACROS_H_

/*************************************************************************
* The following macro returns a random number in the specified range
**************************************************************************/
#define AND(a, b) ((a) < 0 ? ((-(a))&(b)) : ((a)&(b)))
#define OR(a, b) ((a) < 0 ? -((-(a))|(b)) : ((a)|(b)))
#define XOR(a, b) ((a) < 0 ? -((-(a))^(b)) : ((a)^(b)))

//#define icopy(n, a, b) (idx_t *)memcpy((void *)(b), (void *)(a), sizeof(idx_t)*(n)) 

#define HASHFCT(key, size) ((key)%(size))
#define SWAP gk_SWAP

/* gets the appropriate option value */
#define GETOPTION(options, idx, defval) \
            ((options) == NULL || (options)[idx] == -1 ? defval : (options)[idx]) 

/* converts a user provided ufactor into a real ubfactor */
#define I2RUBFACTOR(ufactor) (1.0+0.001*(ufactor))

/* set/reset the current workspace core */
#define WCOREPUSH    wspacepush(ctrl)
#define WCOREPOP     wspacepop(ctrl)



/*************************************************************************
* These macros insert and remove nodes from a Direct Access list 
**************************************************************************/
#define ListInsert(n, lind, lptr, i) \
   do { \
     ASSERT(lptr[i] == -1); \
     lind[n] = i; \
     lptr[i] = (n)++;\
   } while(0) 

#define ListDelete(n, lind, lptr, i) \
   do { \
     ASSERT(lptr[i] != -1); \
     lind[lptr[i]] = lind[--(n)]; \
     lptr[lind[n]] = lptr[i]; \
     lptr[i] = -1; \
   } while(0) 


/*************************************************************************
* These macros insert and remove nodes from the boundary list
**************************************************************************/
#define BNDInsert(nbnd, bndind, bndptr, vtx) \
  ListInsert(nbnd, bndind, bndptr, vtx)

#define BNDDelete(nbnd, bndind, bndptr, vtx) \
  ListDelete(nbnd, bndind, bndptr, vtx)


/*************************************************************************
* These macros deal with id/ed updating during k-way refinement
**************************************************************************/
#define UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, \
            nbnd, bndptr, bndind, bndtype) \
   do { \
     where[i] = to; \
     myrinfo->ed += myrinfo->id-mynbrs[k].ed; \
     SWAP(myrinfo->id, mynbrs[k].ed, j); \
     if (mynbrs[k].ed == 0) \
       mynbrs[k] = mynbrs[--myrinfo->nnbrs]; \
     else \
       mynbrs[k].pid = from; \
     \
     /* Update the boundary information. Both deletion and addition is \
        allowed as this routine can be used for moving arbitrary nodes. */ \
     if (bndtype == BNDTYPE_REFINE) { \
       if (bndptr[i] != -1 && myrinfo->ed - myrinfo->id < 0) \
         BNDDelete(nbnd, bndind, bndptr, i); \
       if (bndptr[i] == -1 && myrinfo->ed - myrinfo->id >= 0) \
         BNDInsert(nbnd, bndind, bndptr, i); \
     } \
     else { \
       if (bndptr[i] != -1 && myrinfo->ed <= 0) \
         BNDDelete(nbnd, bndind, bndptr, i); \
       if (bndptr[i] == -1 && myrinfo->ed > 0) \
         BNDInsert(nbnd, bndind, bndptr, i); \
     } \
   } while(0) 


#define UpdateAdjacentVertexInfoAndBND(ctrl, vid, adjlen, me, from, to, \
            myrinfo, ewgt, nbnd, bndptr, bndind, bndtype) \
   do { \
     idx_t k; \
     cnbr_t *mynbrs; \
     \
     if (myrinfo->inbr == -1) { \
       myrinfo->inbr  = cnbrpoolGetNext(ctrl, adjlen+1); \
       myrinfo->nnbrs = 0; \
     } \
     ASSERT(CheckRInfo(ctrl, myrinfo)); \
     \
     mynbrs = ctrl->cnbrpool + myrinfo->inbr; \
     \
     /* Update global ID/ED and boundary */ \
     if (me == from) { \
       INC_DEC(myrinfo->ed, myrinfo->id, (ewgt)); \
       if (bndtype == BNDTYPE_REFINE) { \
         if (myrinfo->ed-myrinfo->id >= 0 && bndptr[(vid)] == -1) \
           BNDInsert(nbnd, bndind, bndptr, (vid)); \
       } \
       else { \
         if (myrinfo->ed > 0 && bndptr[(vid)] == -1) \
           BNDInsert(nbnd, bndind, bndptr, (vid)); \
       } \
     } \
     else if (me == to) { \
       INC_DEC(myrinfo->id, myrinfo->ed, (ewgt)); \
       if (bndtype == BNDTYPE_REFINE) { \
         if (myrinfo->ed-myrinfo->id < 0 && bndptr[(vid)] != -1) \
           BNDDelete(nbnd, bndind, bndptr, (vid)); \
       } \
       else { \
         if (myrinfo->ed <= 0 && bndptr[(vid)] != -1) \
           BNDDelete(nbnd, bndind, bndptr, (vid)); \
       } \
     } \
     \
     /* Remove contribution from the .ed of 'from' */ \
     if (me != from) { \
       for (k=0; k<myrinfo->nnbrs; k++) { \
         if (mynbrs[k].pid == from) { \
           if (mynbrs[k].ed == (ewgt)) \
             mynbrs[k] = mynbrs[--myrinfo->nnbrs]; \
           else \
             mynbrs[k].ed -= (ewgt); \
           break; \
         } \
       } \
     } \
     \
     /* Add contribution to the .ed of 'to' */ \
     if (me != to) { \
       for (k=0; k<myrinfo->nnbrs; k++) { \
         if (mynbrs[k].pid == to) { \
           mynbrs[k].ed += (ewgt); \
           break; \
         } \
       } \
       if (k == myrinfo->nnbrs) { \
         mynbrs[k].pid  = to; \
         mynbrs[k].ed   = (ewgt); \
         myrinfo->nnbrs++; \
       } \
     } \
     \
     ASSERT(CheckRInfo(ctrl, myrinfo));\
   } while(0) 


#define UpdateQueueInfo(queue, vstatus, vid, me, from, to, myrinfo, oldnnbrs, \
            nupd, updptr, updind, bndtype) \
   do { \
     real_t rgain; \
     \
     if (me == to || me == from || oldnnbrs != myrinfo->nnbrs) {  \
       rgain = (myrinfo->nnbrs > 0 ?  \
                1.0*myrinfo->ed/sqrt(myrinfo->nnbrs) : 0.0) - myrinfo->id; \
   \
       if (bndtype == BNDTYPE_REFINE) { \
         if (vstatus[(vid)] == VPQSTATUS_PRESENT) { \
           if (myrinfo->ed-myrinfo->id >= 0) \
             rpqUpdate(queue, (vid), rgain); \
           else { \
             rpqDelete(queue, (vid)); \
             vstatus[(vid)] = VPQSTATUS_NOTPRESENT; \
             ListDelete(nupd, updind, updptr, (vid)); \
           } \
         } \
         else if (vstatus[(vid)] == VPQSTATUS_NOTPRESENT && myrinfo->ed-myrinfo->id >= 0) { \
           rpqInsert(queue, (vid), rgain); \
           vstatus[(vid)] = VPQSTATUS_PRESENT; \
           ListInsert(nupd, updind, updptr, (vid)); \
         } \
       } \
       else { \
         if (vstatus[(vid)] == VPQSTATUS_PRESENT) { \
           if (myrinfo->ed > 0) \
             rpqUpdate(queue, (vid), rgain); \
           else { \
             rpqDelete(queue, (vid)); \
             vstatus[(vid)] = VPQSTATUS_NOTPRESENT; \
             ListDelete(nupd, updind, updptr, (vid)); \
           } \
         } \
         else if (vstatus[(vid)] == VPQSTATUS_NOTPRESENT && myrinfo->ed > 0) { \
           rpqInsert(queue, (vid), rgain); \
           vstatus[(vid)] = VPQSTATUS_PRESENT; \
           ListInsert(nupd, updind, updptr, (vid)); \
         } \
       } \
     } \
   } while(0) 



/*************************************************************************/
/*! This macro determines the set of subdomains that a vertex can move to
    without increasins the maxndoms. */
/*************************************************************************/
#define SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, vtmp) \
  do { \
    idx_t j, k, l, nadd, to; \
    for (j=0; j<myrinfo->nnbrs; j++) { \
      safetos[to = mynbrs[j].pid] = 0; \
      \
      /* uncompress the connectivity info for the 'to' subdomain */ \
      for (k=0; k<nads[to]; k++) \
        vtmp[adids[to][k]] = 1; \
      \
      for (nadd=0, k=0; k<myrinfo->nnbrs; k++) { \
        if (k == j) \
          continue; \
        \
        l = mynbrs[k].pid; \
        if (vtmp[l] == 0) { \
          if (nads[l] > maxndoms-1) { \
            nadd = maxndoms; \
            break; \
          } \
          nadd++; \
        } \
      } \
      if (nads[to]+nadd <= maxndoms) \
        safetos[to] = 1; \
      if (nadd == 0) \
        safetos[to] = 2; \
      \
      /* cleanup the connectivity info due to the 'to' subdomain */ \
      for (k=0; k<nads[to]; k++) \
        vtmp[adids[to][k]] = 0; \
    } \
  } while (0)


#endif
