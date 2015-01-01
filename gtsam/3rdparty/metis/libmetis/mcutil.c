/*
 * mutil.c 
 *
 * This file contains various utility functions for the MOC portion of the
 * code
 *
 * Started 2/15/98
 * George
 *
 * $Id: mcutil.c 13901 2013-03-24 16:17:03Z karypis $
 *
 */

#include "metislib.h"


/*************************************************************************/
/*! This function compares two vectors x & y and returns true 
    if \forall i, x[i] <= y[i].
*/
/**************************************************************************/
int rvecle(idx_t n, real_t *x, real_t *y)
{
  for (n--; n>=0; n--) {
    if (x[n] > y[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function compares two vectors x & y and returns true 
    if \forall i, x[i] >= y[i].
*/
/**************************************************************************/
int rvecge(idx_t n, real_t *x, real_t *y)
{
  for (n--; n>=0; n--) {
    if (x[n] < y[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function compares vectors x1+x2 against y and returns true 
    if \forall i, x1[i]+x2[i] <= y[i]. 
*/
/**************************************************************************/
int rvecsumle(idx_t n, real_t *x1, real_t *x2, real_t *y)
{
  for (n--; n>=0; n--) {
    if (x1[n]+x2[n] > y[n]) 
      return 0;
  }

  return 1;
}


/*************************************************************************/
/*! This function returns max_i(x[i]-y[i]) */
/**************************************************************************/
real_t rvecmaxdiff(idx_t n, real_t *x, real_t *y)
{
  real_t max;

  max = x[0]-y[0];

  for (n--; n>0; n--) {
    if (max < x[n]-y[n]) 
      max = x[n]-y[n];
  }

  return max;
}


/*************************************************************************/
/*! This function returns true if \forall i, x[i] <= z[i]. */
/**************************************************************************/
int ivecle(idx_t n, idx_t *x, idx_t *z)
{
  for (n--; n>=0; n--) {
    if (x[n] > z[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function returns true if \forall i, x[i] >= z[i]. */
/**************************************************************************/
int ivecge(idx_t n, idx_t *x, idx_t *z)
{
  for (n--; n>=0; n--) {
    if (x[n] < z[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function returns true if \forall i, a*x[i]+y[i] <= z[i]. */
/**************************************************************************/
int ivecaxpylez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z)
{
  for (n--; n>=0; n--) {
    if (a*x[n]+y[n] > z[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function returns true if \forall i, a*x[i]+y[i] >= z[i]. */
/**************************************************************************/
int ivecaxpygez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z)
{
  for (n--; n>=0; n--) {
    if (a*x[n]+y[n] < z[n]) 
      return 0;
  }

  return  1;
}


/*************************************************************************/
/*! This function checks if v+u2 provides a better balance in the weight 
     vector that v+u1 */
/*************************************************************************/
int BetterVBalance(idx_t ncon, real_t *invtvwgt, idx_t *v_vwgt, idx_t *u1_vwgt, 
        idx_t *u2_vwgt)
{
  idx_t i;
  real_t sum1=0.0, sum2=0.0, diff1=0.0, diff2=0.0;

  for (i=0; i<ncon; i++) {
    sum1 += (v_vwgt[i]+u1_vwgt[i])*invtvwgt[i];
    sum2 += (v_vwgt[i]+u2_vwgt[i])*invtvwgt[i];
  }
  sum1 = sum1/ncon;
  sum2 = sum2/ncon;

  for (i=0; i<ncon; i++) {
    diff1 += rabs(sum1 - (v_vwgt[i]+u1_vwgt[i])*invtvwgt[i]);
    diff2 += rabs(sum2 - (v_vwgt[i]+u2_vwgt[i])*invtvwgt[i]);
  }

  return (diff1 - diff2 >= 0);
}


/*************************************************************************/
/*! This function takes two ubfactor-centered load imbalance vectors x & y, 
    and returns true if y is better balanced than x. */
/*************************************************************************/ 
int BetterBalance2Way(idx_t n, real_t *x, real_t *y)
{
  real_t nrm1=0.0, nrm2=0.0;

  for (--n; n>=0; n--) {
    if (x[n] > 0) nrm1 += x[n]*x[n];
    if (y[n] > 0) nrm2 += y[n]*y[n];
  }
  return nrm2 < nrm1;
}


/*************************************************************************/
/*! Given a vertex and two weights, this function returns 1, if the second 
    partition will be more balanced than the first after the weighted 
    additional of that vertex.
    The balance determination takes into account the ideal target weights
    of the two partitions.
*/
/*************************************************************************/
int BetterBalanceKWay(idx_t ncon, idx_t *vwgt, real_t *ubvec, 
        idx_t a1, idx_t *pt1, real_t *bm1, 
        idx_t a2, idx_t *pt2, real_t *bm2)
{
  idx_t i;
  real_t tmp, nrm1=0.0, nrm2=0.0, max1=0.0, max2=0.0;

  for (i=0; i<ncon; i++) {
    tmp = bm1[i]*(pt1[i]+a1*vwgt[i]) - ubvec[i];
    //printf("BB: %d %+.4f ", (int)i, (float)tmp);
    nrm1 += tmp*tmp;
    max1 = (tmp > max1 ? tmp : max1);

    tmp = bm2[i]*(pt2[i]+a2*vwgt[i]) - ubvec[i];
    //printf("%+.4f ", (float)tmp);
    nrm2 += tmp*tmp;
    max2 = (tmp > max2 ? tmp : max2);

    //printf("%4d %4d %4d %4d %4d %4d %4d %.2f\n", 
    //    (int)vwgt[i],
    //    (int)a1, (int)pt1[i], (int)tpt1[i],
    //    (int)a2, (int)pt2[i], (int)tpt2[i], ubvec[i]);
  }
  //printf("   %.3f %.3f %.3f %.3f\n", (float)max1, (float)nrm1, (float)max2, (float)nrm2);

  if (max2 < max1)
    return 1;

  if (max2 == max1 && nrm2 < nrm1)
    return 1;

  return 0;
}


/*************************************************************************/
/*! Computes the maximum load imbalance of a partitioning solution over 
    all the constraints. */
/**************************************************************************/ 
real_t ComputeLoadImbalance(graph_t *graph, idx_t nparts, real_t *pijbm)
{
  idx_t i, j, ncon, *pwgts;
  real_t max, cur;

  ncon  = graph->ncon;
  pwgts = graph->pwgts;

  max = 1.0;
  for (i=0; i<ncon; i++) {
    for (j=0; j<nparts; j++) {
      cur = pwgts[j*ncon+i]*pijbm[j*ncon+i];
      if (cur > max)
        max = cur;
    }
  }

  return max;
}


/*************************************************************************/
/*! Computes the maximum load imbalance difference of a partitioning 
    solution over all the constraints. 
    The difference is defined with respect to the allowed maximum 
    unbalance for the respective constraint. 
 */
/**************************************************************************/ 
real_t ComputeLoadImbalanceDiff(graph_t *graph, idx_t nparts, real_t *pijbm,
           real_t *ubvec)
{
  idx_t i, j, ncon, *pwgts;
  real_t max, cur;

  ncon  = graph->ncon;
  pwgts = graph->pwgts;

  max = -1.0;
  for (i=0; i<ncon; i++) {
    for (j=0; j<nparts; j++) {
      cur = pwgts[j*ncon+i]*pijbm[j*ncon+i] - ubvec[i];
      if (cur > max)
        max = cur;
    }
  }

  return max;
}


/*************************************************************************/
/*! Computes the difference between load imbalance of each constraint across 
    the partitions minus the desired upper bound on the load imabalnce.
    It also returns the maximum load imbalance across the partitions &
    constraints. */
/**************************************************************************/ 
real_t ComputeLoadImbalanceDiffVec(graph_t *graph, idx_t nparts, real_t *pijbm, 
         real_t *ubfactors, real_t *diffvec)
{
  idx_t i, j, ncon, *pwgts;
  real_t cur, max;

  ncon  = graph->ncon;
  pwgts = graph->pwgts;

  for (max=-1.0, i=0; i<ncon; i++) {
    diffvec[i] = pwgts[i]*pijbm[i] - ubfactors[i];
    for (j=1; j<nparts; j++) {
      cur = pwgts[j*ncon+i]*pijbm[j*ncon+i] - ubfactors[i];
      if (cur > diffvec[i])
        diffvec[i] = cur;
    }
    if (max < diffvec[i])
      max = diffvec[i];
  }

  return max;
}


/*************************************************************************/
/*! Computes the load imbalance of each constraint across the partitions. */
/**************************************************************************/ 
void ComputeLoadImbalanceVec(graph_t *graph, idx_t nparts, real_t *pijbm, 
         real_t *lbvec)
{
  idx_t i, j, ncon, *pwgts;
  real_t cur;

  ncon  = graph->ncon;
  pwgts = graph->pwgts;

  for (i=0; i<ncon; i++) {
    lbvec[i] = pwgts[i]*pijbm[i];
    for (j=1; j<nparts; j++) {
      cur = pwgts[j*ncon+i]*pijbm[j*ncon+i];
      if (cur > lbvec[i])
        lbvec[i] = cur;
    }
  }
}


