/*!
 * \file 
 *
 * \brief Various routines that perform random-walk based operations
          on graphs stored as gk_csr_t matrices.
 *
 * \author George Karypis
 * \version\verbatim $Id: rw.c 11078 2011-11-12 00:20:44Z karypis $ \endverbatim
 */

#include <GKlib.h>


/*************************************************************************/
/*! Computes the (personalized) page-rank of the vertices in a graph.

  \param mat is the matrix storing the graph.
  \param lamda is the restart probability.
  \param eps is the error tolerance for convergance.
  \param max_niter is the maximum number of allowed iterations.
  \param pr on entry stores the restart distribution of the vertices. 
         This allows for the computation of personalized page-rank scores 
         by appropriately setting that parameter. 
         On return, pr stores the computed page ranks.
 
  \returns the number of iterations that were performed.
*/
/**************************************************************************/
int gk_rw_PageRank(gk_csr_t *mat, float lamda, float eps, int max_niter, float *pr)
{
  ssize_t i, j, k, iter, nrows;
  double *rscale, *prold, *prnew, *prtmp;
  double fromsinks, error;
  ssize_t *rowptr;
  int *rowind;
  float *rowval;

  nrows  = mat->nrows;
  rowptr = mat->rowptr;
  rowind = mat->rowind;
  rowval = mat->rowval;

  prold  = gk_dsmalloc(nrows, 0, "gk_rw_PageRank: prnew");
  prnew  = gk_dsmalloc(nrows, 0, "gk_rw_PageRank: prold");
  rscale = gk_dsmalloc(nrows, 0, "gk_rw_PageRank: rscale");

  /* compute the scaling factors to get adjacency weights into transition 
     probabilities */
  for (i=0; i<nrows; i++) {
    for (j=rowptr[i]; j<rowptr[i+1]; j++)
      rscale[i] += rowval[j];
    if (rscale[i] > 0)
      rscale[i] = 1.0/rscale[i];
  }

  /* the restart distribution is the initial pr scores */
  for (i=0; i<nrows; i++)
    prnew[i] = pr[i];

  /* get into the PR iteration */
  for (iter=0; iter<max_niter; iter++) {
    gk_SWAP(prnew, prold, prtmp);
    gk_dset(nrows, 0.0, prnew);

    /* determine the total current PR score of the sinks so that you 
       can distribute them to all nodes according to the restart 
       distribution. */
    for (fromsinks=0.0, i=0; i<nrows; i++) {
      if (rscale[i] == 0) 
        fromsinks += prold[i];
    }

    /* push random-walk scores to the outlinks */
    for (i=0; i<nrows; i++) {
      for (j=rowptr[i]; j<rowptr[i+1]; j++)
        prnew[rowind[j]] += prold[i]*rscale[i]*rowval[j];
    }

    /* apply the restart conditions */
    for (i=0; i<nrows; i++) {
      prnew[i] = lamda*(fromsinks*pr[i]+prnew[i]) + (1.0-lamda)*pr[i];
    }

    /* compute the error */
    for (error=0.0, i=0; i<nrows; i++) 
      error = (fabs(prnew[i]-prold[i]) > error ? fabs(prnew[i]-prold[i]) : error);

    //printf("nrm1: %le  maxfabserr: %le\n", gk_dsum(nrows, prnew, 1), error);

    if (error < eps)
      break;
  }

  /* store the computed pr scores into pr for output */
  for (i=0; i<nrows; i++)
    pr[i] = prnew[i];

  gk_free((void **)&prnew, &prold, &rscale, LTERM);
  
  return (int)(iter+1);

}

