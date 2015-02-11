/*!
  \file  evaluate.c
  \brief Various routines to evaluate classification performance

  \author George
  \date 9/23/2008
  \version\verbatim $Id: evaluate.c 13328 2012-12-31 14:57:40Z karypis $ \endverbatim
*/

#include <GKlib.h>

/**********************************************************************
 * This function computes the max accuracy score of a ranked list,
 * given +1/-1 class list
 **********************************************************************/
float ComputeAccuracy(int n, gk_fkv_t *list)
{
  int i, P, N, TP, FN = 0;
  float bAccuracy = 0.0;
  float acc;
  
  for (P=0, i=0;i<n;i++)
    P += (list[i].val == 1? 1 : 0);
  N = n - P;
  
  TP = FN = 0;
  
  for(i=0; i<n; i++){
    if (list[i].val == 1)
      TP++; 
    else
      FN++;
    
    acc = (TP + N - FN) * 100.0/ (P + N) ;
    if (acc > bAccuracy)
      bAccuracy = acc;
  }
  
  return bAccuracy;
}


/*****************************************************************************
 * This function computes the ROC score of a ranked list, given a +1/-1 class
 * list.
 ******************************************************************************/
float ComputeROCn(int n, int maxN, gk_fkv_t *list)
{
  int i, P, TP, FP, TPprev, FPprev, AUC;
  float prev;
  
  FP = TP = FPprev = TPprev = AUC = 0;
  prev = list[0].key -1;
  
  for (P=0, i=0; i<n; i++)
    P += (list[i].val == 1 ? 1 : 0);
  
  for (i=0; i<n && FP < maxN; i++) {
    if (list[i].key != prev) {
      AUC += (TP+TPprev)*(FP-FPprev)/2;
      prev = list[i].key;
      FPprev = FP;
      TPprev = TP;
    }
    if (list[i].val == 1) 
      TP++;
    else {
      FP++;
    }
  }
  AUC += (TP+TPprev)*(FP-FPprev)/2;

  return (TP*FP > 0 ? (float)(1.0*AUC/(P*FP)) : 0.0);
}


/*****************************************************************************
* This function computes the median rate of false positive for each positive
* instance.
******************************************************************************/
float ComputeMedianRFP(int n, gk_fkv_t *list)
{
  int i, P, N, TP, FP;

  P = N = 0;
  for (i=0; i<n; i++) {
    if (list[i].val == 1)
      P++;
    else
      N++;
  }
  
  FP = TP = 0;
  for (i=0; i<n && TP < (P+1)/2; i++) {
    if (list[i].val == 1) 
      TP++;
    else 
      FP++;
  }
  
  return 1.0*FP/N;
}

/*********************************************************
 * Compute the mean
 ********************************************************/
float ComputeMean (int n, float *values)
{
  int i;
  float mean = 0.0;

  for(i=0; i < n; i++)
    mean += values[i];
  
  return 1.0 * mean/ n;
}

/********************************************************
 * Compute the standard deviation
 ********************************************************/
float ComputeStdDev(int  n, float *values)
{
  int i;
  float mean = ComputeMean(n, values);
  float stdDev = 0;
  
  for(i=0;i<n;i++){
    stdDev += (values[i] - mean)* (values[i] - mean);
  }
  
  return sqrt(1.0 * stdDev/n);
}
