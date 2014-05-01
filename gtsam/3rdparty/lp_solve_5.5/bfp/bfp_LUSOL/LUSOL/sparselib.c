
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "commonlib.h"
#include "myblas.h"
#include "sparselib.h"


sparseMatrix *createMatrix(int dimLimit, int lenLimit, int initVectors)
{
  int          initsize;
  sparseMatrix *matrix;

  if(initVectors < 0)
    initVectors = 0;
  if(initVectors == 0)
    initsize = MIN(INITIALSIZE, dimLimit);
  else
    initsize = MAX(INITIALSIZE, initVectors);

  CALLOC(matrix, 1);
  matrix->limit = dimLimit;
  matrix->limitVector = lenLimit;
  resizeMatrix(matrix, initsize);
  while(initVectors > 0) {
    initVectors--;
    appendMatrix(matrix, createVector(lenLimit, 2));
  }
  return(matrix);
}


void resizeMatrix(sparseMatrix *matrix, int newSize)
{
  int oldSize;

  if(matrix == NULL)
    oldSize = 0;
  else
    oldSize = matrix->size;
  while(oldSize>newSize) {
	  oldSize--;
	  freeVector(matrix->list[oldSize]);
    return;
  }
  REALLOC(matrix->list, newSize);
  while(oldSize<newSize) {
	  matrix->list[oldSize] = NULL;
	  oldSize++;
  }
  if(newSize>0)
    matrix->size = newSize;
}

int appendMatrix(sparseMatrix *matrix, sparseVector *newVector)
{
   if(matrix->count == matrix->size)
     resizeMatrix(matrix, matrix->size + 10);
   matrix->list[matrix->count] = newVector;
   matrix->count++;
   putDiagonalIndex(newVector, matrix->count);
   return(matrix->count);
}


int NZcountMatrix(sparseMatrix *matrix)
{
  int i, nz;

  nz = 0;
  for(i = 0; i < matrix->count; i++)
    nz += matrix->list[i]->count;

  return( nz );
}


void freeMatrix(sparseMatrix *matrix)
{
  resizeMatrix(matrix, 0);
  FREE(matrix);
}


void printMatrix(int n, sparseMatrix *matrix, int modulo, MYBOOL showEmpty)
{
   int i;
   for(i = 1; i<=matrix->count; i++) 
     if(matrix->list[i-1] != NULL) { 
       if(showEmpty || matrix->list[i-1]->count>0)
         printVector(n, matrix->list[i-1], modulo);
   }
}


sparseVector *createVector(int dimLimit, int initSize)
{
  sparseVector *newitem;
  CALLOC(newitem, 1);
  newitem->limit = dimLimit;
  initSize = resizeVector(newitem, initSize);
  return(newitem);
}


sparseVector *cloneVector(sparseVector *sparse)
{
  sparseVector *hold;
  hold = createVector(sparse->limit, sparse->count);
  hold->count = sparse->count;
  MEMCOPY(&hold->value[0], &sparse->value[0], (sparse->count+1));
  MEMCOPY(&hold->index[0], &sparse->index[0], (sparse->count+1));
  return(hold);
}

int redimensionVector(sparseVector *sparse, int newDim)
{
  int olddim, i;

  olddim = sparse->limit;
  sparse->limit = newDim;
  if(lastIndex(sparse)>newDim) {
    i = sparse->count;
    while(i>0 && sparse->index[i]>newDim) i--;
    sparse->count = i;
    resizeVector(sparse, sparse->count);
  }
  return(olddim);
}


int resizeVector(sparseVector *sparse, int newSize)
{
  int oldsize;

  oldsize = sparse->size;
  REALLOC(sparse->value, (newSize+1));
  REALLOC(sparse->index, (newSize+1));
  sparse->size = newSize;
  return(oldsize);
}


void moveVector(sparseVector *sparse, int destPos, int sourcePos, int itemCount)
{
  int i;
  
  if(itemCount <= 0 || sourcePos == destPos)
    return;

#if defined DOFASTMATH
  if(TRUE) {
    MEMMOVE(&sparse->value[destPos], &sparse->value[sourcePos], itemCount);
    MEMMOVE(&sparse->index[destPos], &sparse->index[sourcePos], itemCount);
  }
  else {
    int    *idxPtr1, *idxPtr2;
    double *valPtr1, *valPtr2;

    for(i = 1, idxPtr1 = sparse->index+destPos, idxPtr2 = sparse->index+sourcePos,
               valPtr1 = sparse->value+destPos, valPtr2 = sparse->value+sourcePos; 
        i<=itemCount; i++, idxPtr1++, idxPtr2++, valPtr1++, valPtr2++) {
      *idxPtr1 = *idxPtr2;
      *valPtr1 = *valPtr2;
    }
  }
#else
  for(i = 1; i<=itemCount; i++) {
    sparse->value[destPos] = sparse->value[sourcePos];
    sparse->index[destPos] = sparse->index[sourcePos];
    destPos++;
    sourcePos++;
  }
#endif
}


void rotateVector(sparseVector *sparse, int startPos, int chainSize, int stepDelta)
{
/*  int idxHold; */
/*  double valHold; */

}


void swapVector(sparseVector *sparse1, sparseVector *sparse2)
{
  int n, m, *idx;
  REAL *val;

  n = sparse1->count;
  sparse1->count = sparse2->count;
  sparse2->count = n;

  n = sparse1->size;
  sparse1->size = sparse2->size;
  sparse2->size = n;

  n = sparse1->limit;
  sparse1->limit = sparse2->limit;
  sparse2->limit = n;

  idx = sparse1->index;
  sparse1->index = sparse2->index;
  sparse2->index = idx;

  val = sparse1->value;
  sparse1->value = sparse2->value;
  sparse2->value = val;

  n = getDiagonalIndex(sparse1);
  m = getDiagonalIndex(sparse2);
  putDiagonalIndex(sparse1, m);
  putDiagonalIndex(sparse2, n);

}


void freeVector(sparseVector *sparse)
{
	if(sparse != NULL) {
    FREE(sparse->value);
    FREE(sparse->index);
    FREE(sparse);
  }
}


MYBOOL verifyVector(sparseVector *sparse)
{
  int i, n, k1, k2, kd; 
  int  err = 0;
  double vd;

  n = sparse->count;
  kd = sparse->index[0];
  vd = sparse->value[0];
  if(n <= 1)
    return(TRUE);
  k1 = 0;
  k2 = sparse->index[1];
  if(k2 == kd && sparse->value[1] != vd) 
    err = 2;

  for(i = 2; i <= n && err == 0; i++) {
    k1 = k2;
    k2 = sparse->index[i];
    if(k1 >= k2) err = 1;
    if(k2 == kd && sparse->value[i] != vd) err = 2;
  }
  if(err == 0)
    return(TRUE);
  else if(err == 1)
    printf("Invalid sparse vector index order");
  else if(err == 2)
    printf("Invalid sparse vector diagonal value");
  return(FALSE);
}


int firstIndex(sparseVector *sparse)
{
  return(sparse->index[1]);
}


int lastIndex(sparseVector *sparse)
{
  return(sparse->index[sparse->count]);
}


int getDiagonalIndex(sparseVector *sparse)
{
  return(sparse->index[0]);
}


int putDiagonalIndex(sparseVector *sparse, int index)
{
  int oldindex;
  oldindex = sparse->index[0];
  if(index > 0) {
    sparse->index[0] = 0; /* Must temporarily set to zero to force vector search in getItem */
    sparse->value[0] = getItem(sparse, index);
  }
  else
    sparse->value[0] = 0;
  sparse->index[0] = index;
  return(oldindex);
}


MYBOOL putDiagonal(sparseVector *sparse, REAL value)
{
  if(sparse->index[0]>0) {
    putItem(sparse, sparse->index[0], value); 
    return(TRUE);
  }
  else
    return(FALSE);
}


REAL getDiagonal(sparseVector *sparse)
{
   return(sparse->value[0]);
}


REAL getItem(sparseVector *sparse, int targetIndex)
{
  /* First check if we want the diagonal element */ 
  if(targetIndex == sparse->index[0])
    return(sparse->value[0]);

  /* If not, search for the variable's position in the index list */
  targetIndex = findIndex(targetIndex, sparse->index, sparse->count, BLAS_BASE);
  if(targetIndex < 0)
    return(0);
  else
    return(sparse->value[targetIndex]);
}


REAL addtoItem(sparseVector *sparse, int targetIndex, REAL value)
{
  int idx;

  if(targetIndex > 0) 
    idx = findIndex(targetIndex, sparse->index, sparse->count, BLAS_BASE);
  else {
    idx = -targetIndex;
    if(idx > sparse->count)
      /* Index error; ignore item */
      return(0.0);
  }

  if(idx <=0 )
    value = putItem(sparse, targetIndex, value);
  else {
    value += sparse->value[idx];
    putItem(sparse, -idx, value);
  }
  return(value);
}


REAL putItem(sparseVector *sparse, int targetIndex, REAL value)
{
  REAL last = 0.0;
  int  posIndex;

  if(targetIndex < 0) {
    posIndex = -targetIndex;
    if(posIndex > sparse->count)
      return(last);
    targetIndex = sparse->index[posIndex];
  }
  else
    posIndex = findIndex(targetIndex, sparse->index, sparse->count, BLAS_BASE);

  if(fabs(value) < MACHINEPREC)
    value = 0;

  if(targetIndex == sparse->index[0]) 
    sparse->value[0] = value;

  if(posIndex < 0) {
    if(value != 0) {
      if(sparse->count == sparse->size)
        resizeVector(sparse, sparse->size + RESIZEDELTA);
      posIndex = -posIndex;
      sparse->count++;
      if(posIndex < sparse->count) 
        moveVector(sparse, posIndex+1, posIndex, sparse->count-posIndex);
      sparse->value[posIndex] = value;
      sparse->index[posIndex] = targetIndex;
    }
  }
  else {
    if(value == 0) {
      last = sparse->value[posIndex];
      if(sparse->count > posIndex) 
        moveVector(sparse, posIndex, posIndex+1, sparse->count-posIndex);
      sparse->count--;
    }
    else {
      sparse->value[posIndex] = value;
      sparse->index[posIndex] = targetIndex;
    }
  }

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse);
#endif

  return(last);
}


void swapItems(sparseVector *sparse, int firstIndex, int secondIndex)
{
  int i,j,ki,kj;
  REAL hold;

  if(firstIndex == secondIndex)
    return;
  if(firstIndex > secondIndex) {
    i = firstIndex;
    firstIndex = secondIndex;
    secondIndex = i;
  }

  if(FALSE) {
    i = 1;
    ki = 0;
    while(i <= sparse->count && (ki = sparse->index[i])<firstIndex) i++;
    j = i;
    kj = 0;
    while(j <= sparse->count && (kj = sparse->index[j])<secondIndex) j++;
  }
  else {
    i = findIndex(firstIndex, sparse->index, sparse->count, BLAS_BASE);
    if(i < 0)
      i = -i;
    j = findIndex(secondIndex, sparse->index, sparse->count, BLAS_BASE);
    if(j < 0)
      j = -j;
  }

  if(i > sparse->count)
    ki = 0;
  else
    ki = sparse->index[i];
  if(j > sparse->count)
    kj = 0;
  else
    kj = sparse->index[j];

  if(ki == firstIndex && kj == secondIndex) {   /* Found both -> swap in place */
    hold = sparse->value[i];
    sparse->value[i] = sparse->value[j];
    sparse->value[j] = hold;

    if(sparse->index[0] == firstIndex)
      sparse->value[0] = sparse->value[i];
    else if(sparse->index[0] == secondIndex)
      sparse->value[0] = sparse->value[j];
  }
  else if(ki == firstIndex) {                   /* Found first, but not the second -> shift left */
    j--;
    if(i < j) {
      hold = sparse->value[i];
      moveVector(sparse, i, i+1, j-i);
      sparse->value[j] = hold;
    }
    sparse->index[j] = secondIndex;

    if(sparse->index[0] == firstIndex)
      sparse->value[0] = 0;
    else if(sparse->index[0] == secondIndex)
      sparse->value[0] = sparse->value[j];

  }
  else if(kj == secondIndex) {                  /* Found second, but not the first -> shift right */
    if(i < j) {
      hold = sparse->value[j];
      moveVector(sparse, i+1, i, j-i);
      sparse->value[i] = hold;
    }
    sparse->index[i] = firstIndex;

    if(sparse->index[0] == firstIndex)
      sparse->value[0] = sparse->value[i];
    else if(sparse->index[0] == secondIndex)
      sparse->value[0] = 0;
  }

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse);
#endif

}


void clearVector(sparseVector *sparse, int indexStart, int indexEnd)
{
  int i;

  i = sparse->count;
  if(i==0) return;

  if(indexStart<=0)
    indexStart=sparse->index[1];
  if(indexEnd<=0)
    indexEnd=sparse->index[i];

  if(indexStart>indexEnd) return;

  if(sparse->index[0]>=indexStart && sparse->index[0]<=indexEnd) {
    sparse->value[0] = 0;
  }
  if(indexStart<=sparse->index[1] && indexEnd>=sparse->index[i]) 
    sparse->count = 0;
  else {
    while(i>0 && sparse->index[i]>indexEnd) i--;
    indexEnd = i;
    while(i>0 && sparse->index[i]>=indexStart) i--;
    indexStart = i+1;
    if(indexEnd>=indexStart) {
      i = sparse->count-indexEnd;
      moveVector(sparse, indexStart, indexEnd+1, i);
      sparse->count -= indexEnd-indexStart+1;
    }
  }

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse);
#endif

}


int getVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd, MYBOOL doClear)
{
  int i,k;

  i = 1;
  while(i<=sparse->count && sparse->index[i]<indexStart) i++;

  while(i<=sparse->count && (k=sparse->index[i])<=indexEnd) {
    while(indexStart<k) {
      dense[indexStart] = 0;
      indexStart++;
    }
    dense[indexStart] = sparse->value[i];
    indexStart++;
    i++;
  }

  while(indexStart<=indexEnd) {
    dense[indexStart] = 0;
    indexStart++;
  }

  k = sparse->count;
  if(doClear) {
    sparse->count = 0;
    sparse->value[0] = 0;
  }
  return(k);
}

void putVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd)
{
  int i,n;

  n = sparse->count;
  if(indexStart<=0)
    indexStart=sparse->index[1];
  if(indexEnd<=0)
    indexEnd=sparse->index[n];

  if(n==0 || sparse->index[n]<indexStart) {
    i = sparse->index[0];
    if(i>=indexStart && i<=indexEnd)
      sparse->value[0] = 0;
    for(i = indexStart; i<=indexEnd; i++) {
      if(dense[i] == 0) continue;
      if(sparse->size == sparse->count)
        resizeVector(sparse, sparse->size + RESIZEDELTA);
      sparse->count++;
      sparse->value[sparse->count] = dense[i];
      sparse->index[sparse->count] = i;
      if(i == sparse->index[0]) 
        sparse->value[0] = dense[i];
    }
  }
  else {
    while(indexStart <= indexEnd) {
      putItem(sparse, indexStart, dense[indexStart]);
      indexStart++;
    }
  }

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse);
#endif

}


void fillVector(sparseVector *sparse, int count, REAL value)
{
  int i;

  if(sparse->count > 0) 
    clearVector(sparse, 0, 0);
  for(i = 1; i<=count; i++)
    putItem(sparse, i, value);
}


REAL dotVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd)
{
  int  i, n;
  long REAL sum;
  
  n = sparse->count;
  sum = 0;

  if(n > 0) {
    if(indexStart<=0)
      indexStart=sparse->index[1];
    if(indexEnd<=0)
      indexEnd=sparse->index[n];

    if(indexStart > 1) {
      i = findIndex(indexStart, sparse->index, sparse->count, BLAS_BASE);
      if(i < 0) {
        i = -i;
        if(i > n) 
          return(sum);
      }
    }
    else
      i = 1;

    /* CPU intensive loop; provide alternative evaluation models */
#if defined DOFASTMATH
    {
      /* Do fast pointer arithmetic */
      int  *indexptr;
      REAL *valueptr;
/*      for(i = 1, indexptr = sparse->index + 1;
          i <= n && (*indexptr) < indexStart; i++, indexptr++); */
      indexptr = sparse->index + i;
      for(valueptr = sparse->value + i;
          i <= n && (*indexptr) <= indexEnd;  i++, indexptr++, valueptr++) 
        sum += (*valueptr) * dense[(*indexptr)];
    }
#else
    {
      /* Do traditional indexed access */
      int k;
/*      i = 1; */
/*      while(i<=n && sparse->index[i]<indexStart) i++; */
      while(i<=n && (k = sparse->index[i])<=indexEnd) {
        sum += sparse->value[i] * dense[k];
        i++;
      }
    }
#endif    
  }

  return(sum);
}


void daxpyVector1(sparseVector *sparse, REAL scalar, REAL *dense, int indexStart, int indexEnd)
{
  int i, n;

  if(scalar == 0) return;

  n = sparse->count;
  if(indexStart<=0)
    indexStart=sparse->index[1];
  if(indexEnd<=0)
    indexEnd=sparse->index[n];

  /* CPU intensive loop; provide alternative evaluation models */
#if defined DOFASTMATH
  {
    /* Do fast pointer arithmetic */
    int    *indexptr;
    REAL *valueptr;
    for(i = 1, indexptr = sparse->index + 1;
        i <= n && (*indexptr) < indexStart; i++, indexptr++);
    for(valueptr = sparse->value + i;
        i <= n && (*indexptr) <= indexEnd;  i++, indexptr++, valueptr++) 
      dense[(*indexptr)] += (*valueptr) * scalar;
  }
#else
  {
    /* Do traditional indexed access */
    int k;
    for(i = 1; i<= n; i++) {
      k = sparse->index[i];
      if(k<indexStart) continue;
      if(k>indexEnd) break;
      dense[k] += sparse->value[i] * scalar;
    }
  }
#endif  
}
void daxpyVector2(REAL *dense, REAL scalar, sparseVector *sparse, int indexStart, int indexEnd)
{
  sparseVector *hold;

  hold = createVector(sparse->limit, sparse->count);
  putDiagonalIndex(hold, getDiagonalIndex(sparse));
  putVector(hold, dense, indexStart, indexEnd);
  daxpyVector3(hold, scalar, sparse, indexStart, indexEnd);
  freeVector(hold);
}
void daxpyVector3(sparseVector *sparse1, REAL scalar, sparseVector *sparse2, int indexStart, int indexEnd)
{
  int i1, i2, k, p1, p2, c1, c2;
  sparseVector *hold;

  if(sparse1->count == 0) return;

 /* Spool to start positions */
  i1 = 1;
  c1 = sparse1->count;
  while(i1 <= c1 && sparse1->index[i1] < indexStart) i1++;
  if(i1 <= c1)
    p1 = sparse1->index[i1];
  else
    p1 = indexEnd+1;

  i2 = 1;
  c2 = sparse2->count;
  while(i2 <= c2 && sparse2->index[i2] < indexStart) i2++;
  if(i2 <= c2)
    p2 = sparse2->index[i2];
  else
    p2 = indexEnd+1;

 /* Create a temporary vector */
  k = c1+c2;
  if(k > 0) {
    hold = createVector(MAX(sparse1->limit, sparse2->limit), k);
    putDiagonalIndex(hold, getDiagonalIndex(sparse2));
  }
  else
    hold = sparse2;

 /* Loop over all items in both vectors */
  while((i1 <= c1 && p1 <= indexEnd) || 
        (i2 <= c2 && p2 <= indexEnd)) {

    k = 0;

   /* Add/spool exclusive right-vector items */
    while(i2 <= c2 && p2 < p1) {
      if(hold != sparse2)
        putItem(hold, p2, sparse2->value[i2]);
      i2++;
      if(i2 <= c2)
        p2 = sparse2->index[i2];
      else
        p2 = indexEnd+1;
      k++;
    }
   /* Add equal-indexed items */
    while(i1 <= c1 && i2 <= c2 && p1 == p2) {
/*      if(hold != sparse2) */
        putItem(hold, p1, scalar*sparse1->value[i1]+sparse2->value[i2]);
/*      else
          addtoItem(sparse2, -i2, scalar*sparse1->value[i1]); */
      i1++;
      if(i1 <= c1)
        p1 = sparse1->index[i1];
      else
        p1 = indexEnd+1;
      i2++;
      if(i2 <= c2)
        p2 = sparse2->index[i2];
      else
        p2 = indexEnd+1;
      k++;
    }
   /* Add exclusive left-vector items */
    while(i1 <= c1 && p1 < p2) {
      putItem(hold, p1, scalar*sparse1->value[i1]);
/*      if(hold == sparse2) c2++; */
      i1++;
      if(i1 <= c1)
        p1 = sparse1->index[i1];
      else
        p1 = indexEnd+1;
      k++;
    }

    if(k == 0) break;
  }

/*  if(hold != sparse2) */
  {
    swapVector(hold, sparse2);
    freeVector(hold);
  }

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse2);
#endif

}


void dswapVector1(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd)
{
  int i, d, n;
  REAL *x;

  if(indexStart <= 0)
    indexStart = 1;
  n = lastIndex(sparse);
  if(indexEnd <= 0) 
    indexEnd = n;
  CALLOC(x, (MAX(indexEnd,n)+1));

  getVector(sparse, x, indexStart, n, FALSE);
  d = getDiagonalIndex(sparse);
  clearVector(sparse, indexStart, n);
  for(i = indexStart; i<=indexEnd; i++) {
    if(dense[i] != 0)
      putItem(sparse, i, dense[i]);
  }
  for(i = indexEnd+1; i<=n; i++) {
    if(x[i] != 0)
      putItem(sparse, i, x[i]);
  }
  MEMCOPY(&dense[indexStart], &x[indexStart], (indexEnd-indexStart+1));

#ifdef DEBUG_SPARSELIB
  verifyVector(sparse);
#endif
  
  FREE(x);
}
void dswapVector2(REAL *dense, sparseVector *sparse, int indexStart, int indexEnd)
{
  dswapVector1(sparse, dense, indexStart, indexEnd);
}


void dswapVector3(sparseVector *sparse1, sparseVector *sparse2, int indexStart, int indexEnd)
{

  REAL *dense1, *dense2;

  if(indexStart<=0)
    indexStart = 1;
  if(indexEnd<=0)
    indexEnd = MAX(lastIndex(sparse1), lastIndex(sparse2));

  if(indexStart <= firstIndex(sparse1) && indexStart <= firstIndex(sparse2) && 
     indexEnd >= lastIndex(sparse1) && indexEnd >= lastIndex(sparse2)) {
    swapVector(sparse1, sparse2);
  }
  else {

    CALLOC(dense1, (indexEnd+1));
    CALLOC(dense2, (indexEnd+1));
    getVector(sparse1, dense1, indexStart, indexEnd, TRUE);
    getVector(sparse2, dense2, indexStart, indexEnd, TRUE);
    clearVector(sparse1, indexStart, indexEnd);
    clearVector(sparse2, indexStart, indexEnd);
    putVector(sparse1, dense2, indexStart, indexEnd);
    putVector(sparse2, dense1, indexStart, indexEnd);
    FREE(dense1);
    FREE(dense2);
  }
}


int idamaxVector(sparseVector *sparse, int is, REAL *maxValue)
{
  int    i, n, imax;
  REAL xmax;

  n = sparse->count;
  imax = 1;
  if(n == 0)
    xmax = 0;
  else {
    xmax = fabs(sparse->value[imax]);

    /* CPU intensive loop; provide alternative evaluation models */
#if defined DOFASTMATH
    {
      /* Do fast pointer arithmetic */
      int  *indexptr;
      REAL *valueptr;
      for(i = 1, indexptr = sparse->index + 1;
          i <= n && (*indexptr) <= is; i++, indexptr++);
      for(valueptr = sparse->value + i;
          i <= n; i++, indexptr++, valueptr++) {
	      if((*valueptr)>xmax) {
		      xmax = (*valueptr);
		      imax = (*indexptr);
        }
      }
    }
#else
    {
      REAL xtest;
      /* Do traditional indexed access */
      i = 1;
      while(i <= n && sparse->index[i] <= is) i++;
      for(; i<=n; i++) {
        xtest = fabs(sparse->value[i]);
	      if(xtest>xmax) {
		      xmax = xtest;
		      imax = sparse->index[i];
        }
      }
    }
#endif    
  }
  if(maxValue != NULL)
    (*maxValue) = sparse->index[imax];
  return(imax);
}


void printVector(int n, sparseVector *sparse, int modulo )
{
  int i,j,k;

  if(sparse == NULL) return;

  if (modulo <= 0) modulo = 5;
  for (i = 1, j = 1; j<=n; i++, j++) {
    if(i<=sparse->count)
      k = sparse->index[i];
    else
      k = n+1;
    while (j < k) {
      if(mod(j, modulo) == 1) 
        printf("\n%2d:%12g", j, 0.0);
      else
        printf(" %2d:%12g", j, 0.0);
      j++;
    }
    if(k<=n) {
      if(mod(j, modulo) == 1) 
        printf("\n%2d:%12g", k, sparse->value[i]);
      else
        printf(" %2d:%12g", k, sparse->value[i]);
    }
  }
  if(mod(j, modulo) != 0) printf("\n");
}


