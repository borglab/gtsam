
#include "commonlib.h"

/*#define DEBUG_SPARSELIB*/

#define INITIALSIZE 10
#define RESIZEDELTA  4

#ifndef SPARSELIB

#define SPARSELIB

typedef struct _sparseVector {
  int    limit;
  int    size;
  int    count;
  int    *index;
  REAL *value;
} sparseVector;

typedef struct _sparseMatrix {
  int    limit;
  int    size;
  int    count;
  int    limitVector;
  sparseVector **list;
} sparseMatrix;

#endif


#ifdef __cplusplus
  extern "C" {
#endif

sparseMatrix *createMatrix(int dimLimit, int lenLimit, int initVectors);
void resizeMatrix(sparseMatrix *matrix, int newSize);
int appendMatrix(sparseMatrix *matrix, sparseVector *newVector);
int NZcountMatrix(sparseMatrix *matrix);
void freeMatrix(sparseMatrix *matrix);
void printMatrix(int n, sparseMatrix *matrix, int modulo, MYBOOL showEmpty);

sparseVector *createVector(int dimLimit, int initSize);
sparseVector *cloneVector(sparseVector *sparse);
int  redimensionVector(sparseVector *sparse, int newDim);
int  resizeVector(sparseVector *sparse, int newSize);
void moveVector(sparseVector *sparse, int destPos, int sourcePos, int itemCount);
void rotateVector(sparseVector *sparse, int startPos, int chainSize, int stepDelta);
void swapVector(sparseVector *sparse1, sparseVector *sparse2);
void freeVector(sparseVector *sparse);
void printVector(int n, sparseVector *sparse, int modulo);
MYBOOL verifyVector(sparseVector *sparse);

int firstIndex(sparseVector *sparse);
int lastIndex(sparseVector *sparse);
int getDiagonalIndex(sparseVector *sparse);
int putDiagonalIndex(sparseVector *sparse, int index);
MYBOOL putDiagonal(sparseVector *sparse, REAL value);
REAL getDiagonal(sparseVector *sparse);
REAL getItem(sparseVector *sparse, int targetIndex);
REAL putItem(sparseVector *sparse, int targetIndex, REAL value);
REAL addtoItem(sparseVector *sparse, int targetIndex, REAL value);
void swapItems(sparseVector *sparse, int firstIndex, int secondIndex);
void clearVector(sparseVector *sparse, int indexStart, int indexEnd);
int getVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd, MYBOOL doClear);
void putVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd);
void fillVector(sparseVector *sparse, int count, REAL value);

REAL dotVector(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd);

void daxpyVector1(sparseVector *sparse, REAL scalar, REAL *dense, int indexStart, int indexEnd);
void daxpyVector2(REAL *dense, REAL scalar, sparseVector *sparse, int indexStart, int indexEnd);
void daxpyVector3(sparseVector *sparse1, REAL scalar, sparseVector *sparse2, int indexStart, int indexEnd);

void dswapVector1(sparseVector *sparse, REAL *dense, int indexStart, int indexEnd);
void dswapVector2(REAL *dense, sparseVector *sparse, int indexStart, int indexEnd);
void dswapVector3(sparseVector *sparse1, sparseVector *sparse2, int indexStart, int indexEnd);

int idamaxVector(sparseVector *sparse, int is, REAL *maxValue);

#ifdef __cplusplus
  }
#endif

