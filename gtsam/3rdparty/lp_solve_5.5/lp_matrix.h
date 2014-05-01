#ifndef HEADER_lp_matrix
#define HEADER_lp_matrix

#include "lp_types.h"
#include "lp_utils.h"


/* Sparse matrix element (ordered columnwise) */
typedef struct _MATitem
{
  int  rownr;
  int  colnr;
  REAL value;
} MATitem;

/* Constants for matrix product rounding options */
#define MAT_ROUNDNONE             0
#define MAT_ROUNDABS              1
#define MAT_ROUNDREL              2
#define MAT_ROUNDABSREL          (MAT_ROUNDABS + MAT_ROUNDREL)
#define MAT_ROUNDRC               4
#define MAT_ROUNDRCMIN            1.0 /* lp->epspivot */
#if 1
 #define MAT_ROUNDDEFAULT         MAT_ROUNDREL  /* Typically increases performance */
#else
 #define MAT_ROUNDDEFAULT         MAT_ROUNDABS  /* Probably gives more precision */
#endif

/* Compiler option development features */
/*#define DebugInv*/               /* Report array values at factorization/inversion */
#define NoLoopUnroll              /* Do not do loop unrolling */
#define DirectArrayOF             /* Reference lp->obj[] array instead of function call */


/* Matrix column access macros to be able to easily change storage model */
#define CAM_Record                0
#define CAM_Vector                1
#if 0
 #define MatrixColAccess           CAM_Record
#else
 #define MatrixColAccess           CAM_Vector
#endif

#if MatrixColAccess==CAM_Record
#define SET_MAT_ijA(item,i,j,A)   mat->col_mat[item].rownr = i; \
                                  mat->col_mat[item].colnr = j; \
                                  mat->col_mat[item].value = A
#define COL_MAT_COLNR(item)       (mat->col_mat[item].colnr)
#define COL_MAT_ROWNR(item)       (mat->col_mat[item].rownr)
#define COL_MAT_VALUE(item)       (mat->col_mat[item].value)
#define COL_MAT_COPY(left,right)  mat->col_mat[left] = mat->col_mat[right]
#define COL_MAT_MOVE(to,from,rec) MEMMOVE(&(mat->col_mat[to]),&(mat->col_mat[from]),rec)
#define COL_MAT2_COLNR(item)      (mat2->col_mat[item].colnr)
#define COL_MAT2_ROWNR(item)      (mat2->col_mat[item].rownr)
#define COL_MAT2_VALUE(item)      (mat2->col_mat[item].value)
#define matRowColStep             (sizeof(MATitem)/sizeof(int))
#define matValueStep              (sizeof(MATitem)/sizeof(REAL))

#else /* if MatrixColAccess==CAM_Vector */
#define SET_MAT_ijA(item,i,j,A)   mat->col_mat_rownr[item] = i; \
                                  mat->col_mat_colnr[item] = j; \
                                  mat->col_mat_value[item] = A
#define COL_MAT_COLNR(item)       (mat->col_mat_colnr[item])
#define COL_MAT_ROWNR(item)       (mat->col_mat_rownr[item])
#define COL_MAT_VALUE(item)       (mat->col_mat_value[item])
#define COL_MAT_COPY(left,right)  COL_MAT_COLNR(left) = COL_MAT_COLNR(right); \
                                  COL_MAT_ROWNR(left) = COL_MAT_ROWNR(right); \
                                  COL_MAT_VALUE(left) = COL_MAT_VALUE(right)
#define COL_MAT_MOVE(to,from,rec) MEMMOVE(&COL_MAT_COLNR(to),&COL_MAT_COLNR(from),rec); \
                                  MEMMOVE(&COL_MAT_ROWNR(to),&COL_MAT_ROWNR(from),rec); \
                                  MEMMOVE(&COL_MAT_VALUE(to),&COL_MAT_VALUE(from),rec)
#define COL_MAT2_COLNR(item)      (mat2->col_mat_colnr[item])
#define COL_MAT2_ROWNR(item)      (mat2->col_mat_rownr[item])
#define COL_MAT2_VALUE(item)      (mat2->col_mat_value[item])
#define matRowColStep             1
#define matValueStep              1

#endif


/* Matrix row access macros to be able to easily change storage model */
#define RAM_Index                 0
#define RAM_FullCopy              1
#define MatrixRowAccess           RAM_Index

#if MatrixRowAccess==RAM_Index
#define ROW_MAT_COLNR(item)       COL_MAT_COLNR(mat->row_mat[item])
#define ROW_MAT_ROWNR(item)       COL_MAT_ROWNR(mat->row_mat[item])
#define ROW_MAT_VALUE(item)       COL_MAT_VALUE(mat->row_mat[item])

#elif MatrixColAccess==CAM_Record
#define ROW_MAT_COLNR(item)       (mat->row_mat[item].colnr)
#define ROW_MAT_ROWNR(item)       (mat->row_mat[item].rownr)
#define ROW_MAT_VALUE(item)       (mat->row_mat[item].value)

#else /* if MatrixColAccess==CAM_Vector */
#define ROW_MAT_COLNR(item)       (mat->row_mat_colnr[item])
#define ROW_MAT_ROWNR(item)       (mat->row_mat_rownr[item])
#define ROW_MAT_VALUE(item)       (mat->row_mat_value[item])

#endif


typedef struct _MATrec
{
  /* Owner reference */
  lprec     *lp;

  /* Active dimensions */
  int       rows;
  int       columns;

  /* Allocated memory */
  int       rows_alloc;
  int       columns_alloc;
  int       mat_alloc;          /* The allocated size for matrix sized structures */

  /* Sparse problem matrix storage */
#if MatrixColAccess==CAM_Record  
  MATitem   *col_mat;           /* mat_alloc : The sparse data storage */
#else /*MatrixColAccess==CAM_Vector*/
  int       *col_mat_colnr;
  int       *col_mat_rownr;
  REAL      *col_mat_value;
#endif  
  int       *col_end;           /* columns_alloc+1 : col_end[i] is the index of the
                                   first element after column i; column[i] is stored
                                   in elements col_end[i-1] to col_end[i]-1 */
  int       *col_tag;           /* user-definable tag associated with each column */

#if MatrixRowAccess==RAM_Index
  int       *row_mat;           /* mat_alloc : From index 0, row_mat contains the
                                   row-ordered index of the elements of col_mat */
#elif MatrixColAccess==CAM_Record
  MATitem   *row_mat;           /* mat_alloc : From index 0, row_mat contains the
                                   row-ordered copy of the elements in col_mat */
#else /*if MatrixColAccess==CAM_Vector*/
  int       *row_mat_colnr;
  int       *row_mat_rownr;
  REAL      *row_mat_value;
#endif
  int       *row_end;           /* rows_alloc+1 : row_end[i] is the index of the
                                   first element in row_mat after row i */
  int       *row_tag;           /* user-definable tag associated with each row */

  REAL      *colmax;            /* Array of maximum values of each column */
  REAL      *rowmax;            /* Array of maximum values of each row */

  REAL      epsvalue;           /* Zero element rejection threshold */
  REAL      infnorm;            /* The largest absolute value in the matrix */
  REAL      dynrange;
  MYBOOL    row_end_valid;      /* TRUE if row_end & row_mat are valid */
  MYBOOL    is_roworder;        /* TRUE if the current (temporary) matrix order is row-wise */

} MATrec;

typedef struct _DeltaVrec
{
  lprec     *lp;
  int       activelevel;
  MATrec    *tracker;
} DeltaVrec;


#ifdef __cplusplus
__EXTERN_C {
#endif

/* Sparse matrix routines */
STATIC MATrec *mat_create(lprec *lp, int rows, int columns, REAL epsvalue);
STATIC MYBOOL mat_memopt(MATrec *mat, int rowextra, int colextra, int nzextra);
STATIC void mat_free(MATrec **matrix);
STATIC MYBOOL inc_matrow_space(MATrec *mat, int deltarows);
STATIC int mat_mapreplace(MATrec *mat, LLrec *rowmap, LLrec *colmap, MATrec *insmat);
STATIC int mat_matinsert(MATrec *mat, MATrec *insmat);
STATIC int mat_zerocompact(MATrec *mat);
STATIC int mat_rowcompact(MATrec *mat, MYBOOL dozeros);
STATIC int mat_colcompact(MATrec *mat, int prev_rows, int prev_cols);
STATIC MYBOOL inc_matcol_space(MATrec *mat, int deltacols);
STATIC MYBOOL inc_mat_space(MATrec *mat, int mindelta);
STATIC int mat_shiftrows(MATrec *mat, int *bbase, int delta, LLrec *varmap);
STATIC int mat_shiftcols(MATrec *mat, int *bbase, int delta, LLrec *varmap);
STATIC MATrec *mat_extractmat(MATrec *mat, LLrec *rowmap, LLrec *colmap, MYBOOL negated);
STATIC int mat_appendrow(MATrec *mat, int count, REAL *row, int *colno, REAL mult, MYBOOL checkrowmode);
STATIC int mat_appendcol(MATrec *mat, int count, REAL *column, int *rowno, REAL mult, MYBOOL checkrowmode);
MYBOOL mat_get_data(lprec *lp, int matindex, MYBOOL isrow, int **rownr, int **colnr, REAL **value);
MYBOOL mat_set_rowmap(MATrec *mat, int row_mat_index, int rownr, int colnr, int col_mat_index);
STATIC MYBOOL mat_indexrange(MATrec *mat, int index, MYBOOL isrow, int *startpos, int *endpos);
STATIC MYBOOL mat_validate(MATrec *mat);
STATIC MYBOOL mat_equalRows(MATrec *mat, int baserow, int comprow);
STATIC int mat_findelm(MATrec *mat, int row, int column);
STATIC int mat_findins(MATrec *mat, int row, int column, int *insertpos, MYBOOL validate);
STATIC void mat_multcol(MATrec *mat, int col_nr, REAL mult, MYBOOL DoObj);
STATIC REAL mat_getitem(MATrec *mat, int row, int column);
STATIC MYBOOL mat_setitem(MATrec *mat, int row, int column, REAL value);
STATIC MYBOOL mat_additem(MATrec *mat, int row, int column, REAL delta);
STATIC MYBOOL mat_setvalue(MATrec *mat, int Row, int Column, REAL Value, MYBOOL doscale);
STATIC int mat_nonzeros(MATrec *mat);
STATIC int mat_collength(MATrec *mat, int colnr);
STATIC int mat_rowlength(MATrec *mat, int rownr);
STATIC void mat_multrow(MATrec *mat, int row_nr, REAL mult);
STATIC void mat_multadd(MATrec *mat, REAL *lhsvector, int varnr, REAL mult);
STATIC MYBOOL mat_setrow(MATrec *mat, int rowno, int count, REAL *row, int *colno, MYBOOL doscale, MYBOOL checkrowmode);
STATIC MYBOOL mat_setcol(MATrec *mat, int colno, int count, REAL *column, int *rowno, MYBOOL doscale, MYBOOL checkrowmode);
STATIC MYBOOL mat_mergemat(MATrec *target, MATrec *source, MYBOOL usecolmap);
STATIC int mat_checkcounts(MATrec *mat, int *rownum, int *colnum, MYBOOL freeonexit);
STATIC int mat_expandcolumn(MATrec *mat, int colnr, REAL *column, int *nzlist, MYBOOL signedA);
STATIC MYBOOL mat_computemax(MATrec *mat);
STATIC MYBOOL mat_transpose(MATrec *mat);

/* Refactorization and recomputation routine */
MYBOOL __WINAPI invert(lprec *lp, MYBOOL shiftbounds, MYBOOL final);

/* Vector compression and expansion routines */
STATIC MYBOOL vec_compress(REAL *densevector, int startpos, int endpos, REAL epsilon, REAL *nzvector, int *nzindex);
STATIC MYBOOL vec_expand(REAL *nzvector, int *nzindex, REAL *densevector, int startpos, int endpos);

/* Sparse matrix products */
STATIC MYBOOL get_colIndexA(lprec *lp, int varset, int *colindex, MYBOOL append);
STATIC int prod_Ax(lprec *lp, int *coltarget, REAL *input, int *nzinput, REAL roundzero, REAL ofscalar, REAL *output, int *nzoutput, int roundmode);
STATIC int prod_xA(lprec *lp, int *coltarget, REAL *input, int *nzinput, REAL roundzero, REAL ofscalar, REAL *output, int *nzoutput, int roundmode);
STATIC MYBOOL prod_xA2(lprec *lp, int *coltarget, REAL *prow, REAL proundzero, int *pnzprow,
                                                  REAL *drow, REAL droundzero, int *dnzdrow, REAL ofscalar, int roundmode);

/* Equation solution */
STATIC MYBOOL fimprove(lprec *lp, REAL *pcol, int *nzidx, REAL roundzero);
STATIC void ftran(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero);
STATIC MYBOOL bimprove(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero);
STATIC void btran(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero);

/* Combined equation solution and matrix product for simplex operations */
STATIC MYBOOL fsolve(lprec *lp, int varin, REAL *pcol, int *nzidx, REAL roundzero, REAL ofscalar, MYBOOL prepareupdate);
STATIC MYBOOL bsolve(lprec *lp, int row_nr, REAL *rhsvector, int *nzidx, REAL roundzero, REAL ofscalar);
STATIC void bsolve_xA2(lprec *lp, int* coltarget, 
                                  int row_nr1, REAL *vector1, REAL roundzero1, int *nzvector1,
                                  int row_nr2, REAL *vector2, REAL roundzero2, int *nzvector2, int roundmode);

/* Change-tracking routines (primarily for B&B and presolve) */
STATIC DeltaVrec *createUndoLadder(lprec *lp, int levelitems, int maxlevels);
STATIC int incrementUndoLadder(DeltaVrec *DV);
STATIC MYBOOL modifyUndoLadder(DeltaVrec *DV, int itemno, REAL target[], REAL newvalue);
STATIC int countsUndoLadder(DeltaVrec *DV);
STATIC int restoreUndoLadder(DeltaVrec *DV, REAL target[]);
STATIC int decrementUndoLadder(DeltaVrec *DV);
STATIC MYBOOL freeUndoLadder(DeltaVrec **DV);

/* Specialized presolve undo functions */
STATIC MYBOOL appendUndoPresolve(lprec *lp, MYBOOL isprimal, REAL beta, int colnrDep);
STATIC MYBOOL addUndoPresolve(lprec *lp, MYBOOL isprimal, int colnrElim, REAL alpha, REAL beta, int colnrDep);


#ifdef __cplusplus
}
#endif

#endif /* HEADER_lp_matrix */

