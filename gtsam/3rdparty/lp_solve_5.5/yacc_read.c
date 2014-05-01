/*
   ============================================================================
   NAME : yacc_read.c

   PURPOSE : translation of lp-problem and storage in sparse matrix

   SHORT : Subroutines for yacc program to store the input in an intermediate
   data-structure. The yacc and lex programs translate the input.  First the
   problemsize is determined and the date is read into an intermediate
   structure, then readinput fills the sparse matrix.

   USAGE : call yyparse(); to start reading the input.  call readinput(); to
   fill the sparse matrix.
   ============================================================================
   Rows : contains the amount of rows + 1. Rows-1 is the amount of constraints
   (no bounds) Rows also contains the rownr 0 which is the objective function

   Columns : contains the amount of columns (different variable names found in
   the constraints)

   Nonnuls : contains the amount of nonnuls = sum of different entries of all
   columns in the constraints and in the objectfunction

   Hash_tab : contains all columnnames on the first level of the structure the
   row information is kept under each column structure in a linked list (also
   the objective funtion is in this structure) Bound information is also
   stored under under the column name

   First_rside : points to a linked list containing all relational operators
   and the righthandside values of the constraints the linked list is in
   reversed order with respect to the rownumbers
   ============================================================================ */
#include <string.h>
#include <limits.h>
#include <setjmp.h>
#include "lpkit.h"
#include "yacc_read.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


#define tol 1.0e-10
#define coldatastep 100

#define HASHSIZE       10007  /* A prime number! */

static short      Maximise;
static short      Ignore_int_decl;
static short      int_decl;
static short      Ignore_sec_decl;
static short      sos_decl;
static short      Ignore_free_decl;
static int        Rows;
static int        Columns;
static int        Non_zeros;
static short      *relat;
static int        Verbose;
static hashtable  *Hash_tab;
static hashtable  *Hash_constraints;
static jmp_buf    jump_buf;
static int        *lineno;
static int        Lin_term_count;
static char       *title;

struct structSOSvars {
  char                 *name;
  int                  col;
  REAL                 weight;
  struct structSOSvars *next;
};

static struct structSOS {
  char                 *name;
  short                type;
  int                  Nvars;
  int                  weight;
  struct structSOSvars *SOSvars, *LastSOSvars;
  struct structSOS     *next;
} *FirstSOS, *LastSOS;

struct SOSrow {
  int  col;
  REAL value;
  struct SOSrow *next;
};

struct SOSrowdata {
  short type;
  char *name;
  struct SOSrow *SOSrow;
};

static struct _tmp_store_struct
{
  char    *name;
  int     row;
  REAL    value;
  REAL    rhs_value;
  short   relat;
} tmp_store;

static struct rside /* contains relational operator and rhs value */
{
  int           row;
  REAL          value;
  REAL          range_value;
  struct rside  *next;
  short         relat;
  short         range_relat;
  char		negate;
  short         SOStype;
} *First_rside, *rs;

struct column
{
  int            row;
  REAL           value;
  struct  column *next;
  struct  column *prev;
};

static struct structcoldata {
  int               must_be_int;
  int               must_be_sec;
  int               must_be_free;
  REAL              upbo;
  REAL              lowbo;
  struct  column   *firstcol;
  struct  column   *col;
} *coldata;

static void error(int verbose, char *string)
{
  if(Verbose >= verbose)
    report(NULL, verbose, "%s on line %d\n", string, *lineno);
}

/*
 * error handling routine for yyparse()
 */
void read_error(char *string)
{
  error(CRITICAL, string);
}

/* called when lex gets a fatal error */
void lex_fatal_error(char *msg)
{
  read_error(msg);
  longjmp(jump_buf, 1);
}

void add_row()
{
  Rows++;
  rs = NULL;
  Lin_term_count = 0;
}

void add_sos_row(short SOStype)
{
  if (rs != NULL)
    rs->SOStype = SOStype;
  Rows++;
  rs = NULL;
  Lin_term_count = 0;
}

void check_int_sec_sos_free_decl(int within_int_decl, int within_sec_decl, int sos_decl0, int within_free_decl)
{
  Ignore_int_decl = TRUE;
  Ignore_sec_decl = TRUE;
  Ignore_free_decl = TRUE;
  sos_decl = 0;
  if(within_int_decl) {
    Ignore_int_decl = FALSE;
    int_decl = (short) within_int_decl;
    if(within_sec_decl)
      Ignore_sec_decl = FALSE;
  }
  else if(within_sec_decl) {
    Ignore_sec_decl = FALSE;
  }
  else if(sos_decl0) {
    sos_decl = (short) sos_decl0;
  }
  else if(within_free_decl) {
    Ignore_free_decl = FALSE;
  }
}

static void add_int_var(char *name, short int_decl)
{
  hashelem *hp;

  if((hp = findhash(name, Hash_tab)) == NULL) {
    char buf[256];

    sprintf(buf, "Unknown variable %s declared integer, ignored", name);
    error(NORMAL, buf);
  }
  else if(coldata[hp->index].must_be_int) {
    char buf[256];

    sprintf(buf, "Variable %s declared integer more than once, ignored", name);
    error(NORMAL, buf);
  }
  else {
    coldata[hp->index].must_be_int = TRUE;
    if(int_decl == 2) {
      if(coldata[hp->index].lowbo != -DEF_INFINITE * (REAL) 10.0) {
        char buf[256];

        sprintf(buf, "Variable %s: lower bound on variable redefined", name);
        error(NORMAL, buf);
      }
      coldata[hp->index].lowbo = 0;
      if(coldata[hp->index].upbo < DEF_INFINITE) {
        char buf[256];

        sprintf(buf, "Variable %s: upper bound on variable redefined", name);
        error(NORMAL, buf);
      }
      coldata[hp->index].upbo = 1;
    }
    else if(int_decl == 3) {
      if(coldata[hp->index].upbo == DEF_INFINITE)
        coldata[hp->index].upbo = 1.0;
    }
  }
}

static void add_sec_var(char *name)
{
  hashelem *hp;

  if((hp = findhash(name, Hash_tab)) == NULL) {
    char buf[256];

    sprintf(buf, "Unknown variable %s declared semi-continuous, ignored", name);
    error(NORMAL, buf);
  }
  else if(coldata[hp->index].must_be_sec) {
    char buf[256];

    sprintf(buf, "Variable %s declared semi-continuous more than once, ignored", name);
    error(NORMAL, buf);
  }
  else
    coldata[hp->index].must_be_sec = TRUE;
}

int set_sec_threshold(char *name, REAL threshold)
{
  hashelem *hp;

  if((hp = findhash(name, Hash_tab)) == NULL) {
    char buf[256];

    sprintf(buf, "Unknown variable %s declared semi-continuous, ignored", name);
    error(NORMAL, buf);
    return(FALSE);
  }

  if ((coldata[hp->index].lowbo > 0.0) && (threshold > 0.0)) {
    char buf[256];

    coldata[hp->index].must_be_sec = FALSE;
    sprintf(buf, "Variable %s declared semi-continuous, but it has a non-negative lower bound (%f), ignored", name, coldata[hp->index].lowbo);
    error(NORMAL, buf);
  }
  if (threshold > coldata[hp->index].lowbo)
    coldata[hp->index].lowbo = threshold;

  return(coldata[hp->index].must_be_sec);
}

static void add_free_var(char *name)
{
  hashelem *hp;

  if((hp = findhash(name, Hash_tab)) == NULL) {
    char buf[256];

    sprintf(buf, "Unknown variable %s declared free, ignored", name);
    error(NORMAL, buf);
  }
  else if(coldata[hp->index].must_be_free) {
    char buf[256];

    sprintf(buf, "Variable %s declared free more than once, ignored", name);
    error(NORMAL, buf);
  }
  else
    coldata[hp->index].must_be_free = TRUE;
}

static int add_sos_name(char *name)
{
  struct structSOS *SOS;

  if(CALLOC(SOS, 1, struct structSOS) == NULL)
    return(FALSE);

  if(MALLOC(SOS->name, strlen(name) + 1, char) == NULL)
  {
    FREE(SOS);
    return(FALSE);
  }
  strcpy(SOS->name, name);
  SOS->type = 0;

  if(FirstSOS == NULL)
    FirstSOS = SOS;
  else
    LastSOS->next = SOS;
  LastSOS = SOS;

  return(TRUE);
}

static int add_sos_var(char *name)
{
  struct structSOSvars *SOSvar;

  if(name != NULL) {
    if(CALLOC(SOSvar, 1, struct structSOSvars) == NULL)
      return(FALSE);

    if(MALLOC(SOSvar->name, strlen(name) + 1, char) == NULL)
    {
      FREE(SOSvar);
      return(FALSE);
    }
    strcpy(SOSvar->name, name);

    if(LastSOS->SOSvars == NULL)
      LastSOS->SOSvars = SOSvar;
    else
      LastSOS->LastSOSvars->next = SOSvar;
    LastSOS->LastSOSvars = SOSvar;
    LastSOS->Nvars = LastSOS->Nvars + 1;
  }
  LastSOS->LastSOSvars->weight = 0;

  return(TRUE);
}

void storevarandweight(char *name)
{
  if(!Ignore_int_decl) {
    add_int_var(name, int_decl);
    if(!Ignore_sec_decl)
      add_sec_var(name);
  }
  else if(!Ignore_sec_decl)
    add_sec_var(name);
  else if(sos_decl==1)
    add_sos_name(name);
  else if(sos_decl==2)
    add_sos_var(name);
  else if(!Ignore_free_decl)
    add_free_var(name);
}

int set_sos_type(int SOStype)
{
  if(LastSOS != NULL)
    LastSOS->type = (short) SOStype;
  return(TRUE);
}

int set_sos_weight(double weight, int sos_decl)
{
  if(LastSOS != NULL)
    if(sos_decl==1)
      LastSOS->weight = (int) (weight+.1);
    else
      LastSOS->LastSOSvars->weight = weight;
  return(TRUE);
}

void set_obj_dir(int maximise)
{
  Maximise = (short) maximise;
}

static int inccoldata(void)
{
  if(Columns == 0)
    CALLOC(coldata, coldatastep, struct structcoldata);
  else if((Columns%coldatastep) == 0)
    REALLOC(coldata, Columns + coldatastep, struct structcoldata);

  if(coldata != NULL) {
    coldata[Columns].upbo = (REAL) DEF_INFINITE;
    coldata[Columns].lowbo = (REAL) -DEF_INFINITE * (REAL) 10.0; /* temporary. If still this value then 0 will be taken */
    coldata[Columns].col = NULL;
    coldata[Columns].firstcol = NULL;
    coldata[Columns].must_be_int = FALSE;
    coldata[Columns].must_be_sec = FALSE;
    coldata[Columns].must_be_free = FALSE;
  }

  return(coldata != NULL);
}

/*
 * initialisation of hashstruct and globals.
 */
static int init_read(int verbose)
{
  int ok = FALSE;

  Verbose = verbose;
  set_obj_dir(TRUE);
  Rows = 0;
  Non_zeros = 0;
  Columns = 0;
  FirstSOS = LastSOS = NULL;
  Lin_term_count = 0;
  if (CALLOC(First_rside, 1, struct rside) != NULL) {
    rs = First_rside;
    rs->value = rs->range_value = 0;
    /* first row (nr 0) is always the objective function */
    rs->relat = OF;
    rs->range_relat = -1;
    rs->SOStype = 0;
    Hash_tab = NULL;
    Hash_constraints = NULL;
    if (((Hash_tab = create_hash_table(HASHSIZE, 0)) == NULL) ||
        ((Hash_constraints = create_hash_table(HASHSIZE, 0)) == NULL)){
      FREE(First_rside);
      FREE(Hash_tab);
      FREE(Hash_constraints);
    }
    else
      ok = TRUE;
  }
  return(ok);
} /* init */

/*
 * clears the tmp_store variable after all information has been copied
 */
void null_tmp_store(int init_Lin_term_count)
{
  tmp_store.value = 0;
  tmp_store.rhs_value = 0;
  FREE(tmp_store.name);
  if(init_Lin_term_count)
    Lin_term_count = 0;
}

/*
 * variable : pointer to text array with name of variable
 * row      : the rownumber of the constraint
 * value    : value of matrixelement
 *            A(row, variable).
 * Sign     : (global)  determines the sign of value.
 * store()  : stores value in matrix
 *	      A(row, variable). If A(row, variable) already contains data,
 *	      value is added to the existing value.
 */
static int store(char *variable,
		  int row,
		  REAL value)
{
  hashelem *h_tab_p;
  struct column *col_p;

  if(value == 0) {
    char buf[256];

    sprintf(buf, "(store) Warning, variable %s has an effective coefficient of 0, Ignored", variable);
    error(NORMAL, buf);
    /* return(TRUE); */
  }

  if((h_tab_p = findhash(variable, Hash_tab)) == NULL) {
    if (((h_tab_p = puthash(variable, Columns, NULL, Hash_tab)) == NULL)
       ) return(FALSE);
    inccoldata();
    Columns++; /* counter for calloc of final array */
    if(value) {
      if (CALLOC(col_p, 1, struct column) == NULL)
        return(FALSE);
      Non_zeros++; /* for calloc of final arrays */
      col_p->row = row;
      col_p->value = value;
      coldata[h_tab_p->index].firstcol = coldata[h_tab_p->index].col = col_p;
    }
  }
  else if((coldata[h_tab_p->index].col == NULL) || (coldata[h_tab_p->index].col->row != row)) {
    if(value) {
      if (CALLOC(col_p, 1, struct column) == NULL)
        return(FALSE);
      Non_zeros++; /* for calloc of final arrays */
      if(coldata[h_tab_p->index].col != NULL)
        coldata[h_tab_p->index].col->prev = col_p;
      else
        coldata[h_tab_p->index].firstcol = col_p;
      col_p->value = value;
      col_p->row = row;
      col_p->next = coldata[h_tab_p->index].col;
      coldata[h_tab_p->index].col = col_p;
    }
  }
  else if(value) {
    coldata[h_tab_p->index].col->value += value;
    if(fabs(coldata[h_tab_p->index].col->value) < tol) /* eliminitate rounding errors */
      coldata[h_tab_p->index].col->value = 0;
  }
  return(TRUE);
} /* store */

static int storefirst(void)
{
    struct rside *rp;

    if ((rs != NULL) && (rs->row == tmp_store.row))
      return(TRUE);

    /* make space for the rhs information */
    if (CALLOC(rp, 1, struct rside) == NULL)
      return(FALSE);
    rp->next = First_rside;
    First_rside = rs = rp;
    rs->row = /* row */ tmp_store.row;
    rs->value = tmp_store.rhs_value;
    rs->relat = tmp_store.relat;
    rs->range_relat = -1;
    rs->SOStype = 0;

    if(tmp_store.name != NULL) {
      if(tmp_store.value != 0) {
	if (!store(tmp_store.name, tmp_store.row, tmp_store.value))
	  return(FALSE);
      }
      else {
	char buf[256];

	sprintf(buf, "Warning, variable %s has an effective coefficient of 0, ignored", tmp_store.name);
	error(NORMAL, buf);
      }
    }
    null_tmp_store(FALSE);
    return(TRUE);
}

/*
 * store relational operator given in yylex[0] in the rightside list.
 * Also checks if it constraint was a bound and if so stores it in the
 * boundslist
 */
int store_re_op(char *yytext, int HadConstraint, int HadVar, int Had_lineair_sum)
{
  short tmp_relat;

  switch(yytext[0]) {

  case '=':
    tmp_relat = EQ;
    break;

  case '>':
    tmp_relat = GE;
    break;

  case '<':
    tmp_relat = LE;
    break;

  case 0:
    if(rs != NULL)
      tmp_relat = rs->relat;
    else
      tmp_relat = tmp_store.relat;
    break;

  default:
    {
      char buf[256];

      sprintf(buf, "Error: unknown relational operator %s", yytext);
      error(CRITICAL, buf);
    }
    return(FALSE);
    break;
  }

  if(/* Lin_term_count > 1 */ HadConstraint && HadVar) {/* it is not a bound */
    if(Lin_term_count <= 1)
      if(!storefirst())
        return(FALSE);
    rs->relat = tmp_relat;
  }
  else if(/* Lin_term_count == 0 */ HadConstraint && !Had_lineair_sum /* HadVar */ /* && (rs != NULL) */) { /* it is a range */
    if(Lin_term_count == 1)
      if(!storefirst())
        return(FALSE);
    if(rs == NULL) { /* range before row, already reported */
      error(CRITICAL, "Error: range for undefined row");
      return(FALSE);
    }

    if(rs->negate)
      switch (tmp_relat) {
      case LE:
        tmp_relat = GE;
        break;
      case GE:
        tmp_relat = LE;
        break;
      }

    if(rs->range_relat != -1) {
      error(CRITICAL, "Error: There was already a range for this row");
      return(FALSE);
    }
    else if(tmp_relat == rs->relat) {
      error(CRITICAL, "Error: relational operator for range is the same as relation operator for equation");
      return(FALSE);
    }
    else
      rs->range_relat = tmp_relat;
  }
  else /* could be a bound */
    tmp_store.relat = tmp_relat;

  return(TRUE);
} /* store_re_op */

int negate_constraint()
{
    if(rs != NULL)
      rs->negate = TRUE;

    return(TRUE);
}

/*
 * store RHS value in the rightside structure
 * if type = true then
 */
int rhs_store(REAL value, int HadConstraint, int HadVar, int Had_lineair_sum)
{
  if(/* Lin_term_count > 1 */ (HadConstraint && HadVar) || (Rows == 0)){ /* not a bound */
    if (Rows == 0)
      value = -value;
    /* if(Lin_term_count < 2) */
    if(rs == NULL)
      tmp_store.rhs_value += value;
    else

    if(rs == NULL) {
      error(CRITICAL, "Error: No variable specified");
      return(FALSE);
    }
    else
      rs->value += value;
  }
  else if(/* Lin_term_count == 0 */ HadConstraint && !HadVar) { /* a range */
    if(rs == NULL) /* if range before row, already reported */
      tmp_store.rhs_value += value;
    else if(rs->range_relat < 0) /* was a bad range; ignore */;
    else {
      if(rs->negate)
        value = -value;
      if(((rs->relat == LE) && (rs->range_relat == GE) &&
         (rs->value < value)) ||
        ((rs->relat == GE) && (rs->range_relat == LE) &&
         (rs->value > value)) ||
	((rs->relat == EQ) || (rs->range_relat == EQ))) {
        rs->range_relat = -2;
	error(CRITICAL, "Error: range restriction conflicts");
	return(FALSE);
      }
      else
        rs->range_value += value;
    }
  }
  else /* a bound */
    tmp_store.rhs_value += value;
  return(TRUE);
} /* RHS_store */

/*
 * store all data in the right place
 * count the amount of lineair terms in a constraint
 * only store in data-structure if the constraint is not a bound
 */
int var_store(char *var, REAL value, int HadConstraint, int HadVar, int Had_lineair_sum)
{
  int row;

  row = Rows;

  /* also in a bound the same var name can occur more than once. Check for
     this. Don't increment Lin_term_count */

  if(Lin_term_count != 1 || tmp_store.name == NULL || strcmp(tmp_store.name, var) != 0)
    Lin_term_count++;

  /* always store objective function with rownr == 0. */
  if(row == 0)
    return(store(var,  row,  value));

  if(Lin_term_count == 1) { /* don't store yet. could be a bound */
    if(MALLOC(tmp_store.name, strlen(var) + 1, char) != NULL)
      strcpy(tmp_store.name, var);
    tmp_store.row = row;
    tmp_store.value += value;
    return(TRUE);
  }

  if(Lin_term_count == 2) { /* now you can also store the first variable */
    if(!storefirst())
      return(FALSE);
    /* null_tmp_store(FALSE); */
  }

  return(store(var, row, value));
} /* var_store */



/*
 * store the information in tmp_store because it is a bound
 */
int store_bounds(int warn)
{
  if(tmp_store.value != 0) {
    hashelem *h_tab_p;
    REAL boundvalue;

    if((h_tab_p = findhash(tmp_store.name, Hash_tab)) == NULL) {
      /* a new columnname is found, create an entry in the hashlist */
      if ((h_tab_p = puthash(tmp_store.name, Columns, NULL, Hash_tab)) == NULL) {
        error(CRITICAL, "Not enough memory");
        return(FALSE);
      }
      inccoldata();
      Columns++; /* counter for calloc of final array */
    }

    if(tmp_store.value < 0) { /* divide by negative number, */
      /* relational operator may change */
      if(tmp_store.relat == GE)
	tmp_store.relat = LE;
      else if(tmp_store.relat == LE)
	tmp_store.relat = GE;
    }

    boundvalue = tmp_store.rhs_value / tmp_store.value;

#if FALSE
    /* Check sanity of bound; all variables should be positive */
    if(   ((tmp_store.relat == EQ) && (boundvalue < 0))
       || ((tmp_store.relat == LE) && (boundvalue < 0))) { /* Error */
      error(CRITICAL, "Error: variables must always be non-negative");
      return(FALSE);
    }
#endif

#if FALSE
    if((tmp_store.relat == GE) && (boundvalue <= 0)) /* Warning */
      error(NORMAL, "Warning: useless bound; variables are always >= 0");
#endif

    /* bound seems to be sane, add it */
    if((tmp_store.relat == GE) || (tmp_store.relat == EQ)) {
      if(boundvalue > coldata[h_tab_p->index].lowbo - tol)
	coldata[h_tab_p->index].lowbo = boundvalue;
      else if(warn)
        error(NORMAL, "Ineffective lower bound, ignored");
    }
    if((tmp_store.relat == LE) || (tmp_store.relat == EQ)) {
      if(boundvalue < coldata[h_tab_p->index].upbo + tol)
	coldata[h_tab_p->index].upbo  = boundvalue;
      else if (warn)
        error(NORMAL, "Ineffective upper bound, ignored");
    }

    /* check for empty range */
    if((warn) && (coldata[h_tab_p->index].upbo + tol < coldata[h_tab_p->index].lowbo)) {
      error(CRITICAL, "Error: bound contradicts earlier bounds");
      return(FALSE);
    }
  }
  else /* tmp_store.value = 0 ! */ {
    char buf[256];

    if((tmp_store.rhs_value == 0) ||
       ((tmp_store.rhs_value > 0) && (tmp_store.relat == LE)) ||
       ((tmp_store.rhs_value < 0) && (tmp_store.relat == GE))) {
      sprintf(buf, "Variable %s has an effective coefficient of 0 in bound, ignored",
	      tmp_store.name);
      if(warn)
        error(NORMAL, buf);
    }
    else {
      sprintf(buf, "Error, variable %s has an effective coefficient of 0 in bound",
	      tmp_store.name);
      error(CRITICAL, buf);
      return(FALSE);
    }
  }

  /* null_tmp_store(FALSE); */
  tmp_store.rhs_value = 0;

  return(TRUE);
} /* store_bounds */

int set_title(char *name)
{
  title = strdup(name);
  return(TRUE);
}

int add_constraint_name(char *name)
{
  int row;
  hashelem *hp;

  if((hp = findhash(name, Hash_constraints)) != NULL) {
    row = hp->index;
    rs = First_rside;
    while ((rs != NULL) && (rs->row != row))
      rs = rs->next;
  }
  else {
    row = Rows;
    if (((hp = puthash(name, row, NULL, Hash_constraints)) == NULL)
       ) return(FALSE);
    if(row)
      rs = NULL;
  }

  return(TRUE);
}

/*
 * transport the data from the intermediate structure to the sparse matrix
 * and free the intermediate structure
 */
static int readinput(lprec *lp)
{
  int    i, i1, count, index, col;
  struct column *cp, *tcp;
  hashelem *hp;
  struct rside *rp;
  signed char *negateAndSOS = NULL;
  REAL *row = NULL, a;
  int *rowno = NULL;
  MYBOOL SOSinMatrix = FALSE;
  struct SOSrowdata *SOSrowdata = NULL;
  struct SOSrow *SOSrow, *SOSrow1;

  if(lp != NULL) {
    if (CALLOC(negateAndSOS, 1 + Rows, signed char) == NULL)
      return(FALSE);

    rp = First_rside;
    for(i = Rows; (i >= 0) && (rp != NULL); i--) {
      if(rp->SOStype == 0)
        negateAndSOS[i] = (rp->negate ? -1 : 0);
      else
        negateAndSOS[i] = (signed char) rp->SOStype;

      rp = rp->next;
    }

    /* fill names with the rownames */
    hp = Hash_constraints->first;
    while(hp != NULL) {
      if (/* (negateAndSOS[hp->index] <= 0) && */ (!set_row_name(lp, hp->index, hp->name)))
        return(FALSE);
      hp = hp->nextelem;
    }
  }

  for(i = Rows; i >= 0; i--) {
    rp = First_rside;
    if((lp != NULL) && (rp != NULL)) {
      if(rp->SOStype == 0) {
        if (rp->negate) {
          switch (rp->relat) {
          case LE:
            rp->relat = GE;
            break;
          case GE:
            rp->relat = LE;
            break;
          }
          switch (rp->range_relat) {
          case LE:
            rp->range_relat = GE;
            break;
          case GE:
            rp->range_relat = LE;
            break;
          }
          rp->range_value = -rp->range_value;
          rp->value = -rp->value;
        }

        if((rp->range_relat >= 0) && (rp->value == lp->infinite)) {
          rp->value = rp->range_value;
          rp->relat = rp->range_relat;
          rp->range_relat = -1;
        }
        else if((rp->range_relat >= 0) && (rp->value == -lp->infinite)) {
          rp->value = rp->range_value;
          rp->relat = rp->range_relat;
          rp->range_relat = -1;
        }
        if ((rp->range_relat >= 0) && (rp->range_value == rp->value)) {
          rp->relat = EQ;
          rp->range_relat = EQ;
        }
	if(i) {
          set_constr_type(lp, i, rp->relat);
	  relat[i] = rp->relat;
	}
        set_rh(lp, i, rp->value);
        if (rp->range_relat >= 0)
          set_rh_range(lp, i, rp->range_value - rp->value);
      }
      else {
        SOSinMatrix = TRUE;
        if(i)
          relat[i] = rp->relat;
      }
    }
    if(rp != NULL) {
      First_rside = rp->next;
      free(rp); /* free memory when data has been read */
    }
    else
      First_rside = NULL;
  }

  while(First_rside != NULL) {
    rp = First_rside;
    First_rside = rp->next;
    free(rp); /* free memory when data has been read */
  }

  /* start reading the Hash_list structure */
  index = 0;

  if((SOSinMatrix) && (CALLOC(SOSrowdata, 1 + Rows, struct SOSrowdata) == NULL)) {
    FREE(negateAndSOS);
    FREE(row);
    FREE(rowno);
    return(FALSE);
  }

  if((lp != NULL) &&
     ((MALLOC(row, 1 + Rows, REAL) == NULL) || (MALLOC(rowno, 1 + Rows, int) == NULL))) {
    FREE(SOSrowdata);
    FREE(negateAndSOS);
    FREE(row);
    FREE(rowno);
    return(FALSE);
  }

  /* for(i = 0; i < Hash_tab->size; i++) {
    hp = Hash_tab->table[i]; */
    hp = Hash_tab->first;
    while(hp != NULL) {
      count = 0;
      index++;
      cp = coldata[hp->index].firstcol;
      col = hp->index + 1;
      while(cp != NULL) {
        if(lp != NULL) {
          if (negateAndSOS[cp->row] <= 0) {
            rowno[count] = cp->row;
  	    a = cp->value;
  	    if (negateAndSOS[cp->row])
  	      a = -a;
            row[count++] = a;
          }
	  else {
	    if (MALLOC(SOSrow, 1, struct SOSrow) == NULL) {
              FREE(SOSrowdata);
              FREE(negateAndSOS);
              FREE(row);
              FREE(rowno);
              return(FALSE);
	    }
	    if(SOSrowdata[cp->row].SOSrow == NULL)
	      SOSrowdata[cp->row].name = strdup(get_row_name(lp, cp->row));
	    SOSrow->next = SOSrowdata[cp->row].SOSrow;
	    SOSrowdata[cp->row].SOSrow = SOSrow;
	    SOSrowdata[cp->row].type = negateAndSOS[cp->row];
	    SOSrow->col = col;
	    SOSrow->value = cp->value;
	  }
	}
	tcp = cp;
	/* cp = cp->next; */
	cp = cp->prev;
	free(tcp); /* free memory when data has been read */
      }

      if(lp != NULL) {
        add_columnex(lp, count, row, rowno);
        /* check for bound */
        if(coldata[hp->index].lowbo == -DEF_INFINITE * 10.0)
  	  /* lp->orig_lowbo[Rows+index] = 0.0; */
          set_lowbo(lp, index, 0);
        else
  	  /* lp->orig_lowbo[Rows+index] = coldata[hp->index].lowbo; */
          set_lowbo(lp, index, coldata[hp->index].lowbo);
        /* lp->orig_upbo[Rows+index] = coldata[hp->index].upbo; */
        set_upbo(lp, index, coldata[hp->index].upbo);

        /* check if it must be an integer variable */
        if(coldata[hp->index].must_be_int) {
  	  /* lp->must_be_int[Rows + index]=TRUE; */
          set_int(lp, index, TRUE);
        }
        if(coldata[hp->index].must_be_sec) {
          set_semicont(lp, index, TRUE);
        }
        if(coldata[hp->index].must_be_free) {
          set_unbounded(lp, index);
        }

        /* copy name of column variable */
        if (!set_col_name(lp, index, hp->name)) {
	  FREE(SOSrowdata);
          FREE(negateAndSOS);
          FREE(row);
	  FREE(rowno);
          return(FALSE);
        }

        /* put matrix values in intermediate row */
        /* cp = hp->col; */
        /* cp = hp->firstcol; */
      }

      /* thp = hp; */
      /* hp = hp->next; */
      /* free(thp->name); */
      /* free(thp); */ /* free memory when data has been read */

      hp = hp->nextelem;

    }
    /* Hash_tab->table[i] = NULL; */

  FREE(coldata);

  if(SOSrowdata != NULL) {
    struct structSOS *structSOS;
    struct structSOSvars *SOSvars, *SOSvars1;
    int SOSweight = 0;

    for(i = 1; i <= Rows; i++) {
      SOSrow = SOSrowdata[i].SOSrow;
      if(SOSrow != NULL) {
	if(MALLOC(structSOS, 1, struct structSOS) == NULL) {
	  FREE(SOSrowdata);
          FREE(negateAndSOS);
          FREE(row);
	  FREE(rowno);
          return(FALSE);
	}
	structSOS->Nvars = 0;
	structSOS->type = SOSrowdata[i].type;
	structSOS->weight = ++SOSweight;
	structSOS->name = strdup(SOSrowdata[i].name);
	structSOS->LastSOSvars = NULL;
	structSOS->next = FirstSOS;
	FirstSOS = structSOS;
	SOSvars = NULL;
        while(SOSrow != NULL) {
	  SOSvars1 = SOSvars;
	  MALLOC(SOSvars, 1, struct structSOSvars);
	  SOSvars->next = SOSvars1;
	  SOSvars->col = SOSrow->col;
	  SOSvars->weight = SOSrow->value;
	  SOSvars->name = NULL;
	  structSOS->Nvars++;
          SOSrow1 = SOSrow->next;
	  FREE(SOSrow);
	  SOSrow = SOSrow1;
	}
	structSOS->SOSvars = SOSvars;
      }
    }
    FREE(SOSrowdata);
  }

  while(FirstSOS != NULL)
  {
    struct structSOSvars *SOSvars, *SOSvars1;
    int *sosvars, n, col;
    REAL *weights;
    hashelem *hp;

    LastSOS = FirstSOS;
    FirstSOS = FirstSOS->next;
    SOSvars = LastSOS->SOSvars;
    if(lp != NULL) {
      MALLOC(sosvars, LastSOS->Nvars, int);
      MALLOC(weights, LastSOS->Nvars, double);
    }
    else {
      sosvars = NULL;
      weights = NULL;
    }
    n = 0;
    while(SOSvars != NULL)
    {
      SOSvars1 = SOSvars;
      SOSvars = SOSvars->next;
      if(lp != NULL) {
        col = SOSvars1->col;
	if(col == 0)
          if((hp = findhash(SOSvars1->name, lp->colname_hashtab)) != NULL)
	    col = hp->index;
	if (col) {
          sosvars[n] = col;
	  weights[n++] = SOSvars1->weight;
	}
      }
      FREE(SOSvars1->name);
      FREE(SOSvars1);
    }
    if(lp != NULL) {
      add_SOS(lp, LastSOS->name, LastSOS->type, LastSOS->weight, n, sosvars, weights);
      FREE(weights);
      FREE(sosvars);
    }
    FREE(LastSOS->name);
    FREE(LastSOS);
  }

  if(negateAndSOS != NULL) {
    for(i1 = 0, i = 1; i <= Rows; i++)
      if(negateAndSOS[i] <= 0)
        relat[++i1] = relat[i];

#if 01
    for(i = Rows; i > 0; i--)
      if(negateAndSOS[i] > 0) {
        del_constraint(lp, i);
        Rows--;
      }
#endif
  }

  /* the following should be replaced by a call to the MPS print routine MB */

#if 0
  if(Verbose) {
    int j;

    printf("\n");
    printf("**********Data read**********\n");
    printf("Rows    : %d\n", Rows);
    printf("Columns : %d\n", Columns);
    printf("Nonnuls : %d\n", Non_zeros);
    printf("NAME          LPPROB\n");
    printf("ROWS\n");
    for(i = 0; i <= Rows; i++) {
      if(relat[i] == LE)
	printf(" L  ");
      else if(relat[i] == EQ)
	printf(" E  ");
      else if(relat[i] == GE)
	printf(" G  ");
      else if(relat[i] == OF)
	printf(" N  ");
      printf("%s\n", get_row_name(lp, i));
    }

    printf("COLUMNS\n");
    j = 0;
    for(i = 0; i < Non_zeros; i++) {
      if(i == lp->col_end[j])
	j++;
      printf("    %-8s  %-8s  %g\n", get_col_name(lp, j),
	     get_row_name(lp, lp->mat[i].row_nr), (double)lp->mat[i].value);
    }

    printf("RHS\n");
    for(i = 0; i <= Rows; i++) {
      printf("    RHS       %-8s  %g\n", get_row_name(lp, i),
	     (double)lp->orig_rhs[i]);
    }

    printf("RANGES\n");
    for(i = 1; i <= Rows; i++)
      if((lp->orig_upbo[i] != lp->infinite) && (lp->orig_upbo[i] != 0)) {
	printf("    RGS       %-8s  %g\n", get_row_name(lp, i),
	       (double)lp->orig_upbo[i]);
      }
      else if((lp->orig_lowbo[i] != 0)) {
	printf("    RGS       %-8s  %g\n", get_row_name(lp, i),
	       (double)-lp->orig_lowbo[i]);
      }

    printf("BOUNDS\n");
    for(i = Rows + 1; i <= Rows + Columns; i++) {
      if((lp->orig_lowbo[i] != 0) && (lp->orig_upbo[i] < lp->infinite) &&
         (lp->orig_lowbo[i] == lp->orig_upbo[i])) {
        printf(" FX BND       %-8s  %g\n", get_col_name(lp, i - Rows),
               (double)lp->orig_upbo[i]);
      }
      else {
        if(lp->orig_upbo[i] < lp->infinite)
  	  printf(" UP BND       %-8s  %g\n", get_col_name(lp, i - Rows),
  		 (double)lp->orig_upbo[i]);
        if(lp->orig_lowbo[i] > 0)
  	  printf(" LO BND       %-8s  %g\n", get_col_name(lp, i - Rows),
  		 (double)lp->orig_lowbo[i]);
      }
    }

    printf("ENDATA\n");
  }
#endif

  FREE(row);
  FREE(rowno);
  FREE(negateAndSOS);
  return(TRUE);
} /* readinput */

lprec *yacc_read(lprec *lp, int verbose, char *lp_name, int *_lineno, int (*parse) (void), void (*delete_allocated_memory) (void))
{
  REAL *orig_upbo;
  int stat = -1;
  lprec *lp0 = lp;

  lineno = _lineno;
  title = lp_name;

  if(!init_read(verbose))
    error(CRITICAL, "init_read failed");
  else if (setjmp(jump_buf) == 0)
    stat = parse();

  delete_allocated_memory();

  Rows--;

  relat = NULL;
  if((stat != 0) || (CALLOC(relat, Rows + 1, short) != NULL)) {
    if(stat == 0) {
      if(lp == NULL) {
        lp = make_lp(Rows, 0);
      }
      else {
        int NRows;

	for(NRows = get_Nrows(lp); NRows < Rows; NRows++)
	  add_constraintex(lp, 0, NULL, NULL, LE, 0);
      }
    }
    else
      lp = NULL;
    if ((stat != 0) || (lp != NULL)) {
      if(lp != NULL) {
        set_verbose(lp, Verbose);
      }

      if (!readinput(lp)) {
	if((lp != NULL) && (lp0 == NULL))
          delete_lp(lp);
	lp = NULL;
      }

      if(lp != NULL) {
	set_lp_name(lp, title);
	if(Maximise)
	  set_maxim(lp);

	if(Rows) {
	  int row;

	  MALLOCCPY(orig_upbo, lp->orig_upbo, 1 + Rows, REAL);
	  for(row = 1; row <= Rows; row++)
	    set_constr_type(lp, row, relat[row]);

	  memcpy(lp->orig_upbo, orig_upbo, (1 + Rows) * sizeof(*orig_upbo)); /* restore upper bounds (range) */
	  FREE(orig_upbo);
	}
      }
      if((title != NULL) && (title != lp_name))
        free(title);

      free_hash_table(Hash_tab);
      free_hash_table(Hash_constraints);
    }
    FREE(relat);
  }
  null_tmp_store(FALSE);
  return(lp);
}
