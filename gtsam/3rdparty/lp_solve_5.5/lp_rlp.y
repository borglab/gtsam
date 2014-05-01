/* ========================================================================= */
/* NAME  : lp_rlp.y                                                          */
/* ========================================================================= */


%token VAR CONS INTCONS VARIABLECOLON INF SEC_INT SEC_BIN SEC_SEC SEC_SOS SOSDESCR SEC_FREE SIGN AR_M_OP RE_OPLE RE_OPGE END_C COMMA COLON MINIMISE MAXIMISE UNDEFINED


%{
#include <string.h>
#include <ctype.h>

#include "lpkit.h"
#include "yacc_read.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

static int HadVar0, HadVar1, HadVar2, HasAR_M_OP, do_add_row, Had_lineair_sum0, HadSign;
static char *Last_var = NULL, *Last_var0 = NULL;
static REAL f, f0, f1;
static int x;
static int state, state0;
static int Sign;
static int isign, isign0;      /* internal_sign variable to make sure nothing goes wrong */
                /* with lookahead */
static int make_neg;   /* is true after the relational operator is seen in order */
                /* to remember if lin_term stands before or after re_op */
static int Within_int_decl = FALSE; /* TRUE when we are within an int declaration */
static int Within_bin_decl = FALSE; /* TRUE when we are within an bin declaration */
static int Within_sec_decl = FALSE; /* TRUE when we are within a sec declaration */
static int Within_sos_decl = FALSE; /* TRUE when we are within a sos declaration */
static int Within_sos_decl1;
static int Within_free_decl = FALSE; /* TRUE when we are within a free declaration */
static short SOStype0; /* SOS type */
static short SOStype; /* SOS type */
static int SOSNr;
static int SOSweight = 0; /* SOS weight */

static int HadConstraint;
static int HadVar;
static int Had_lineair_sum;

extern FILE *yyin;

#define YY_FATAL_ERROR lex_fatal_error

/* let's please C++ users */
#ifdef __cplusplus
extern "C" {
#endif

static int wrap(void)
{
  return(1);
}

static int __WINAPI lp_input_yyin(void *fpin, char *buf, int max_size)
{
  int result;

  if ( (result = fread( (char*)buf, sizeof(char), max_size, (FILE *) fpin)) < 0)
    YY_FATAL_ERROR( "read() in flex scanner failed");

  return(result);
}

static read_modeldata_func *lp_input;

#undef YY_INPUT
#define YY_INPUT(buf,result,max_size) result = lp_input((void *) yyin, buf, max_size);

#ifdef __cplusplus
};
#endif

#define yywrap wrap
#define yyerror read_error

#include "lp_rlp.h"

%}

%start inputfile
%%

EMPTY: /* EMPTY */
                ;

inputfile       :
{
  isign = 0;
  make_neg = 0;
  Sign = 0;
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
}
                  objective_function
                  constraints
                  int_bin_sec_sos_free_declarations
                ;

/* start objective_function */

/*

 objective_function: MAXIMISE real_of | MINIMISE real_of | real_of;
 real_of:            lineair_sum END_C;
 lineair_sum:        EMPTY | x_lineair_sum;

*/

objective_function:   MAXIMISE real_of
{
  set_obj_dir(TRUE);
}
                    | MINIMISE real_of
{
  set_obj_dir(FALSE);
}
                    | real_of
                ;

real_of:            lineair_sum
                    END_C
{
  add_row();
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
  isign = 0;
  make_neg = 0;
}
                ;

lineair_sum:          EMPTY
                    | x_lineair_sum
                ;

/* end objective_function */



/* start constraints */

/*

 constraints:        EMPTY | x_constraints;
 x_constraints:      constraint | x_constraints constraint;
 constraint:         real_constraint | VARIABLECOLON real_constraint;
 real_constraint:    x_lineair_sum2 RE_OP x_lineair_sum3 optionalrange END_C;
 optionalrange:      EMPTY | RE_OP cons_term RHS_STORE;
 RE_OP:              RE_OPLE | RE_OPGE;
 cons_term:          x_SIGN REALCONS | INF;
 x_lineair_sum2:     EMPTY | x_lineair_sum3;
 x_lineair_sum3:     x_lineair_sum | INF RHS_STORE;
 x_lineair_sum:      x_lineair_sum1;
 x_lineair_sum1:     x_lineair_term | x_lineair_sum1 x_lineair_term;
 x_lineair_term:     x_SIGN x_lineair_term1;
 x_lineair_term1:    REALCONS | optional_AR_M_OP VAR;
 x_SIGN:             EMPTY | SIGN;
 REALCONS:           INTCONS | CONS;
 optional_AR_M_OP:   EMPTY | AR_M_OP;

*/

constraints:      EMPTY
                | x_constraints
                ;

x_constraints   : constraint
                | x_constraints
                  constraint
                ;

constraint      : real_constraint
                | VARIABLECOLON
{
  if(!add_constraint_name(Last_var))
    YYABORT;
  HadConstraint = TRUE;
}
                  real_constraint
                ;

real_constraint : x_lineair_sum2
{
  HadVar1 = HadVar0;
  HadVar0 = FALSE;
}
                  RE_OP
{
  if(!store_re_op((char *) yytext, HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
  make_neg = 1;
  f1 = 0;
}
                  x_lineair_sum3
{
  Had_lineair_sum0 = Had_lineair_sum;
  Had_lineair_sum = TRUE;
  HadVar2 = HadVar0;
  HadVar0 = FALSE;
  do_add_row = FALSE;
  if(HadConstraint && !HadVar ) {
    /* it is a range */
    /* already handled */
  }
  else if(!HadConstraint && HadVar) {
    /* it is a bound */

    if(!store_bounds(TRUE))
      YYABORT;
  }
  else {
    /* it is a row restriction */
    if(HadConstraint && HadVar)
      store_re_op("", HadConstraint, HadVar, Had_lineair_sum); /* makes sure that data stored in temporary buffers is treated correctly */
    do_add_row = TRUE;
  }
}
                  optionalrange
                  END_C
{
  if((!HadVar) && (!HadConstraint)) {
    yyerror("parse error");
    YYABORT;
  }
  if(do_add_row)
    add_row();
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
  isign = 0;
  make_neg = 0;
  null_tmp_store(TRUE);
}
                ;

optionalrange:    EMPTY
{
  if((!HadVar1) && (Had_lineair_sum0))
    if(!negate_constraint())
      YYABORT;
}
                | RE_OP
{
  make_neg = 0;
  isign = 0;
  if(HadConstraint)
    HadVar = Had_lineair_sum = FALSE;
  HadVar0 = FALSE;
  if(!store_re_op((char *) ((*yytext == '<') ? ">" : (*yytext == '>') ? "<" : yytext), HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
}
                  cons_term
{
  f -= f1;
}
                  RHS_STORE
{
  if((HadVar1) || (!HadVar2) || (HadVar0)) {
    yyerror("parse error");
    YYABORT;
  }

  if(HadConstraint && !HadVar ) {
    /* it is a range */
    /* already handled */
    if(!negate_constraint())
      YYABORT;
  }
  else if(!HadConstraint && HadVar) {
    /* it is a bound */

    if(!store_bounds(TRUE))
      YYABORT;
  }
}
                ;

x_lineair_sum2:   EMPTY
{
  /* to allow a range */
  /* constraint: < max */
  if(!HadConstraint) {
    yyerror("parse error");
    YYABORT;
  }
  Had_lineair_sum = FALSE;
}
                | x_lineair_sum3
{
  Had_lineair_sum = TRUE;
}
                ;

x_lineair_sum3  :  x_lineair_sum
                | INF
{
  isign = Sign;
}
                  RHS_STORE
                ;

x_lineair_sum:
{
  state = state0 = 0;
}
                x_lineair_sum1
{
  if (state == 1) {
    /* RHS_STORE */
    if (    (isign0 || !make_neg)
        && !(isign0 && !make_neg)) /* but not both! */
      f0 = -f0;
    if(make_neg)
      f1 += f0;
    if(!rhs_store(f0, HadConstraint, HadVar, Had_lineair_sum))
      YYABORT;
  }
}
                ;

x_lineair_sum1  : x_lineair_term
                | x_lineair_sum1
                  x_lineair_term
                ;

x_lineair_term  : x_SIGN
                  x_lineair_term1
{
  if ((HadSign || state == 1) && (state0 == 1)) {
    /* RHS_STORE */
    if (    (isign0 || !make_neg)
        && !(isign0 && !make_neg)) /* but not both! */
      f0 = -f0;
    if(make_neg)
      f1 += f0;
    if(!rhs_store(f0, HadConstraint, HadVar, Had_lineair_sum))
      YYABORT;
  }
  if (state == 1) {
    f0 = f;
    isign0 = isign;
  }
  if (state == 2) {
    if((HadSign) || (state0 != 1)) {
     isign0 = isign;
     f0 = 1.0;
    }
    if (    (isign0 || make_neg)
        && !(isign0 && make_neg)) /* but not both! */
      f0 = -f0;
    if(!var_store(Last_var, f0, HadConstraint, HadVar, Had_lineair_sum)) {
      yyerror("var_store failed");
      YYABORT;
    }
    HadConstraint |= HadVar;
    HadVar = HadVar0 = TRUE;
  }
  state0 = state;
}
                ;

x_lineair_term1 : REALCONS
{
  state = 1;
}
                | optional_AR_M_OP
{
  if ((HasAR_M_OP) && (state != 1)) {
    yyerror("parse error");
    YYABORT;
  }
}
                  VAR
{
  state = 2;
}
                ;

RE_OP: RE_OPLE | RE_OPGE
                ;

cons_term:        x_SIGN
                  REALCONS
                | INF
{
  isign = Sign;
}
                ;

/* end constraints */


/* start common for objective & constraints */

REALCONS: INTCONS | CONS
                ;

x_SIGN:           EMPTY
{
  isign = 0;
  HadSign = FALSE;
}
                | SIGN
{
  isign = Sign;
  HadSign = TRUE;
}
                ;

optional_AR_M_OP: EMPTY
{
  HasAR_M_OP = FALSE;
}
                | AR_M_OP
{
  HasAR_M_OP = TRUE;
}
                ;

RHS_STORE:        EMPTY
{
  if (    (isign || !make_neg)
      && !(isign && !make_neg)) /* but not both! */
    f = -f;
  if(!rhs_store(f, HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
  isign = 0;
}
                ;

/* end common for objective & constraints */



/* start int_bin_sec_sos_free_declarations */

int_bin_sec_sos_free_declarations:
                  EMPTY
                | real_int_bin_sec_sos_free_decls
                ;

real_int_bin_sec_sos_free_decls: int_bin_sec_sos_free_declaration
                | real_int_bin_sec_sos_free_decls int_bin_sec_sos_free_declaration
                ;

SEC_INT_BIN_SEC_SOS_FREE: SEC_INT | SEC_BIN | SEC_SEC | SEC_SOS | SEC_FREE
                ;

int_bin_sec_sos_free_declaration:
                  SEC_INT_BIN_SEC_SOS_FREE
{
  Within_sos_decl1 = Within_sos_decl;
}
                  x_int_bin_sec_sos_free_declaration
                ;

xx_int_bin_sec_sos_free_declaration:
{
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl1) && (!Within_free_decl)) {
    yyerror("parse error");
    YYABORT;
  }
  SOStype = SOStype0;
  check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, Within_sos_decl1 = (Within_sos_decl1 ? 1 : 0), Within_free_decl);
}
                  optionalsos
                  vars
                  optionalsostype
                  END_C
{
  if((Within_sos_decl1) && (SOStype == 0))
  {
    yyerror("Unsupported SOS type (0)");
    YYABORT;
  }
}
                ;

x_int_bin_sec_sos_free_declaration:
                  xx_int_bin_sec_sos_free_declaration
                | x_int_bin_sec_sos_free_declaration xx_int_bin_sec_sos_free_declaration
                ;

optionalsos:      EMPTY
                | SOSDESCR
{
  FREE(Last_var0);
  Last_var0 = strdup(Last_var);
}
                  sosdescr
                ;

optionalsostype:  EMPTY
{
  if(Within_sos_decl1) {
    set_sos_type(SOStype);
    set_sos_weight((double) SOSweight, 1);
  }
}
                | RE_OPLE
                  INTCONS
{
  if((Within_sos_decl1) && (!SOStype))
  {
    set_sos_type(SOStype = (short) (f + .1));
  }
  else
  {
    yyerror("SOS type not expected");
    YYABORT;
  }
}
                optionalSOSweight
                ;

optionalSOSweight:EMPTY
{
  set_sos_weight((double) SOSweight, 1);
}
                | COLON
                  INTCONS
{
  set_sos_weight(f, 1);
}
                ;

vars:             EMPTY
                | x_vars
                ;

x_vars          : onevarwithoptionalweight
                | x_vars
                  optionalcomma
                  onevarwithoptionalweight
                ;

optionalcomma:    EMPTY
                | COMMA
                ;

variable:         EMPTY
{
  if(Within_sos_decl1 == 1)
  {
    char buf[16];

    SOSweight++;
    sprintf(buf, "SOS%d", SOSweight);
    storevarandweight(buf);

    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;
  }

  storevarandweight(Last_var);

  if(Within_sos_decl1 == 2)
  {
    SOSNr++;
    set_sos_weight((double) SOSNr, 2);
  }
}
                ;

variablecolon:
{
  if(!Within_sos_decl1) {
    yyerror("parse error");
    YYABORT;
  }
  if(Within_sos_decl1 == 1) {
    FREE(Last_var0);
    Last_var0 = strdup(Last_var);
  }
  if(Within_sos_decl1 == 2)
  {
    storevarandweight(Last_var);
    SOSNr++;
    set_sos_weight((double) SOSNr, 2);
  }
}
                ;

sosweight:        EMPTY
{
  if(Within_sos_decl1 == 1)
  {
    char buf[16];

    SOSweight++;
    sprintf(buf, "SOS%d", SOSweight);
    storevarandweight(buf);

    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;

    storevarandweight(Last_var0);
    SOSNr++;
  }

  set_sos_weight(f, 2);
}
                ;

sosdescr:         EMPTY
{ /* SOS name */
  if(Within_sos_decl1 == 1)
  {
    storevarandweight(Last_var0);
    set_sos_type(SOStype);
    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;
    SOSweight++;
  }
}
                ;

onevarwithoptionalweight:
                  VAR
                  variable
                | VARIABLECOLON
                  variablecolon
                  INTCONSorVARIABLE
                ;

INTCONSorVARIABLE:REALCONS /* INTCONS */
                  sosweight
                | sosdescr
                  x_onevarwithoptionalweight
                ;

x_onevarwithoptionalweight:
                  VAR
                  variable
                | VARIABLECOLON
                  variablecolon
                  REALCONS /* INTCONS */
                  sosweight
                ;

/* end int_bin_sec_sos_free_declarations */

%%

static void yy_delete_allocated_memory(void)
{
  /* free memory allocated by flex. Otherwise some memory is not freed.
     This is a bit tricky. There is not much documentation about this, but a lot of
     reports of memory that keeps allocated */

  /* If you get errors on this function call, just comment it. This will only result
     in some memory that is not being freed. */

# if defined YY_CURRENT_BUFFER
    /* flex defines the macro YY_CURRENT_BUFFER, so you should only get here if lp_rlp.h is
       generated by flex */
    /* lex doesn't define this macro and thus should not come here, but lex doesn't has
       this memory leak also ...*/

    yy_delete_buffer(YY_CURRENT_BUFFER); /* comment this line if you have problems with it */
    yy_init = 1; /* make sure that the next time memory is allocated again */
    yy_start = 0;
# endif

  FREE(Last_var);
  FREE(Last_var0);
}

static int parse(void)
{
  return(yyparse());
}

lprec *read_lp1(lprec *lp, void *userhandle, read_modeldata_func read_modeldata, int verbose, char *lp_name)
{
  yyin = (FILE *) userhandle;
  yyout = NULL;
  yylineno = 1;
  lp_input = read_modeldata;
  return(yacc_read(lp, verbose, lp_name, &yylineno, parse, yy_delete_allocated_memory));
}

lprec * __WINAPI read_lp(FILE *filename, int verbose, char *lp_name)
{
  return(read_lp1(NULL, filename, lp_input_yyin, verbose, lp_name));
}

lprec * __WINAPI read_lpex(void *userhandle, read_modeldata_func read_modeldata, int verbose, char *lp_name)
{
  return(read_lp1(NULL, userhandle, read_modeldata, verbose, lp_name));
}

lprec *read_LP1(lprec *lp, char *filename, int verbose, char *lp_name)
{
  FILE *fpin;

  if((fpin = fopen(filename, "r")) != NULL) {
    lp = read_lp1(lp, fpin, lp_input_yyin, verbose, lp_name);
    fclose(fpin);
  }
  else
    lp = NULL;
  return(lp);
}

lprec * __WINAPI read_LP(char *filename, int verbose, char *lp_name)
{
  return(read_LP1(NULL, filename, verbose, lp_name));
}

MYBOOL __WINAPI LP_readhandle(lprec **lp, FILE *filename, int verbose, char *lp_name)
{
  if(lp != NULL)
    *lp = read_lp1(*lp, filename, lp_input_yyin, verbose, lp_name);

  return((lp != NULL) && (*lp != NULL));
}
