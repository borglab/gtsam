#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

#include "commonlib.h"
#include "lp_lib.h"
#include "lp_report.h"
#include "ini.h"

typedef int (__WINAPI int_get_function)(lprec *lp);
typedef long (__WINAPI long_get_function)(lprec *lp);
typedef MYBOOL (__WINAPI MYBOOL_get_function)(lprec *lp);
typedef REAL (__WINAPI REAL_get_function)(lprec *lp);
typedef void (__WINAPI int_set_function)(lprec *lp, int value);
typedef void (__WINAPI long_set_function)(lprec *lp, long value);
typedef void (__WINAPI MYBOOL_set_function)(lprec *lp, MYBOOL value);
typedef void (__WINAPI REAL_set_function)(lprec *lp, REAL value);

#define intfunction    1
#define longfunction   2
#define MYBOOLfunction 3
#define REALfunction   4

#define setvalues(values, basemask) values, sizeof(values) / sizeof(*values), basemask
#define setNULLvalues NULL, 0, 0
#define setvalue(value) value, #value
#define setintfunction(get_function, set_function) get_function, set_function, intfunction
#define setlongfunction(get_function, set_function) (int_get_function *) get_function, (int_set_function *) set_function, longfunction
#define setMYBOOLfunction(get_function, set_function) (int_get_function *) get_function, (int_set_function *) set_function, MYBOOLfunction
#define setREALfunction(get_function, set_function) (int_get_function *) get_function, (int_set_function *) set_function, REALfunction

#define WRITE_COMMENTED 0
#define WRITE_ACTIVE    1

struct _values {
  int value;
  char *svalue;
};

struct _functions {
  char *par;                                    /* name of parameter in ini file */
  union {
    int_get_function *int_get_function;         /* set via setintfunction */
    long_get_function *long_get_function;       /* set via setlongfunction */
    MYBOOL_get_function *MYBOOL_get_function;   /* set via setMYBOOLfunction */
    REAL_get_function *REAL_get_function;       /* set via setREALfunction */
  } get_function;
  union {
    int_set_function *int_set_function;         /* set via setintfunction */
    long_set_function *long_set_function;       /* set via setlongfunction */
    MYBOOL_set_function *MYBOOL_set_function;   /* set via setMYBOOLfunction */
    REAL_set_function *REAL_set_function;       /* set via setREALfunction */
  } set_function;
  int type;                                     /* set via set*function */
  struct _values *values;                       /* set via setvalues to a structure of _values */
  int elements;                                 /*  or via setNULLvalues if the value is shown as is */
  unsigned int basemask;
  int mask;                                     /* WRITE_ACTIVE or WRITE_COMMENTED */
};

static struct _values anti_degen[] =
{
  { setvalue(ANTIDEGEN_NONE) },
  { setvalue(ANTIDEGEN_FIXEDVARS) },
  { setvalue(ANTIDEGEN_COLUMNCHECK) },
  { setvalue(ANTIDEGEN_STALLING) },
  { setvalue(ANTIDEGEN_NUMFAILURE) },
  { setvalue(ANTIDEGEN_LOSTFEAS) },
  { setvalue(ANTIDEGEN_INFEASIBLE) },
  { setvalue(ANTIDEGEN_DYNAMIC) },
  { setvalue(ANTIDEGEN_DURINGBB) },
  { setvalue(ANTIDEGEN_RHSPERTURB) },
  { setvalue(ANTIDEGEN_BOUNDFLIP) },
};

static struct _values basiscrash[] =
{
  { setvalue(CRASH_NONE) },
  /* { setvalue(CRASH_NONBASICBOUNDS) }, */ /* not yet implemented */
  { setvalue(CRASH_MOSTFEASIBLE) },
  { setvalue(CRASH_LEASTDEGENERATE) },
};

static struct _values bb_floorfirst[] =
{
  { setvalue(BRANCH_CEILING) },
  { setvalue(BRANCH_FLOOR) },
  { setvalue(BRANCH_AUTOMATIC) },
};

static struct _values bb_rule[] =
{
  { setvalue(NODE_FIRSTSELECT) },
  { setvalue(NODE_GAPSELECT) },
  { setvalue(NODE_RANGESELECT) },
  { setvalue(NODE_FRACTIONSELECT) },
  { setvalue(NODE_PSEUDOCOSTSELECT) },
  { setvalue(NODE_PSEUDONONINTSELECT) },
  { setvalue(NODE_PSEUDORATIOSELECT) },
  { setvalue(NODE_USERSELECT) },
  { setvalue(NODE_WEIGHTREVERSEMODE) },
  { setvalue(NODE_BRANCHREVERSEMODE) },
  { setvalue(NODE_GREEDYMODE) },
  { setvalue(NODE_PSEUDOCOSTMODE) },
  { setvalue(NODE_DEPTHFIRSTMODE) },
  { setvalue(NODE_RANDOMIZEMODE) },
  { setvalue(NODE_GUBMODE) },
  { setvalue(NODE_DYNAMICMODE) },
  { setvalue(NODE_RESTARTMODE) },
  { setvalue(NODE_BREADTHFIRSTMODE) },
  { setvalue(NODE_AUTOORDER) },
  { setvalue(NODE_RCOSTFIXING) },
  { setvalue(NODE_STRONGINIT) },
};

static struct _values improve[] =
{
  { setvalue(IMPROVE_NONE) },
  { setvalue(IMPROVE_SOLUTION) },
  { setvalue(IMPROVE_DUALFEAS) },
  { setvalue(IMPROVE_THETAGAP) },
  { setvalue(IMPROVE_BBSIMPLEX) },
};

static REAL __WINAPI get_mip_gap_abs(lprec *lp)
{
  return(get_mip_gap(lp, TRUE));
}

static REAL __WINAPI get_mip_gap_rel(lprec *lp)
{
  return(get_mip_gap(lp, FALSE));
}

static void __WINAPI set_mip_gap_abs(lprec *lp, REAL mip_gap)
{
  set_mip_gap(lp, TRUE, mip_gap);
}

static void __WINAPI set_mip_gap_rel(lprec *lp, REAL mip_gap)
{
  set_mip_gap(lp, FALSE, mip_gap);
}

static struct _values pivoting[] =
{
  { setvalue(PRICER_FIRSTINDEX) },
  { setvalue(PRICER_DANTZIG) },
  { setvalue(PRICER_DEVEX) },
  { setvalue(PRICER_STEEPESTEDGE) },
  { setvalue(PRICE_PRIMALFALLBACK) },
  { setvalue(PRICE_MULTIPLE) },
  { setvalue(PRICE_PARTIAL) },
  { setvalue(PRICE_ADAPTIVE) },
  { setvalue(PRICE_RANDOMIZE) },
  { setvalue(PRICE_AUTOPARTIAL) },
  { setvalue(PRICE_LOOPLEFT) },
  { setvalue(PRICE_LOOPALTERNATE) },
  { setvalue(PRICE_HARRISTWOPASS) },
  { setvalue(PRICE_TRUENORMINIT) },
};

static struct _values presolving[] =
{
  { setvalue(PRESOLVE_NONE) },
  { setvalue(PRESOLVE_ROWS) },
  { setvalue(PRESOLVE_COLS) },
  { setvalue(PRESOLVE_LINDEP) },
  { setvalue(PRESOLVE_AGGREGATE) },
  { setvalue(PRESOLVE_SPARSER) },
  { setvalue(PRESOLVE_SOS) },
  { setvalue(PRESOLVE_REDUCEMIP) },
  { setvalue(PRESOLVE_KNAPSACK) },
  { setvalue(PRESOLVE_ELIMEQ2) },
  { setvalue(PRESOLVE_IMPLIEDFREE) },
  { setvalue(PRESOLVE_REDUCEGCD) },
  { setvalue(PRESOLVE_PROBEFIX) },
  { setvalue(PRESOLVE_PROBEREDUCE) },
  { setvalue(PRESOLVE_ROWDOMINATE) },
  { setvalue(PRESOLVE_COLDOMINATE) },
  { setvalue(PRESOLVE_MERGEROWS) },
  { setvalue(PRESOLVE_IMPLIEDSLK) },
  { setvalue(PRESOLVE_COLFIXDUAL) },
  { setvalue(PRESOLVE_BOUNDS) },
  { setvalue(PRESOLVE_DUALS) },
  { setvalue(PRESOLVE_SENSDUALS) },
};

static char *STRLWR(char *str)
{
  char *ptr;

  for(ptr = str; *ptr; ptr++)
    *ptr = (char) tolower((unsigned char) *ptr);

  return(str);
}

static char *STRUPR(char *str)
{
  char *ptr;

  for(ptr = str; *ptr; ptr++)
    *ptr = (char) toupper((unsigned char) *ptr);

  return(str);
}

static void __WINAPI set_presolve1(lprec *lp, int do_presolve)
{
  set_presolve(lp, do_presolve, get_presolveloops(lp));
}

static void __WINAPI set_presolve2(lprec *lp, int maxloops)
{
  set_presolve(lp, get_presolve(lp), maxloops);
}

static struct _values print_sol[] =
{
  { FALSE, "0" },
  { TRUE,  "1" },
  { setvalue(AUTOMATIC) },
};

static struct _values scaling[] =
{
  { setvalue(SCALE_NONE) },
  { setvalue(SCALE_EXTREME) },
  { setvalue(SCALE_RANGE) },
  { setvalue(SCALE_MEAN) },
  { setvalue(SCALE_GEOMETRIC) },
  { setvalue(SCALE_CURTISREID) },
  { setvalue(SCALE_QUADRATIC) },
  { setvalue(SCALE_LOGARITHMIC) },
  { setvalue(SCALE_USERWEIGHT) },
  { setvalue(SCALE_POWER2) },
  { setvalue(SCALE_EQUILIBRATE) },
  { setvalue(SCALE_INTEGERS) },
  { setvalue(SCALE_DYNUPDATE) },
  { setvalue(SCALE_ROWSONLY) },
  { setvalue(SCALE_COLSONLY) },
};

static struct _values simplextype[] =
{
  { setvalue(SIMPLEX_PRIMAL_PRIMAL) },
  { setvalue(SIMPLEX_DUAL_PRIMAL) },
  { setvalue(SIMPLEX_PRIMAL_DUAL) },
  { setvalue(SIMPLEX_DUAL_DUAL) },
};

static struct _values verbose[] =
{
  { setvalue(NEUTRAL) },
  { setvalue(CRITICAL) },
  { setvalue(SEVERE) },
  { setvalue(IMPORTANT) },
  { setvalue(NORMAL) },
  { setvalue(DETAILED) },
  { setvalue(FULL) },
};

static struct _functions functions[] =
{
  /* solve options */
  { "ANTI_DEGEN", setintfunction(get_anti_degen, set_anti_degen), setvalues(anti_degen, ~0), WRITE_ACTIVE },
  { "BASISCRASH", setintfunction(get_basiscrash, set_basiscrash), setvalues(basiscrash, ~0), WRITE_ACTIVE },
  { "IMPROVE", setintfunction(get_improve, set_improve), setvalues(improve, ~0), WRITE_ACTIVE },
  { "MAXPIVOT", setintfunction(get_maxpivot, set_maxpivot), setNULLvalues, WRITE_ACTIVE },
  { "NEGRANGE", setREALfunction(get_negrange, set_negrange), setNULLvalues, WRITE_ACTIVE },
  { "PIVOTING", setintfunction(get_pivoting, set_pivoting), setvalues(pivoting, PRICER_LASTOPTION), WRITE_ACTIVE },
  { "PRESOLVE", setintfunction(get_presolve, set_presolve1), setvalues(presolving, ~0), WRITE_ACTIVE },
  { "PRESOLVELOOPS", setintfunction(get_presolveloops, set_presolve2), setNULLvalues, WRITE_ACTIVE },
  { "SCALELIMIT", setREALfunction(get_scalelimit, set_scalelimit), setNULLvalues, WRITE_ACTIVE },
  { "SCALING", setintfunction(get_scaling, set_scaling), setvalues(scaling, SCALE_CURTISREID), WRITE_ACTIVE },
  { "SIMPLEXTYPE", setintfunction(get_simplextype, set_simplextype), setvalues(simplextype, ~0), WRITE_ACTIVE },
  { "OBJ_IN_BASIS", setMYBOOLfunction(is_obj_in_basis, set_obj_in_basis), setNULLvalues, WRITE_COMMENTED },

  /* B&B options */
  { "BB_DEPTHLIMIT", setintfunction(get_bb_depthlimit, set_bb_depthlimit), setNULLvalues, WRITE_ACTIVE },
  { "BB_FLOORFIRST", setintfunction(get_bb_floorfirst, set_bb_floorfirst), setvalues(bb_floorfirst, ~0), WRITE_ACTIVE },
  { "BB_RULE", setintfunction(get_bb_rule, set_bb_rule), setvalues(bb_rule, NODE_STRATEGYMASK), WRITE_ACTIVE },
  { "BREAK_AT_FIRST", setMYBOOLfunction(is_break_at_first, set_break_at_first), setNULLvalues, WRITE_COMMENTED },
  { "BREAK_AT_VALUE", setREALfunction(get_break_at_value, set_break_at_value), setNULLvalues, WRITE_COMMENTED },
  { "MIP_GAP_ABS", setREALfunction(get_mip_gap_abs, set_mip_gap_abs), setNULLvalues, WRITE_ACTIVE },
  { "MIP_GAP_REL", setREALfunction(get_mip_gap_rel, set_mip_gap_rel), setNULLvalues, WRITE_ACTIVE },
  { "EPSINT", setREALfunction(get_epsint, set_epsint), setNULLvalues, WRITE_ACTIVE },

  /* tolerances, values */
  { "EPSB", setREALfunction(get_epsb, set_epsb), setNULLvalues, WRITE_ACTIVE },
  { "EPSD", setREALfunction(get_epsd, set_epsd), setNULLvalues, WRITE_ACTIVE },
  { "EPSEL", setREALfunction(get_epsel, set_epsel), setNULLvalues, WRITE_ACTIVE },
  { "EPSPERTURB", setREALfunction(get_epsperturb, set_epsperturb), setNULLvalues, WRITE_ACTIVE },
  { "EPSPIVOT", setREALfunction(get_epspivot, set_epspivot), setNULLvalues, WRITE_ACTIVE },
  { "INFINITE", setREALfunction(get_infinite, set_infinite), setNULLvalues, WRITE_ACTIVE },

  /* read-only options */
  { "DEBUG", setMYBOOLfunction(is_debug, set_debug), setNULLvalues, WRITE_COMMENTED },
  { "OBJ_BOUND", setREALfunction(get_obj_bound, set_obj_bound), setNULLvalues, WRITE_COMMENTED },
  { "PRINT_SOL", setintfunction(get_print_sol, set_print_sol), setvalues(print_sol, ~0), WRITE_COMMENTED },
  { "TIMEOUT", setlongfunction(get_timeout, set_timeout), setNULLvalues, WRITE_COMMENTED },
  { "TRACE", setMYBOOLfunction(is_trace, set_trace), setNULLvalues, WRITE_COMMENTED },
  { "VERBOSE", setintfunction(get_verbose, set_verbose), setvalues(verbose, ~0), WRITE_COMMENTED },
};

static void write_params1(lprec *lp, FILE *fp, char *header, int newline)
{
  int ret = 0, ret2, i, j, k, value, value2, elements, majorversion, minorversion, release, build;
  unsigned int basemask;
  REAL a = 0;
  char buf[4096], par[20];

  ini_writeheader(fp, header, newline);
  lp_solve_version(&majorversion, &minorversion, &release, &build);
  sprintf(buf, "lp_solve version %d.%d settings\n", majorversion, minorversion);
  ini_writecomment(fp, buf);
  for(i = 0; i < sizeof(functions) / sizeof(*functions); i++) {
    switch(functions[i].type) {
    case intfunction:
      if(functions[i].get_function.int_get_function == NULL)
        continue;
      ret = functions[i].get_function.int_get_function(lp);
      break;
    case longfunction:
      if(functions[i].get_function.long_get_function == NULL)
        continue;
      ret = functions[i].get_function.long_get_function(lp);
      break;
    case MYBOOLfunction:
      if(functions[i].get_function.MYBOOL_get_function == NULL)
        continue;
      ret = (int) functions[i].get_function.MYBOOL_get_function(lp);
      break;
    case REALfunction:
      if(functions[i].get_function.REAL_get_function == NULL)
        continue;
      a = functions[i].get_function.REAL_get_function(lp);
      break;
    }
    buf[0] = 0;
    if(functions[i].values == NULL) {
      switch(functions[i].type) {
      case intfunction:
      case longfunction:
      case MYBOOLfunction:
        sprintf(buf, "%d", ret);
        break;
      case REALfunction:
        sprintf(buf, "%g", a);
        break;
      }
    }
    else {
      elements = functions[i].elements;
      basemask = functions[i].basemask;
      for(j = 0; j < elements; j++) {
        value = functions[i].values[j].value;
	ret2 = ret;
	if(((unsigned int) value) < basemask)
	  ret2 &= basemask;
        if(value == 0) {
          if(ret2 == 0) {
            if(*buf)
              strcat(buf, " + ");
            strcat(buf, functions[i].values[j].svalue);
          }
        }
        else if((ret2 & value) == value) {
          for(k = 0; k < elements; k++) {
            value2 = functions[i].values[k].value;
            if((k != j) && (value2 > value) && ((value2 & value) == value) && ((ret2 & value2) == value2))
              break;
          }
          if(k == elements) {
            if(*buf)
              strcat(buf, " + ");
            strcat(buf, functions[i].values[j].svalue);
          }
        }
      }
    }
    if(functions[i].mask & WRITE_ACTIVE)
      par[0] = 0;
    else
      strcpy(par, ";");
    strcat(par, functions[i].par);
    ini_writedata(fp, STRLWR(par), buf);
  }
}

static void readoptions(char *options, char **header)
{
  char *ptr1, *ptr2;

  if(options != NULL) {
    ptr1 = options;
    while(*ptr1) {
      ptr2 = strchr(ptr1, '-');
      if(ptr2 == NULL)
        break;
      ptr2++;
      if(tolower((unsigned char) *ptr2) == 'h') {
        for(++ptr2; (*ptr2) && (isspace(*ptr2)); ptr2++);
        for(ptr1 = ptr2; (*ptr1) && (!isspace(*ptr1)); ptr1++);
        *header = (char *) calloc(1 + (int) (ptr1 - ptr2), 1);
        memcpy(*header, ptr2, (int) (ptr1 - ptr2));
      }
    }
  }

  if(*header == NULL)
    *header = strdup("Default");
}

MYBOOL __WINAPI write_params(lprec *lp, char *filename, char *options)
{
  int k, ret, params_written;
  FILE *fp, *fp0;
  int state = 0, looping, newline;
  char buf[4096], *filename0, *ptr1, *ptr2, *header = NULL;

  readoptions(options, &header);

  k = strlen(filename);
  filename0 = (char *) malloc(k + 1 + 1);
  strcpy(filename0, filename);
  ptr1 = strrchr(filename0, '.');
  ptr2 = strrchr(filename0, '\\');
  if((ptr1 == NULL) || ((ptr2 != NULL) && (ptr1 < ptr2)))
    ptr1 = filename0 + k;
  memmove(ptr1 + 1, ptr1, k + 1 - (int) (ptr1 - filename0));
  ptr1[0] = '_';
  if(rename(filename, filename0)) {
    switch(errno) {
    case ENOENT: /* File or path specified by oldname not found */
      FREE(filename0);
      filename0 = NULL;
      break;
    case EACCES: /* File or directory specified by newname already exists or could not be created (invalid path); or oldname is a directory and newname specifies a different path. */
      FREE(filename0);
      FREE(header);
      return(FALSE);
      break;
    }
  }

  if((fp = ini_create(filename)) == NULL)
    ret = FALSE;
  else {
    params_written = FALSE;
    newline = TRUE;
    if(filename0 != NULL) {
      fp0 = ini_open(filename0);
      if(fp0 == NULL) {
        rename(filename0, filename);
        FREE(filename0);
        FREE(header);
        return(FALSE);
      }
      looping = TRUE;
      while(looping) {
        switch(ini_readdata(fp0, buf, sizeof(buf), TRUE)) {
        case 0: /* End of file */
          looping = FALSE;
          break;
        case 1: /* header */
          ptr1 = strdup(buf);
          STRUPR(buf);
          ptr2 = strdup(header);
          STRUPR(ptr2);
          if(strcmp(buf, ptr2) == 0) {
            write_params1(lp, fp, ptr1, newline);
            params_written = TRUE;
            newline = TRUE;
            state = 1;
          }
          else {
            state = 0;
            ini_writeheader(fp, ptr1, newline);
            newline = TRUE;
          }
          FREE(ptr2);
          FREE(ptr1);
          break;
        case 2: /* data */
          if(state == 0) {
            ini_writedata(fp, NULL, buf);
            newline = (*buf != 0);
          }
          break;
        }
      }
      ini_close(fp0);
    }

    if(!params_written)
      write_params1(lp, fp, header, newline);

    ini_close(fp);
    ret = TRUE;
  }

  if(filename0 != NULL) {
    remove(filename0);
    FREE(filename0);
  }

  FREE(header);

  return( (MYBOOL) ret );
}


MYBOOL __WINAPI read_params(lprec *lp, char *filename, char *options)
{
  int ret, looping, line;
  FILE *fp;
  hashtable *hashfunctions, *hashparameters;
  hashelem *hp;
  int i, j, elements, n, intvalue, state = 0;
  REAL REALvalue;
  char buf[4096], *header = NULL, *ptr, *ptr1, *ptr2;

  if((fp = ini_open(filename)) == NULL)
    ret = FALSE;
  else {
    /* create hashtable of all callable commands to find them quickly */
    hashfunctions = create_hash_table(sizeof(functions) / sizeof(*functions), 0);
    for (n = 0, i = 0; i < (int) (sizeof(functions)/sizeof(*functions)); i++) {
      puthash(functions[i].par, i, NULL, hashfunctions);
      if(functions[i].values != NULL)
        n += functions[i].elements;
    }
    /* create hashtable of all arguments to find them quickly */
    hashparameters = create_hash_table(n, 0);
    for (n = 0, i = 0; i < (int) (sizeof(functions)/sizeof(*functions)); i++) {
      if(functions[i].values != NULL) {
        elements = functions[i].elements;
        for(j = 0; j < elements; j++)
          if((strcmp(functions[i].values[j].svalue, "0") != 0) &&
             (strcmp(functions[i].values[j].svalue, "1") != 0))
            puthash(functions[i].values[j].svalue, j, NULL, hashparameters);
      }
    }

    readoptions(options, &header);

    STRUPR(header);
    ret = looping = TRUE;
    line = 0;
    while((ret) && (looping)) {
      line++;
      switch(ini_readdata(fp, buf, sizeof(buf), FALSE)) {
        case 0: /* End of file */
          looping = FALSE;
          break;
        case 1: /* header */
          switch(state) {
            case 0:
              STRUPR(buf);
              if(strcmp(buf, header) == 0)
                state = 1;
              break;
            case 1:
              looping = FALSE;
              break;
          }
          break;
        case 2: /* data */
          if(state == 1) {
            for(ptr = buf; (*ptr) && (isspace(*ptr)); ptr++);
          }
          else
            ptr = NULL;
          if((ptr != NULL) && (*ptr)) {
            STRUPR(buf);
            ptr = strchr(buf, '=');
            if(ptr == NULL) {
              report(lp, IMPORTANT, "read_params: No equal sign on line %d\n", line);
              ret = FALSE;
            }
            else {
              *ptr = 0;
              for(ptr1 = buf; isspace(*ptr1); ptr1++);
              for(ptr2 = ptr - 1; (ptr2 >= ptr1) && (isspace(*ptr2)); ptr2--);
              if(ptr2 <= ptr1) {
                report(lp, IMPORTANT, "read_params: No parameter name before equal sign on line %d\n", line);
                ret = FALSE;
              }
              else {
                ptr2[1] = 0;
                hp = findhash(ptr1, hashfunctions);
                if(hp == NULL) {
                  report(lp, IMPORTANT, "read_params: Unknown parameter name (%s) before equal sign on line %d\n", ptr1, line);
                  ret = FALSE;
                }
                else {
                  i = hp->index;
                  ptr1 = ++ptr;
                  intvalue = 0;
                  REALvalue = 0;
                  if(functions[i].values == NULL) {
                    switch(functions[i].type) {
                      case intfunction:
                      case longfunction:
                      case MYBOOLfunction:
                        intvalue = strtol(ptr1, &ptr2, 10);
                        while((*ptr2) && (isspace(*ptr2)))
                          ptr2++;
                        if(*ptr2) {
                          report(lp, IMPORTANT, "read_params: Invalid integer value on line %d\n", line);
                          ret = FALSE;
                        }
                        break;
                      case REALfunction:
                        REALvalue = strtod(ptr1, &ptr2);
                        while((*ptr2) && (isspace(*ptr2)))
                          ptr2++;
                        if(*ptr2) {
                          report(lp, IMPORTANT, "read_params: Invalid real value on line %d\n", line);
                          ret = FALSE;
                        }
                        break;
                    }
                  }
                  else {
                    while(ret) {
                      ptr = strchr(ptr1, '+');
                      if(ptr == NULL)
                        ptr = ptr1 + strlen(ptr1);
                      for(; isspace(*ptr1); ptr1++);
                      for(ptr2 = ptr - 1; (ptr2 >= ptr1) && (isspace(*ptr2)); ptr2--);
                      if(ptr2 <= ptr1)
                        break;
                      else {
                        ptr2[1] = 0;
                        hp = findhash(ptr1, hashparameters);
                        if (hp == NULL) {
                          report(lp, IMPORTANT, "read_params: Invalid parameter name (%s) on line %d\n", ptr1, line);
                          ret = FALSE;
                        }
                        else {
                          j = hp->index;
                          if((j >= functions[i].elements) ||
                             (strcmp(functions[i].values[j].svalue, ptr1))) {
                            report(lp, IMPORTANT, "read_params: Inappropriate parameter name (%s) on line %d\n", ptr1, line);
                            ret = FALSE;
                          }
                          else {
                            intvalue += functions[i].values[j].value;
                          }
                        }
                        ptr1 = ptr + 1;
                      }
                    }
                  }
                  if(ret) {
                    switch(functions[i].type) {
                      case intfunction:
                        functions[i].set_function.int_set_function(lp, intvalue);
                        break;
                      case longfunction:
                        functions[i].set_function.long_set_function(lp, intvalue);
                        break;
                      case MYBOOLfunction:
                        functions[i].set_function.MYBOOL_set_function(lp, (MYBOOL) intvalue);
                        break;
                      case REALfunction:
                        functions[i].set_function.REAL_set_function(lp, REALvalue);
                        break;
                    }
                  }
                }
              }
            }
          }
          break;
      }
    }

    FREE(header);
    free_hash_table(hashfunctions);
    free_hash_table(hashparameters);

    ini_close(fp);
  }

  return( (MYBOOL) ret );
}
