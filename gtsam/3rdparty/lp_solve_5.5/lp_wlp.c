
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "lp_lib.h"
#include "lp_scale.h"
#include "lp_utils.h"
#include "lp_report.h"
#include "lp_wlp.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

/* Define buffer-size controled function mapping */
# if defined _MSC_VER
#  define vsnprintf _vsnprintf
# endif

/* ------------------------------------------------------------------------- */
/* Input and output of lp format model files for lp_solve                    */
/* ------------------------------------------------------------------------- */

static int write_data(void *userhandle, write_modeldata_func write_modeldata, char *format, ...)
{
  char buff[DEF_STRBUFSIZE+1];
  va_list ap;
  int n;

  va_start(ap, format);
  vsnprintf(buff, DEF_STRBUFSIZE, format, ap);
  n = write_modeldata(userhandle, buff);
  va_end(ap);
  return(n);
}

STATIC void write_lpcomment(void *userhandle, write_modeldata_func write_modeldata, char *string, MYBOOL newlinebefore)
{
  write_data(userhandle, write_modeldata, "%s/* %s */\n", (newlinebefore) ? "\n" : "", string);
}

STATIC int write_lprow(lprec *lp, int rowno, void *userhandle, write_modeldata_func write_modeldata, int maxlen)
{
  int     i, ie, j, nchars;
  REAL    a;
  MATrec  *mat = lp->matA;
  MYBOOL  first = TRUE, elements;

  if(rowno == 0) {
    i = 1;
    ie = lp->columns+1;
  }
  else {
    i = mat->row_end[rowno-1];
    ie = mat->row_end[rowno];
  }
  elements = ie - i;
  if(write_modeldata != NULL) {
    nchars = 0;
    for(; i < ie; i++) {
      if(rowno == 0) {
	j = i;
	a = get_mat(lp, 0, i);
	if(a == 0)
	  continue;
      }
      else {
	j = ROW_MAT_COLNR(i);
	a = ROW_MAT_VALUE(i);
	a = my_chsign(is_chsign(lp, rowno), a);
	a = unscaled_mat(lp, a, rowno, j);
      }
      if(is_splitvar(lp, j))
	continue;
      if(!first)
	nchars += write_data(userhandle, write_modeldata, " ");
      else
	first = FALSE;
      if(a == -1)
	nchars += write_data(userhandle, write_modeldata, "-");
      else if(a == 1)
	nchars += write_data(userhandle, write_modeldata, "+");
      else
	nchars += write_data(userhandle, write_modeldata, "%+.12g ", (double)a);
      nchars += write_data(userhandle, write_modeldata, "%s", get_col_name(lp, j));
      /* Check if we should add a linefeed */
      if((maxlen > 0) && (nchars >= maxlen) && (i < ie-1)) {
	write_data(userhandle, write_modeldata, "%s", "\n");
	nchars = 0;
      }
    }
  }
  return(elements);
}

#if !defined LP_MAXLINELEN
# define LP_MAXLINELEN 100
#endif

MYBOOL __WINAPI write_lpex(lprec *lp, void *userhandle, write_modeldata_func write_modeldata)
{
  int    i, j, b,
         nrows = lp->rows,
         ncols = lp->columns,
         nchars, maxlen = LP_MAXLINELEN;
  MYBOOL ok;
  REAL   a;
  char   *ptr;

  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "LP_writefile: Cannot write to LP file while in row entry mode.\n");
    return(FALSE);
  }
  if(!mat_validate(lp->matA)) {
    report(lp, IMPORTANT, "LP_writefile: Could not validate the data matrix.\n");
    return(FALSE);
  }

  /* Write name of model */
  ptr = get_lp_name(lp);
  if(ptr != NULL)
    if(*ptr)
      write_lpcomment(userhandle, write_modeldata, ptr, FALSE);
    else
      ptr = NULL;

  /* Write the objective function */
  write_lpcomment(userhandle, write_modeldata, "Objective function", (MYBOOL) (ptr != NULL));
  if(is_maxim(lp))
    write_data(userhandle, write_modeldata, "max: ");
  else
    write_data(userhandle, write_modeldata, "min: ");

  write_lprow(lp, 0, userhandle, write_modeldata, maxlen);
  a = get_rh(lp, 0);
  if(a != 0)
    write_data(userhandle, write_modeldata, " %+.12g", a);
  write_data(userhandle, write_modeldata, ";\n");

  /* Write constraints */
  if(nrows > 0)
    write_lpcomment(userhandle, write_modeldata, "Constraints", TRUE);
  for(j = 1; j <= nrows; j++) {
    if(((lp->names_used) && (lp->row_name[j] != NULL)) || (write_lprow(lp, j, userhandle, NULL, maxlen) == 1))
      ptr = get_row_name(lp, j);
    else
      ptr = NULL;
    if((ptr != NULL) && (*ptr))
      write_data(userhandle, write_modeldata, "%s: ", ptr);

#ifndef SingleBoundedRowInLP
    /* Write the ranged part of the constraint, if specified */
    if ((lp->orig_upbo[j]) && (lp->orig_upbo[j] < lp->infinite)) {
      if(my_chsign(is_chsign(lp, j), lp->orig_rhs[j]) == -lp->infinite)
        write_data(userhandle, write_modeldata, "-Inf %s ", (is_chsign(lp, j)) ? ">=" : "<=");
      else if(my_chsign(is_chsign(lp, j), lp->orig_rhs[j]) == lp->infinite)
        write_data(userhandle, write_modeldata, "+Inf %s ", (is_chsign(lp, j)) ? ">=" : "<=");
      else
        write_data(userhandle, write_modeldata, "%+.12g %s ",
                (lp->orig_upbo[j]-lp->orig_rhs[j]) * (is_chsign(lp, j) ? 1.0 : -1.0) / (lp->scaling_used ? lp->scalars[j] : 1.0),
                (is_chsign(lp, j)) ? ">=" : "<=");
    }
#endif

    if((!write_lprow(lp, j, userhandle, write_modeldata, maxlen)) && (ncols >= 1))
      write_data(userhandle, write_modeldata, "0 %s", get_col_name(lp, 1));

    if(lp->orig_upbo[j] == 0)
      write_data(userhandle, write_modeldata, " =");
    else if(is_chsign(lp, j))
      write_data(userhandle, write_modeldata, " >=");
    else
      write_data(userhandle, write_modeldata, " <=");
    if(fabs(get_rh(lp, j) + lp->infinite) < 1)
      write_data(userhandle, write_modeldata, " -Inf;\n");
    else if(fabs(get_rh(lp, j) - lp->infinite) < 1)
      write_data(userhandle, write_modeldata, " +Inf;\n");
    else
      write_data(userhandle, write_modeldata, " %.12g;\n", get_rh(lp, j));

#ifdef SingleBoundedRowInLP
    /* Write the ranged part of the constraint, if specified */
    if ((lp->orig_upbo[j]) && (lp->orig_upbo[j] < lp->infinite)) {
      if(((lp->names_used) && (lp->row_name[j] != NULL)) || (write_lprow(lp, j, userhandle, NULL, maxlen) == 1))
        ptr = get_row_name(lp, j);
      else
        ptr = NULL;
      if((ptr != NULL) && (*ptr))
        write_data(userhandle, write_modeldata, "%s: ", ptr);
      if((!write_lprow(lp, j, userhandle, write_modeldata, maxlen)) && (get_Ncolumns(lp) >= 1))
        write_data(userhandle, write_modeldata, "0 %s", get_col_name(lp, 1));
      write_data(userhandle, write_modeldata, " %s %g;\n",
                     (is_chsign(lp, j)) ? "<=" : ">=",
                     (lp->orig_upbo[j]-lp->orig_rhs[j]) * (is_chsign(lp, j) ? 1.0 : -1.0) / (lp->scaling_used ? lp->scalars[j] : 1.0));
    }
#endif
  }

  /* Write bounds on variables */
  ok = FALSE;
  for(i = nrows + 1; i <= lp->sum; i++)
    if(!is_splitvar(lp, i - nrows)) {
      if(lp->orig_lowbo[i] == lp->orig_upbo[i]) {
        if(!ok) {
	  write_lpcomment(userhandle, write_modeldata, "Variable bounds", TRUE);
	  ok = TRUE;
	}
        write_data(userhandle, write_modeldata, "%s = %.12g;\n", get_col_name(lp, i - nrows), get_upbo(lp, i - nrows));
      }
      else {
#ifndef SingleBoundedRowInLP
        if((lp->orig_lowbo[i] != 0) && (lp->orig_upbo[i] < lp->infinite)) {
          if(!ok) {
	    write_lpcomment(userhandle, write_modeldata, "Variable bounds", TRUE);
	    ok = TRUE;
	  }
          if(lp->orig_lowbo[i] == -lp->infinite)
            write_data(userhandle, write_modeldata, "-Inf");
          else
            write_data(userhandle, write_modeldata, "%.12g", get_lowbo(lp, i - nrows));
          write_data(userhandle, write_modeldata, " <= %s <= ", get_col_name(lp, i - nrows));
          if(lp->orig_lowbo[i] == lp->infinite)
            write_data(userhandle, write_modeldata, "+Inf");
          else
            write_data(userhandle, write_modeldata, "%.12g", get_upbo(lp, i - nrows));
          write_data(userhandle, write_modeldata, ";\n");
	}
        else
#endif
        {
          if(lp->orig_lowbo[i] != 0) {
            if(!ok) {
	      write_lpcomment(userhandle, write_modeldata, "Variable bounds", TRUE);
	      ok = TRUE;
	    }
      	    if(lp->orig_lowbo[i] == -lp->infinite)
	      write_data(userhandle, write_modeldata, "%s >= -Inf;\n", get_col_name(lp, i - nrows));
      	    else if(lp->orig_lowbo[i] == lp->infinite)
	      write_data(userhandle, write_modeldata, "%s >= +Inf;\n", get_col_name(lp, i - nrows));
	    else
              write_data(userhandle, write_modeldata, "%s >= %.12g;\n",
                              get_col_name(lp, i - nrows), get_lowbo(lp, i - nrows));
	  }
	  if(lp->orig_upbo[i] != lp->infinite) {
            if(!ok) {
	      write_lpcomment(userhandle, write_modeldata, "Variable bounds", TRUE);
	      ok = TRUE;
	    }
            write_data(userhandle, write_modeldata, "%s <= %.12g;\n",
                            get_col_name(lp, i - nrows), get_upbo(lp, i - nrows));
	  }
        }
      }
    }

  /* Write optional integer section */
  if(lp->int_vars > 0) {
    write_lpcomment(userhandle, write_modeldata, "Integer definitions", TRUE);
    i = 1;
    while((i <= ncols) && !is_int(lp, i))
      i++;
    if(i <= ncols) {
      nchars = write_data(userhandle, write_modeldata, "int %s", get_col_name(lp, i));
      i++;
      for(; i <= ncols; i++)
        if((!is_splitvar(lp, i)) && (is_int(lp, i))) {
          if((maxlen!= 0) && (nchars > maxlen)) {
            write_data(userhandle, write_modeldata, "%s", "\n");
            nchars = 0;
          }
          write_data(userhandle, write_modeldata, ",%s", get_col_name(lp, i));
        }
      write_data(userhandle, write_modeldata, ";\n");
    }
  }

  /* Write optional SEC section */
  if(lp->sc_vars > 0) {
    write_lpcomment(userhandle, write_modeldata, "Semi-continuous variables", TRUE);
    i = 1;
    while((i <= ncols) && !is_semicont(lp, i))
      i++;
    if(i <= ncols) {
      nchars = write_data(userhandle, write_modeldata, "sec %s", get_col_name(lp, i));
      i++;
      for(; i <= ncols; i++)
        if((!is_splitvar(lp, i)) && (is_semicont(lp, i))) {
          if((maxlen != 0) && (nchars > maxlen)) {
            write_data(userhandle, write_modeldata, "%s", "\n");
            nchars = 0;
          }
          nchars += write_data(userhandle, write_modeldata, ",%s", get_col_name(lp, i));
        }
      write_data(userhandle, write_modeldata, ";\n");
    }
  }

  /* Write optional SOS section */
  if(SOS_count(lp) > 0) {
    SOSgroup *SOS = lp->SOS;
    write_lpcomment(userhandle, write_modeldata, "SOS definitions", TRUE);
    write_data(userhandle, write_modeldata, "SOS\n");
    for(b = 0, i = 0; i < SOS->sos_count; b = SOS->sos_list[i]->priority, i++) {
      nchars = write_data(userhandle, write_modeldata, "%s: ",
              (SOS->sos_list[i]->name == NULL) ||
              (*SOS->sos_list[i]->name==0) ? "SOS" : SOS->sos_list[i]->name); /* formatnumber12((double) lp->sos_list[i]->priority) */

      for(a = 0.0, j = 1; j <= SOS->sos_list[i]->size; a = SOS->sos_list[i]->weights[j], j++) {
        if((maxlen != 0) && (nchars > maxlen)) {
          write_data(userhandle, write_modeldata, "%s", "\n");
          nchars = 0;
        }
        if(SOS->sos_list[i]->weights[j] == ++a)
          nchars += write_data(userhandle, write_modeldata, "%s%s",
                  (j > 1) ? "," : "",
                  get_col_name(lp, SOS->sos_list[i]->members[j]));
        else
          nchars += write_data(userhandle, write_modeldata, "%s%s:%.12g",
                  (j > 1) ? "," : "",
                  get_col_name(lp, SOS->sos_list[i]->members[j]),
		  SOS->sos_list[i]->weights[j]);
      }
      if(SOS->sos_list[i]->priority == ++b)
        nchars += write_data(userhandle, write_modeldata, " <= %d;\n", SOS->sos_list[i]->type);
      else
        nchars += write_data(userhandle, write_modeldata, " <= %d:%d;\n", SOS->sos_list[i]->type, SOS->sos_list[i]->priority);
    }
  }

  ok = TRUE;

  return(ok);
}

static int __WINAPI write_lpdata(void *userhandle, char *buf)
{
  return(fprintf((FILE *) userhandle, "%s", buf));
}

MYBOOL LP_writefile(lprec *lp, char *filename)
{
  FILE *output = stdout;
  MYBOOL ok;

  if (filename != NULL) {
    ok = (MYBOOL) ((output = fopen(filename, "w")) != NULL);
    if(!ok)
      return(ok);
  }
  else
    output = lp->outstream;

  ok = write_lpex(lp, (void *) output, write_lpdata);

  if (filename != NULL)
    fclose(output);

  return(ok);
}

MYBOOL LP_writehandle(lprec *lp, FILE *output)
{
  MYBOOL ok;

  if (output != NULL)
    set_outputstream(lp, output);

  output = lp->outstream;

  ok = write_lpex(lp, (void *) output, write_lpdata);

  return(ok);
}
