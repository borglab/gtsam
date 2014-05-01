
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_scale.h"
#include "lp_report.h"
#include "lp_MPS.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

/* Define buffer-size controled function mapping */
# if defined _MSC_VER
#  define vsnprintf _vsnprintf
# endif

/* MPS file input and output routines for lp_solve                           */
/* ------------------------------------------------------------------------- */

/*
A:  MPS format was named after an early IBM LP product and has emerged
as a de facto standard ASCII medium among most of the commercial LP
codes.  Essentially all commercial LP codes accept this format, but if
you are using public domain software and have MPS files, you may need
to write your own reader routine for this.  It's not too hard.  The
main things to know about MPS format are that it is column oriented (as
opposed to entering the model as equations), and everything (variables,
rows, etc.) gets a name.  MPS format is described in more detail in
Murtagh's book, referenced in another section. Also,

ftp://softlib.cs.rice.edu/pub/miplib/mps_format

is a nice short introduction.  exports

MPS is an old format, so it is set up as though you were using punch
cards, and is not free format. Fields start in column 1, 5, 15, 25, 40
and 50.  Sections of an MPS file are marked by so-called header cards,
which are distinguished by their starting in column 1.  Although it is
typical to use upper-case throughout the file (like I said, MPS has
long historical roots), many MPS-readers will accept mixed-case for
anything except the header cards, and some allow mixed-case anywhere.
The names that you choose for the individual entities (constraints or
variables) are not important to the solver; you should pick names that
are meaningful to you, or will be easy for a post-processing code to
read.

Here is a little sample model written in MPS format (explained in more
detail below):

NAME          TESTPROB
ROWS
 N  COST
 L  LIM1
 G  LIM2
 E  MYEQN
COLUMNS
    XONE      COST                 1   LIM1                 1
    XONE      LIM2                 1
    YTWO      COST                 4   LIM1                 1
    YTWO      MYEQN               -1
    ZTHREE    COST                 9   LIM2                 1
    ZTHREE    MYEQN                1
RHS
    RHS1      LIM1                 5   LIM2                10
    RHS1      MYEQN                7
BOUNDS
 UP BND1      XONE                 4
 LO BND1      YTWO                -1
 UP BND1      YTWO                 1
ENDATA

means:

Optimize
 COST:    XONE + 4 YTWO + 9 ZTHREE
Subject To
 LIM1:    XONE + YTWO <= 5
 LIM2:    XONE + ZTHREE >= 10
 MYEQN:   - YTWO + ZTHREE  = 7
Bounds
 0 <= XONE <= 4
-1 <= YTWO <= 1
End

*/

/* copy a MPS name, only trailing spaces are removed. In MPS, names can have
   embedded spaces! */
STATIC void namecpy(char *into, char *from)
{
  int i;

  /* copy at most 8 characters of from, stop at end of string or newline */
  for(i = 0; (from[i] != '\0') && (from[i] != '\n') && (from[i] != '\r') && (i < 8); i++)
    into[i] = from[i];

  /* end with end of string */
  into[i] = '\0';

  /* remove trailing spaces, if any */
  for(i--; (i >= 0) && (into[i] == ' '); i--)
    into[i] = '\0';
}

/* scan an MPS line, and pick up the information in the fields that are
   present */

/* scan_line for fixed MPS format */
STATIC int scan_lineFIXED(lprec *lp, int section, char* line, char *field1, char *field2, char *field3,
                          double *field4, char *field5, double *field6)
{
  int  items = 0, line_len;
  char buf[16], *ptr1, *ptr2;

  line_len = (int) strlen(line);
  while ((line_len) && ((line[line_len-1] == '\n') || (line[line_len-1] == '\r') || (line[line_len-1] == ' ')))
   line_len--;

  if(line_len >= 1) { /* spaces or N/L/G/E or UP/LO */
    strncpy(buf, line, 4);
    buf[4] = '\0';
    sscanf(buf, "%s", field1);
    items++;
  }
  else
    field1[0] = '\0';

  line += 4;

  if(line_len >= 5) { /* name */
    if (line[-1] != ' ') {
      report(lp, IMPORTANT, "MPS_readfile: invalid data card; column 4 must be blank\n");
      return(-1);
    }
    namecpy(field2, line);
    items++;
  }
  else
    field2[0] = '\0';

  line += 10;

  if(line_len >= 14) { /* name */
    if (line[-1] != ' ' || line[-2] != ' ') {
      report(lp, IMPORTANT, "MPS_readfile: invalid data card; columns 13-14 must be blank\n");
      return(-1);
    }
    namecpy(field3, line);
    items++;
  }
  else
    field3[0] = '\0';

  line += 10;

  if(line_len >= 25) { /* number */
    if (line[-1] != ' ' || line[-2] != ' ') {
      report(lp, IMPORTANT, "MPS_readfile: invalid data card; columns 23-24 must be blank\n");
      return(-1);
    }
    strncpy(buf, line, 15);
    buf[15] = '\0';
    for(ptr1 = ptr2 = buf; ; ptr1++)
      if(!isspace((unsigned char) *ptr1))
        if((*(ptr2++) = *ptr1) == 0)
          break;
    /* *field4 = atof(buf); */
    *field4 = strtod(buf, &ptr1);
    if(*ptr1) {
      report(lp, IMPORTANT, "MPS_readfile: invalid number in columns 25-36 \n");
      return(-1);
    }
    items++;
  }
  else
    *field4 = 0;

  line += 15;

  if(line_len >= 40) { /* name */
    if (line[-1] != ' ' || line[-2] != ' ' || line[-3] != ' ') {
      report(lp, IMPORTANT, "MPS_readfile: invalid data card; columns 37-39 must be blank\n");
      return(-1);
    }
    namecpy(field5, line);
    items++;
  }
  else
    field5[0] = '\0';
  line += 10;

  if(line_len >= 50) { /* number */
    if (line[-1] != ' ' || line[-2] != ' ') {
      report(lp, IMPORTANT, "MPS_readfile: invalid data card; columns 48-49 must be blank\n");
      return(-1);
    }
    strncpy(buf, line, 15);
    buf[15] = '\0';
    for(ptr1 = ptr2 = buf; ; ptr1++)
      if(!isspace((unsigned char) *ptr1))
        if((*(ptr2++) = *ptr1) == 0)
          break;
    /* *field6 = atof(buf); */
    *field6 = strtod(buf, &ptr1);
    if(*ptr1) {
      report(lp, IMPORTANT, "MPS_readfile: invalid number in columns 50-61 \n");
      return(-1);
    }
    items++;
  }
  else
    *field6 = 0;

  return(items);
}

STATIC int spaces(char *line, int line_len)
{
  int l;
  char *line1 = line;

  while (*line1 == ' ')
    line1++;
  l = (int) (line1 - line);
  if (line_len < l)
    l = line_len;
  return(l);
}

STATIC int lenfield(char *line, int line_len)
{
  int l;
  char *line1 = line;

  while ((*line1) && (*line1 != ' '))
    line1++;
  l = (int) (line1 - line);
  if (line_len < l)
    l = line_len;
  return(l);
}

/* scan_line for fixed MPS format */
STATIC int scan_lineFREE(lprec *lp, int section, char* line, char *field1, char *field2, char *field3,
                         double *field4, char *field5, double *field6)
{
  int  items = 0, line_len, len;
  char buf[256], *ptr1, *ptr2;

  line_len = (int) strlen(line);
  while ((line_len) && ((line[line_len-1] == '\n') || (line[line_len-1] == '\r') || (line[line_len-1] == ' ')))
   line_len--;

  len = spaces(line, line_len);
  line += len;
  line_len -= len;

  if ((section == MPSCOLUMNS) || (section == MPSRHS) || (section == MPSRANGES)) {
    field1[0] = '\0';
    items++;
  }
  else {
    len = lenfield(line, line_len);
    if(line_len >= 1) { /* spaces or N/L/G/E or UP/LO */
      strncpy(buf, line, len);
      buf[len] = '\0';
      sscanf(buf, "%s", field1);
      if(section == MPSBOUNDS) {
        for(ptr1 = field1; *ptr1; ptr1++)
          *ptr1=(char)toupper(*ptr1);
      }
      items++;
    }
    else
      field1[0] = '\0';

    line += len;
    line_len -= len;

    len = spaces(line, line_len);
    line += len;
    line_len -= len;
  }

  len = lenfield(line, line_len);
  if(line_len >= 1) { /* name */
    strncpy(field2, line, len);
    field2[len] = '\0';
    items++;
  }
  else
    field2[0] = '\0';

  line += len;
  line_len -= len;

  len = spaces(line, line_len);
  line += len;
  line_len -= len;

  len = lenfield(line, line_len);
  if(line_len >= 1) { /* name */
    strncpy(field3, line, len);
    field3[len] = '\0';
    items++;
  }
  else
    field3[0] = '\0';

  line += len;
  line_len -= len;

  len = spaces(line, line_len);
  line += len;
  line_len -= len;

  if (*field3) {
    if((section == MPSCOLUMNS) && (strcmp(field3, "'MARKER'") == 0)) {
      *field4 = 0;
      items++;
      ptr1 = field3;
    }
    else if((section == MPSBOUNDS) &&
            ((strcmp(field1, "FR") == 0) || (strcmp(field1, "MI") == 0) || (strcmp(field1, "PL") == 0) || (strcmp(field1, "BV") == 0)))
      /* field3 *is* the variable name */;
    else {
      /* Some free MPS formats allow that field 2 is not provided after the first time.
         The fieldname is then the same as the the defined field the line before.
         In that case field2 shifts to field3, field1 shifts to field 2.
         This situation is tested by checking if field3 is numerical AND there are an even number of fields after.
      */
      char *line1 = line;
      int line_len1 = line_len;
      int items1 = 0;

      while (line_len1 > 0) {
        len = lenfield(line1, line_len1);
        if (len > 0) {
          line1 += len;
          line_len1 -= len;
          items1++;
        }
        len = spaces(line1, line_len1);
        line1 += len;
        line_len1 -= len;
      }
      if ((items1 % 2) == 0) {
        *field4 = strtod(field3, &ptr1);
        if(*ptr1 == 0) {
          strcpy(field3, field2);
          if ((section == MPSROWS) || (section == MPSBOUNDS) /* || (section == MPSSOS) */)
            *field2 = 0;
          else {
            strcpy(field2, field1);
            *field1 = 0;
          }
          items++;
        }
        else
          ptr1 = NULL;
      }
      else
        ptr1 = NULL;
    }
  }
  else {
    ptr1 = NULL;
    if((section == MPSBOUNDS) &&
       ((strcmp(field1, "FR") == 0) || (strcmp(field1, "MI") == 0) || (strcmp(field1, "PL") == 0) || (strcmp(field1, "BV") == 0))) {
      strcpy(field3, field2);
      *field2 = 0;
      items++;
    }
  }

  if(ptr1 == NULL) {
    len = lenfield(line, line_len);
    if(line_len >= 1) { /* number */
      strncpy(buf, line, len);
      buf[len] = '\0';
      for(ptr1 = ptr2 = buf; ; ptr1++)
        if(!isspace((unsigned char) *ptr1))
          if((*(ptr2++) = *ptr1) == 0)
            break;
      /* *field4 = atof(buf); */
      *field4 = strtod(buf, &ptr1);
      if(*ptr1)
        return(-1);
      items++;
    }
    else
      *field4 = 0;

    line += len;
    line_len -= len;

    len = spaces(line, line_len);
    line += len;
    line_len -= len;
  }

  len = lenfield(line, line_len);
  if(line_len >= 1) { /* name */
    strncpy(field5, line, len);
    field5[len] = '\0';
    items++;
  }
  else
    field5[0] = '\0';
  line += len;
  line_len -= len;

  len = spaces(line, line_len);
  line += len;
  line_len -= len;

  len = lenfield(line, line_len);
  if(line_len >= 1) { /* number */
    strncpy(buf, line, len);
    buf[len] = '\0';
    for(ptr1 = ptr2 = buf; ; ptr1++)
      if(!isspace((unsigned char) *ptr1))
        if((*(ptr2++) = *ptr1) == 0)
          break;
    /* *field6 = atof(buf); */
    *field6 = strtod(buf, &ptr1);
    if(*ptr1)
      return(-1);
    items++;
  }
  else
    *field6 = 0;

  if((section == MPSSOS) && (items == 2)) {
    strcpy(field3, field2);
    strcpy(field2, field1);
    *field1 = 0;
  }

  if((section != MPSOBJNAME) && (section != MPSBOUNDS)) {
    for(ptr1 = field1; *ptr1; ptr1++)
      *ptr1=(char)toupper(*ptr1);
  }

  return(items);
}

STATIC int addmpscolumn(lprec *lp, MYBOOL Int_section, MYBOOL *Column_ready,
                        int *count, REAL *Last_column, int *Last_columnno, char *Last_col_name)
{
  int ok = TRUE;

  if (*Column_ready) {
    ok = add_columnex(lp, *count, Last_column, Last_columnno);
    if (ok) {
      ok = set_col_name(lp, lp->columns, Last_col_name);
    }
    if (ok)
      set_int(lp, lp->columns, Int_section);
  }
  *Column_ready = FALSE;
  *count = 0;
  return(ok);
}

#if 0
STATIC MYBOOL appendmpsitem(int *count, int rowIndex[], REAL rowValue[])
{
  int i = *count;

  if(rowValue[i] == 0)
    return( FALSE );

  while((i > 0) && (rowIndex[i] < rowIndex[i-1])) {
    swapINT (rowIndex+i, rowIndex+i-1);
    swapREAL(rowValue+i, rowValue+i-1);
    i--;
  }
  (*count)++;
  return( TRUE );
}
#endif

STATIC MYBOOL appendmpsitem(int *count, int rowIndex[], REAL rowValue[])
{
  int i = *count;

  /* Check for non-negativity of the index */
  if(rowIndex[i] < 0)
    return( FALSE );

  /* Move the element so that the index list is sorted ascending */
  while((i > 0) && (rowIndex[i] < rowIndex[i-1])) {
    swapINT (rowIndex+i, rowIndex+i-1);
    swapREAL(rowValue+i, rowValue+i-1);
    i--;
  }

  /* Add same-indexed items (which is rarely encountered), and shorten the list */
  if((i < *count) && (rowIndex[i] == rowIndex[i+1])) {
    int ii = i + 1;
    rowValue[i] += rowValue[ii];
    (*count)--;
    while(ii < *count) {
      rowIndex[ii] = rowIndex[ii+1];
      rowValue[ii] = rowValue[ii+1];
      ii++;
    }
  }

  /* Update the count and return */
  (*count)++;
  return( TRUE );
}

MYBOOL MPS_readfile(lprec **newlp, char *filename, int typeMPS, int verbose)
{
  MYBOOL status = FALSE;
  FILE   *fpin;

  fpin = fopen(filename, "r");
  if(fpin != NULL) {
    status = MPS_readhandle(newlp, fpin, typeMPS, verbose);
    fclose(fpin);
  }
  return( status );
}

static int __WINAPI MPS_input(void *fpin, char *buf, int max_size)
{
  return(fgets(buf, max_size, (FILE *) fpin) != NULL);
}

MYBOOL __WINAPI MPS_readhandle(lprec **newlp, FILE *filehandle, int typeMPS, int verbose)
{
  return(MPS_readex(newlp, (void *) filehandle, MPS_input, typeMPS, verbose));
}

MYBOOL __WINAPI MPS_readex(lprec **newlp, void *userhandle, read_modeldata_func read_modeldata, int typeMPS, int verbose)
{
  char   field1[BUFSIZ], field2[BUFSIZ], field3[BUFSIZ], field5[BUFSIZ], line[BUFSIZ], tmp[BUFSIZ],
         Last_col_name[BUFSIZ], probname[BUFSIZ], OBJNAME[BUFSIZ], *ptr;
  int    items, row, Lineno, var,
         section = MPSUNDEF, variant = 0, NZ = 0, SOS = 0;
  MYBOOL Int_section, Column_ready, Column_ready1,
         Unconstrained_rows_found = FALSE, OF_found = FALSE, CompleteStatus = FALSE;
  double field4, field6;
  REAL   *Last_column = NULL;
  int    count = 0, *Last_columnno = NULL;
  int    OBJSENSE = ROWTYPE_EMPTY;
  lprec  *lp;
  int    (*scan_line)(lprec *lp, int section, char* line, char *field1, char *field2, char *field3,
                      double *field4, char *field5, double *field6);

  if(newlp == NULL)
    return( CompleteStatus );
  else if(*newlp == NULL)
    lp = make_lp(0, 0);
  else
    lp = *newlp;

  switch(typeMPS) {
    case MPSFIXED:
      scan_line = scan_lineFIXED;
      break;
    case MPSFREE:
      scan_line = scan_lineFREE;
      break;
    default:
      report(lp, IMPORTANT, "MPS_readfile: Unrecognized MPS line type.\n");
      delete_lp(lp);
      return( CompleteStatus );
  }

  if (lp != NULL) {
    lp->source_is_file = TRUE;
    lp->verbose = verbose;
    strcpy(Last_col_name, "");
    strcpy(OBJNAME, "");
    Int_section = FALSE;
    Column_ready = FALSE;
    Lineno = 0;

    /* let's initialize line to all zero's */
    MEMCLEAR(line, BUFSIZ);

    while(read_modeldata(userhandle, line, BUFSIZ - 1)) {
      Lineno++;

      for(ptr = line; (*ptr) && (isspace((unsigned char) *ptr)); ptr++);

      /* skip lines which start with "*", they are comment */
      if((line[0] == '*') || (*ptr == 0) || (*ptr == '\n') || (*ptr == '\r')) {
        report(lp, FULL, "Comment on line %d: %s", Lineno, line);
        continue;
      }

      report(lp, FULL, "Line %6d: %s", Lineno, line);

      /* first check for "special" lines: NAME, ROWS, BOUNDS .... */
      /* this must start in the first position of line */
      if(line[0] != ' ') {
        sscanf(line, "%s", tmp);
        if(strcmp(tmp, "NAME") == 0) {
          section = MPSNAME;
          *probname = 0;
          sscanf(line, "NAME %s", probname);
          if (!set_lp_name(lp, probname))
            break;
        }
        else if((typeMPS == MPSFREE) && (strcmp(tmp, "OBJSENSE") == 0)) {
          section = MPSOBJSENSE;
          report(lp, FULL, "Switching to OBJSENSE section\n");
        }
        else if((typeMPS == MPSFREE) && (strcmp(tmp, "OBJNAME") == 0)) {
          section = MPSOBJNAME;
          report(lp, FULL, "Switching to OBJNAME section\n");
        }
        else if(strcmp(tmp, "ROWS") == 0) {
          section = MPSROWS;
          report(lp, FULL, "Switching to ROWS section\n");
        }
        else if(strcmp(tmp, "COLUMNS") == 0) {
          allocREAL(lp, &Last_column, lp->rows + 1, TRUE);
          allocINT(lp, &Last_columnno, lp->rows + 1, TRUE);
          count = 0;
          if ((Last_column == NULL) || (Last_columnno == NULL))
            break;
          section = MPSCOLUMNS;
          report(lp, FULL, "Switching to COLUMNS section\n");
        }
        else if(strcmp(tmp, "RHS") == 0) {
          if (!addmpscolumn(lp, Int_section, &Column_ready, &count, Last_column, Last_columnno, Last_col_name))
            break;
          section = MPSRHS;
          report(lp, FULL, "Switching to RHS section\n");
        }
        else if(strcmp(tmp, "BOUNDS") == 0) {
          section = MPSBOUNDS;
          report(lp, FULL, "Switching to BOUNDS section\n");
        }
        else if(strcmp(tmp, "RANGES") == 0) {
          section = MPSRANGES;
          report(lp, FULL, "Switching to RANGES section\n");
        }
        else if((strcmp(tmp, "SOS") == 0) || (strcmp(tmp, "SETS") == 0)) {
          section = MPSSOS;
          if(strcmp(tmp, "SOS") == 0)
            variant = 0;
          else
            variant = 1;
          report(lp, FULL, "Switching to %s section\n", tmp);
        }
        else if(strcmp(tmp, "ENDATA") == 0) {
          report(lp, FULL, "Finished reading MPS file\n");
          CompleteStatus = TRUE;
          break;
        }
        else { /* line does not start with space and does not match above */
          report(lp, IMPORTANT, "Unrecognized MPS line %d: %s\n", Lineno, line);
          break;
        }
      }
      else { /* normal line, process */
        items = scan_line(lp, section, line, field1, field2, field3, &field4, field5, &field6);
        if(items < 0){
          report(lp, IMPORTANT, "Syntax error on line %d: %s\n", Lineno, line);
          break;
        }

        switch(section) {

        case MPSNAME:
          report(lp, IMPORTANT, "Error, extra line under NAME line\n");
          break;

        case MPSOBJSENSE:
          if(OBJSENSE != ROWTYPE_EMPTY) {
            report(lp, IMPORTANT, "Error, extra line under OBJSENSE line\n");
            break;
          }
          if((strcmp(field1, "MAXIMIZE") == 0) || (strcmp(field1, "MAX") == 0)) {
            OBJSENSE = ROWTYPE_OFMAX;
            set_maxim(lp);
          }
          else if((strcmp(field1, "MINIMIZE") == 0) || (strcmp(field1, "MIN") == 0)) {
            OBJSENSE = ROWTYPE_OFMIN;
            set_minim(lp);
          }
          else {
            report(lp, SEVERE, "Unknown OBJSENSE direction '%s' on line %d\n", field1, Lineno);
            break;
          }
          continue;

        case MPSOBJNAME:
          if(*OBJNAME) {
            report(lp, IMPORTANT, "Error, extra line under OBJNAME line\n");
            break;
          }
          strcpy(OBJNAME, field1);
          continue;

        /* Process entries in the ROWS section */
        case MPSROWS:
          /* field1: rel. operator; field2: name of constraint */

          report(lp, FULL, "Row   %5d: %s %s\n", lp->rows + 1, field1, field2);

          if(strcmp(field1, "N") == 0) {
            if((*OBJNAME) && (strcmp(field2, OBJNAME)))
              /* Ignore this objective name since it is not equal to the OBJNAME name */;
            else if(!OF_found) { /* take the first N row as OF, ignore others */
              if (!set_row_name(lp, 0, field2))
                break;
              OF_found = TRUE;
            }
            else if(!Unconstrained_rows_found) {
              report(lp, IMPORTANT, "Unconstrained row %s ignored\n", field2);
              report(lp, IMPORTANT, "Further messages of this kind will be suppressed\n");
              Unconstrained_rows_found = TRUE;
            }
          }
          else if(strcmp(field1, "L") == 0) {
            if ((!str_add_constraint(lp, "" ,LE ,0)) || (!set_row_name(lp, lp->rows, field2)))
              break;
          }
          else if(strcmp(field1, "G") == 0) {
            if ((!str_add_constraint(lp, "" ,GE ,0)) || (!set_row_name(lp, lp->rows, field2)))
              break;
          }
          else if(strcmp(field1, "E") == 0) {
            if ((!str_add_constraint(lp, "",EQ ,0)) || (!set_row_name(lp, lp->rows, field2)))
              break;
          }
          else {
            report(lp, SEVERE, "Unknown relation code '%s' on line %d\n", field1, Lineno);
            break;
          }

          continue;

        /* Process entries in the COLUMNS section */
        case MPSCOLUMNS:
          /* field2: variable; field3: constraint; field4: coef */
          /* optional: field5: constraint; field6: coef */

          report(lp, FULL, "Column %4d: %s %s %g %s %g\n",
                            lp->columns + 1, field2, field3, field4, field5, field6);

          if((items == 4) || (items == 5) || (items == 6)) {
            if (NZ == 0)
              strcpy(Last_col_name, field2);
            else if(*field2) {
              Column_ready1 = (MYBOOL) (strcmp(field2, Last_col_name) != 0);
              if(Column_ready1) {
                if (find_var(lp, field2, FALSE) >= 0) {
                  report(lp, SEVERE, "Variable name (%s) is already used!\n", field2);
                  break;
                }

                if(Column_ready) {  /* Added ability to handle non-standard "same as above" column name */
                  if (addmpscolumn(lp, Int_section, &Column_ready, &count, Last_column, Last_columnno, Last_col_name)) {
                    strcpy(Last_col_name, field2);
                    NZ = 0;
                  }
                  else
                    break;
                }
              }
            }
            if(items == 5) { /* there might be an INTEND or INTORG marker */
             /* look for "    <name>  'MARKER'                 'INTORG'"
                      or "    <name>  'MARKER'                 'INTEND'"  */
              if(strcmp(field3, "'MARKER'") != 0)
                break;
              if(strcmp(field5, "'INTORG'") == 0) {
                Int_section = TRUE;
                report(lp, FULL, "Switching to integer section\n");
              }
              else if(strcmp(field5, "'INTEND'") == 0) {
                Int_section = FALSE;
                report(lp, FULL, "Switching to non-integer section\n");
              }
              else
                report(lp, IMPORTANT, "Unknown marker (ignored) at line %d: %s\n",
                                       Lineno, field5);
            }
            else if((row = find_row(lp, field3, Unconstrained_rows_found)) >= 0) {
              if(row > lp->rows)
                report(lp, CRITICAL, "Invalid row %s encountered in the MPS file\n", field3);
              Last_columnno[count] = row;
              Last_column[count] = (REAL)field4;
              if(appendmpsitem(&count, Last_columnno, Last_column)) {
                NZ++;
                Column_ready = TRUE;
              }
            }
          }
          if(items == 6) {
            if((row = find_row(lp, field5, Unconstrained_rows_found)) >= 0) {
              if(row > lp->rows)
                report(lp, CRITICAL, "Invalid row %s encountered in the MPS file\n", field5);
              Last_columnno[count] = row;
              Last_column[count] = (REAL)field6;
              if(appendmpsitem(&count, Last_columnno, Last_column)) {
                NZ++;
                Column_ready = TRUE;
              }
            }
          }

          if((items < 4) || (items > 6)) { /* Wrong! */
            report(lp, CRITICAL, "Wrong number of items (%d) in COLUMNS section (line %d)\n",
                                  items, Lineno);
            break;
          }

          continue;

        /* Process entries in the RHS section */
        /* field2: uninteresting name; field3: constraint name */
        /* field4: value */
        /* optional: field5: constraint name; field6: value */
        case MPSRHS:

          report(lp, FULL, "RHS line: %s %s %g %s %g\n",
                            field2, field3, field4, field5, field6);

          if((items != 4) && (items != 6)) {
            report(lp, CRITICAL, "Wrong number of items (%d) in RHS section line %d\n",
                                  items, Lineno);
            break;
          }

          if((row = find_row(lp, field3, Unconstrained_rows_found)) >= 0) {
            set_rh(lp, row, (REAL)field4);
          }

          if(items == 6) {
            if((row = find_row(lp, field5, Unconstrained_rows_found)) >= 0) {
              set_rh(lp, row, (REAL)field6);
            }
          }

          continue;

        /* Process entries in the BOUNDS section */
        /* field1: bound type; field2: uninteresting name; */
        /* field3: variable name; field4: value */
        case MPSBOUNDS:

          report(lp, FULL, "BOUNDS line: %s %s %s %g\n",
                            field1, field2, field3, field4);

          var = find_var(lp, field3, FALSE);
          if(var < 0){ /* bound on undefined var in COLUMNS section ... */
            Column_ready = TRUE;
            if (!addmpscolumn(lp, FALSE, &Column_ready, &count, Last_column, Last_columnno, field3))
              break;
            Column_ready = TRUE;
            var = find_var(lp, field3, TRUE);
          }
          if(var < 0) /* undefined var and could add ... */;
          else if(strcmp(field1, "UP") == 0) {
          /* upper bound */
            if(!set_bounds(lp, var, get_lowbo(lp, var), field4))
              break;
          }
          else if(strcmp(field1, "SC") == 0) {
            /* upper bound */
            if(field4 == 0)
              field4 = lp->infinite;
            if(!set_bounds(lp, var, get_lowbo(lp, var), field4))
              break;
            set_semicont(lp, var, TRUE);
          }
          else if(strcmp(field1, "SI") == 0) {
            /* upper bound */
            if(field4 == 0)
              field4 = lp->infinite;
            if(!set_bounds(lp, var, get_lowbo(lp, var), field4))
              break;
            set_int(lp, var, TRUE);
            set_semicont(lp, var, TRUE);
          }
          else if(strcmp(field1, "LO") == 0) {
            /* lower bound */
            if(!set_bounds(lp, var, field4, get_upbo(lp, var)))
              break;
          }
      else if(strcmp(field1, "PL") == 0) { /* plus-ranged variable */
            if(!set_bounds(lp, var, get_lowbo(lp, var), lp->infinite))
              break;
      }
          else if(strcmp(field1, "MI") == 0) { /* minus-ranged variable */
            if(!set_bounds(lp, var, -lp->infinite, get_upbo(lp, var)))
              break;
          }
          else if(strcmp(field1, "FR") == 0) { /* free variable */
            set_unbounded(lp, var);
          }
          else if(strcmp(field1, "FX") == 0) {
            /* fixed, upper _and_ lower  */
            if(!set_bounds(lp, var, field4, field4))
              break;
          }
          else if(strcmp(field1, "BV") == 0) { /* binary variable */
            set_binary(lp, var, TRUE);
          }
          /* AMPL bounds type UI and LI added by E.Imamura (CRIEPI)  */
          else if(strcmp(field1, "UI") == 0) { /* upper bound for integer variable */
            if(!set_bounds(lp, var, get_lowbo(lp, var), field4))
              break;
            set_int(lp, var, TRUE);
          }
          else if(strcmp(field1, "LI") == 0) { /* lower bound for integer variable - corrected by KE */
            if(!set_bounds(lp, var, field4, get_upbo(lp, var)))
              break;
            set_int(lp, var, TRUE);
          }
          else {
            report(lp, CRITICAL, "BOUND type %s on line %d is not supported",
                                  field1, Lineno);
            break;
          }

          continue;

          /* Process entries in the BOUNDS section */

      /* We have to implement the following semantics:

      D. The RANGES section is for constraints of the form: h <=
      constraint <= u .  The range of the constraint is r = u - h .  The
      value of r is specified in the RANGES section, and the value of u or
      h is specified in the RHS section.  If b is the value entered in the
      RHS section, and r is the value entered in the RANGES section, then
      u and h are thus defined:

      row type       sign of r       h          u
      ----------------------------------------------
     G            + or -         b        b + |r|
     L            + or -       b - |r|      b
     E              +            b        b + |r|
     E              -          b - |r|      b            */

        /* field2: uninteresting name; field3: constraint name */
        /* field4: value */
        /* optional: field5: constraint name; field6: value */

        case MPSRANGES:

          report(lp, FULL, "RANGES line: %s %s %g %s %g",
                            field2, field3, field4, field5, field6);

          if((items != 4) && (items != 6)) {
            report(lp, CRITICAL, "Wrong number of items (%d) in RANGES section line %d",
                                  items, Lineno);
            break;
          }

          if((row = find_row(lp, field3, Unconstrained_rows_found)) >= 0) {
            /* Determine constraint type */

            if(fabs(field4) >= lp->infinite) {
              report(lp, IMPORTANT,
                          "Warning, Range for row %s >= infinity (value %g) on line %d, ignored",
                          field3, field4, Lineno);
            }
            else if(field4 == 0) {
              /* Change of a GE or LE to EQ */
              if(lp->orig_upbo[row] != 0)
                set_constr_type(lp, row, EQ);
            }
            else if(is_chsign(lp, row)) {
              /* GE */
              lp->orig_upbo[row] = fabs(field4);
            }
            else if((lp->orig_upbo[row] == 0) && (field4 >= 0)) {
              /*  EQ with positive sign of r value */
              set_constr_type(lp, row, GE);
              lp->orig_upbo[row] = field4;
            }
            else if(lp->orig_upbo[row] == lp->infinite) {
              /* LE */
              lp->orig_upbo[row] = fabs(field4);
            }
            else if((lp->orig_upbo[row] == 0) && (field4 < 0)) {
              /* EQ with negative sign of r value */
              set_constr_type(lp, row, LE);
              lp->orig_upbo[row] = my_flipsign(field4);
            }
            else { /* let's be paranoid */
              report(lp, IMPORTANT,
                          "Cannot figure out row type, row = %d, is_chsign = %d, upbo = %g on line %d",
                          row, is_chsign(lp, row), (double)lp->orig_upbo[row], Lineno);
            }
          }

          if(items == 6) {
            if((row = find_row(lp, field5, Unconstrained_rows_found)) >= 0) {
              /* Determine constraint type */

              if(fabs(field6) >= lp->infinite) {
                report(lp, IMPORTANT,
                            "Warning, Range for row %s >= infinity (value %g) on line %d, ignored",
                            field5, field6, Lineno);
              }
              else if(field6 == 0) {
                /* Change of a GE or LE to EQ */
                if(lp->orig_upbo[row] != 0)
                  set_constr_type(lp, row, EQ);
              }
              else if(is_chsign(lp, row)) {
                /* GE */
                lp->orig_upbo[row] = fabs(field6);
              }
              else if(lp->orig_upbo[row] == 0 && field6 >= 0) {
                /*  EQ with positive sign of r value */
                set_constr_type(lp, row, GE);
                lp->orig_upbo[row] = field6;
              }
              else if(lp->orig_upbo[row] == lp->infinite) {
                /* LE */
                lp->orig_upbo[row] = fabs(field6);
              }
              else if((lp->orig_upbo[row] == 0) && (field6 < 0)) {
                /* EQ with negative sign of r value */
                set_constr_type(lp, row, LE);
                lp->orig_upbo[row] = my_flipsign(field6);
              }
              else { /* let's be paranoid */
                report(lp, IMPORTANT,
                            "Cannot figure out row type, row = %d, is_chsign = %d, upbo = %g on line %d",
                            row, is_chsign(lp,row), (double) lp->orig_upbo[row], Lineno);
              }
            }
          }

          continue;

        /* Process entries in the SOS section */

        /* We have to implement the following semantics:

          E. The SOS section is for ordered variable sets of the form:
      x1, x2, x3 ... xn where only a given number of consequtive variables
          may be non-zero.  Each set definition is prefaced by type, name
      and priority data.  Each set member has an optional weight that
      determines its order.  There are two forms supported; a full format
      and a reduced CPLEX-like format.                                       */

        case MPSSOS:
          report(lp, FULL, "SOS line: %s %s %g %s %g",
                             field2, field3, field4, field5, field6);

          if((items == 0) || (items > 4)) {
            report(lp, IMPORTANT,
                   "Invalid number of items (%d) in SOS section line %d\n",
                   items, Lineno);
            break;
          }

          if(strlen(field1) == 0) items--;  /* fix scanline anomoly! */

          /* Check if this is the start of a new SOS */
          if(items == 1 || items == 4) {
            row = (int) (field1[1] - '0');
            if((row <= 0) || (row > 9)) {
              report(lp, IMPORTANT,
                     "Error: Invalid SOS type %s line %d\n", field1, Lineno);
              break;
            }
            field1[0] = '\0';               /* fix scanline anomoly! */

            /* lp_solve needs a name for the SOS */
            if(variant == 0) {
              if(strlen(field3) == 0)  /* CPLEX format does not provide a SOS name; create one */
                sprintf(field3, "SOS_%d", SOS_count(lp) + 1);
            }
            else {                     /* Remap XPRESS format name */
              strcpy(field3, field1);
            }
            /* Obtain the SOS priority */
            if(items == 4)
              SOS = (int) field4;
            else
              SOS = 1;

            /* Define a new SOS instance */

            SOS = add_SOS(lp, field3, (int) row, SOS, 0, NULL, NULL);
          }
          /* Otherwise, add set members to the active SOS */
          else {
            char *field = (items == 3) ? field3 /* Native lp_solve and XPRESS formats */ : field2 /* CPLEX format */;

            var = find_var(lp, field, FALSE);  /* Native lp_solve and XPRESS formats */
            if(var < 0){ /* SOS on undefined var in COLUMNS section ... */
              Column_ready = TRUE;
              if (!addmpscolumn(lp, FALSE, &Column_ready, &count, Last_column, Last_columnno, field))
                break;
              Column_ready = TRUE;
              var = find_var(lp, field, TRUE);
            }
            if((var < 0) || (SOS < 1)) /* undefined var and could add ... */;
            else append_SOSrec(lp->SOS->sos_list[SOS-1], 1, &var, &field4);
          }

          continue;
        }

        /* If we got here there was an error "upstream" */
         report(lp, IMPORTANT,
                     "Error: Cannot handle line %d\n", Lineno);
         break;
      }
    }

    if((*OBJNAME) && (!OF_found)) {
      report(lp, IMPORTANT,
                  "Error: Objective function specified by OBJNAME card not found\n");
      CompleteStatus = FALSE;
    }

    if(CompleteStatus == FALSE)
      delete_lp(lp);
    else
      *newlp = lp;
    if(Last_column != NULL)
      FREE(Last_column);
    if(Last_columnno != NULL)
      FREE(Last_columnno);
  }

  return( CompleteStatus );
}

static void number(char *str,REAL value)
 {
  char __str[80], *_str;
  int  i;

  /* sprintf(_str,"%12.6G",value); */
  _str=__str+2;
  if (value>=0.0)
   if ((value!=0.0) && ((value>0.99999999e12) || (value<0.0001))) {
    int n=15;

    do {
     n--;
     i=sprintf(_str,"%*.*E",n,n-6,(double) value);
     if (i>12) {
      char *ptr=strchr(_str,'E');

      if (ptr!=NULL) {
       if (*(++ptr)=='-') ptr++;
       while ((i>12) && ((*ptr=='+') || (*ptr=='0'))) {
        strcpy(ptr,ptr+1);
        i--;
       }
      }
     }
    } while (i>12);
   }
   else if (value>=1.0e10) {
    int n=13;

    do {
     i=sprintf(_str,"%*.0f",--n,(double) value);
    } while (i>12);
   }
   else {
    if (((i=sprintf(_str,"%12.10f",(double) value))>12) && (_str[12]>='5')) {
     for (i=11;i>=0;i--)
      if (_str[i]!='.') {
       if (++_str[i]>'9') _str[i]='0';
       else break;
      }
     if (i<0) {
      *(--_str)='1';
      *(--_str)=' ';
     }
    }
   }
  else
   if ((value<-0.99999999e11) || (value>-0.0001)) {
    int n=15;

    do {
     n--;
     i=sprintf(_str,"%*.*E",n,n-7,(double) value);
     if (i>12) {
      char *ptr=strchr(_str,'E');

      if (ptr!=NULL) {
       if (*(++ptr)=='-') ptr++;
       while ((i>12) && ((*ptr=='+') || (*ptr=='0'))) {
        strcpy(ptr,ptr+1);
        i--;
       }
      }
     }
    } while (i>12);
   }
   else if (value<=-1.0e9) {
    int n=13;

    do {
     i=sprintf(_str,"%*.0f",--n,(double) value);
    } while (i>12);
   }
   else
    if (((i=sprintf(_str,"%12.9f",(double) value))>12) && (_str[12]>='5')) {
     for (i=11;i>=1;i--)
      if (_str[i]!='.') {
       if (++_str[i]>'9') _str[i]='0';
       else break;
      }
     if (i<1) {
      *_str='1';
      *(--_str)='-';
      *(--_str)=' ';
     }
    }
  strncpy(str,_str,12);
 }

static char numberbuffer[15];

static char *formatnumber12(double a)
{
#if 0
  return(sprintf(numberbuffer, "%12g", a));
#else
  number(numberbuffer, a);
  return(numberbuffer);
#endif
}

STATIC char *MPSnameFIXED(char *name)
{
  static char name0[9];

  sprintf(name0, "%-8.8s", name);
  return(name0);
}

STATIC char *MPSnameFREE(char *name)
{
  if(strlen(name) < 8)
    return(MPSnameFIXED(name));
  else
    return(name);
}

static void write_data(void *userhandle, write_modeldata_func write_modeldata, char *format, ...)
{
  char buff[DEF_STRBUFSIZE+1];
  va_list ap;

  va_start(ap, format);
  vsnprintf(buff, DEF_STRBUFSIZE, format, ap);
  write_modeldata(userhandle, buff);
  va_end(ap);
}

MYBOOL MPS_writefileex(lprec *lp, int typeMPS, void *userhandle, write_modeldata_func write_modeldata)
{
  int    i, j, jj, je, k, marker, putheader, ChangeSignObj = FALSE, *idx, *idx1;
  MYBOOL ok = TRUE, names_used;
  REAL   a, *val, *val1;
  FILE   *output = stdout;
  char * (*MPSname)(char *name);

  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "MPS_writefile: Cannot write to MPS file while in row entry mode.\n");
    return(FALSE);
  }

  switch(typeMPS) {
    case MPSFIXED:
      MPSname = MPSnameFIXED;
      ChangeSignObj = is_maxim(lp);
      break;
    case MPSFREE:
      MPSname = MPSnameFREE;
      break;
    default:
      report(lp, IMPORTANT, "MPS_writefile: unrecognized MPS name type.\n");
      return(FALSE);
  }

  names_used = lp->names_used;

  if(typeMPS == MPSFIXED) {
    /* Check if there is no variable name where the first 8 charachters are equal to the first 8 characters of anothe variable */
    if(names_used)
      for(i = 1; (i <= lp->columns) && (ok); i++)
        if((lp->col_name[i] != NULL) && (lp->col_name[i]->name != NULL) && (!is_splitvar(lp, i)) && (strlen(lp->col_name[i]->name) > 8))
          for(j = 1; (j < i) && (ok); j++)
    if((lp->col_name[j] != NULL) && (lp->col_name[j]->name != NULL) && (!is_splitvar(lp, j)))
      if(strncmp(lp->col_name[i]->name, lp->col_name[j]->name, 8) == 0)
        ok = FALSE;
  }

  if(!ok) {
    lp->names_used = FALSE;
    ok = TRUE;
  }
  marker = 0;

  /* First write metadata in structured comment form (lp_solve style) */
  write_data(userhandle, write_modeldata, "*<meta creator='lp_solve v%d.%d'>\n",
                  (int) MAJORVERSION, (int) MINORVERSION);
  write_data(userhandle, write_modeldata, "*<meta rows=%d>\n", lp->rows);
  write_data(userhandle, write_modeldata, "*<meta columns=%d>\n", lp->columns);
  write_data(userhandle, write_modeldata, "*<meta equalities=%d>\n", lp->equalities);
  if(SOS_count(lp) > 0)
    write_data(userhandle, write_modeldata, "*<meta SOS=%d>\n", SOS_count(lp));
  write_data(userhandle, write_modeldata, "*<meta integers=%d>\n", lp->int_vars);
  if(lp->sc_vars > 0)
    write_data(userhandle, write_modeldata, "*<meta scvars=%d>\n", lp->sc_vars);
  write_data(userhandle, write_modeldata, "*<meta origsense='%s'>\n", (is_maxim(lp) ? "MAX" : "MIN"));
  write_data(userhandle, write_modeldata, "*\n");

  /* Write the MPS content */
  write_data(userhandle, write_modeldata, "NAME          %s\n", MPSname(get_lp_name(lp)));
  if((typeMPS == MPSFREE) && (is_maxim(lp)))
    write_data(userhandle, write_modeldata, "OBJSENSE\n MAX\n");
  write_data(userhandle, write_modeldata, "ROWS\n");
  for(i = 0; i <= lp->rows; i++) {
    if(i == 0)
      write_data(userhandle, write_modeldata, " N  ");
    else if(lp->orig_upbo[i] != 0) {
      if(is_chsign(lp,i))
        write_data(userhandle, write_modeldata, " G  ");
      else
        write_data(userhandle, write_modeldata, " L  ");
    }
    else
      write_data(userhandle, write_modeldata, " E  ");
    write_data(userhandle, write_modeldata, "%s\n", MPSname(get_row_name(lp, i)));
  }

  allocREAL(lp, &val, 1 + lp->rows, TRUE);
  allocINT(lp, &idx, 1 + lp->rows, TRUE);
  write_data(userhandle, write_modeldata, "COLUMNS\n");
  for(i = 1; i <= lp->columns; i++) {
    if(!is_splitvar(lp, i)) {
      if(is_int(lp,i) && (marker % 2) == 0) {
        write_data(userhandle, write_modeldata, "    MARK%04d  'MARKER'                 'INTORG'\n",
                marker);
        marker++;
      }
      if(!is_int(lp,i) && (marker % 2) == 1) {
        write_data(userhandle, write_modeldata, "    MARK%04d  'MARKER'                 'INTEND'\n",
                marker);
        marker++;
      }

      /* Loop over non-zero column entries */
      je = get_columnex(lp, i, val, idx);
      for(k = 1, val1 = val, idx1 = idx, jj = 0; jj < je; jj++) {
        k = 1 - k;
        j = *(idx1++);
        a = *(val1++);
        if (k == 0) {
          write_data(userhandle, write_modeldata, "    %s",
                          MPSname(get_col_name(lp, i)));
          write_data(userhandle, write_modeldata, "  %s  %s",
                          MPSname(get_row_name(lp, j)),
/*                          formatnumber12((double) a)); */
                          formatnumber12((double) (a * (j == 0 && ChangeSignObj ? -1 : 1))));
    }
        else
          write_data(userhandle, write_modeldata, "   %s  %s\n",
                          MPSname(get_row_name(lp, j)),
                          formatnumber12((double) (a * (j == 0 && ChangeSignObj ? -1 : 1))));
/*                          formatnumber12((double) a)); */
      }
      if(k == 0)
        write_data(userhandle, write_modeldata, "\n");
    }
  }
  if((marker % 2) == 1) {
    write_data(userhandle, write_modeldata, "    MARK%04d  'MARKER'                 'INTEND'\n",
            marker);
  /* marker++; */ /* marker not used after this */
  }
  FREE(idx);
  FREE(val);

  write_data(userhandle, write_modeldata, "RHS\n");
  for(k = 1, i = 0; i <= lp->rows; i++) {
    a = lp->orig_rhs[i];
    if(a) {
      a = unscaled_value(lp, a, i);
      if((i == 0) || is_chsign(lp, i))
        a = my_flipsign(a);
      k = 1 - k;
      if(k == 0)
        write_data(userhandle, write_modeldata, "    RHS       %s  %s",
                        MPSname(get_row_name(lp, i)),
                        formatnumber12((double)a));
      else
        write_data(userhandle, write_modeldata, "   %s  %s\n",
                        MPSname(get_row_name(lp, i)),
                        formatnumber12((double)a));
    }
  }
  if(k == 0)
    write_data(userhandle, write_modeldata, "\n");

  putheader = TRUE;
  for(k = 1, i = 1; i <= lp->rows; i++){
    a = 0;
    if((lp->orig_upbo[i] < lp->infinite) && (lp->orig_upbo[i] != 0.0))
      a = lp->orig_upbo[i];
    if(a) {
      if(putheader) {
        write_data(userhandle, write_modeldata, "RANGES\n");
        putheader = FALSE;
      }
      a = unscaled_value(lp, a, i);
      k = 1 - k;
      if(k == 0)
        write_data(userhandle, write_modeldata, "    RGS       %s  %s",
                        MPSname(get_row_name(lp, i)),
                        formatnumber12((double)a));
      else
        write_data(userhandle, write_modeldata, "   %s  %s\n",
                        MPSname(get_row_name(lp, i)),
                        formatnumber12((double)a));
    }
  }
  if(k == 0)
    write_data(userhandle, write_modeldata, "\n");

  putheader = TRUE;
  for(i = lp->rows + 1; i <= lp->sum; i++)
    if(!is_splitvar(lp, i - lp->rows)) {
      j = i - lp->rows;
      if((lp->orig_lowbo[i] != 0) && (lp->orig_upbo[i] < lp->infinite) &&
         (lp->orig_lowbo[i] == lp->orig_upbo[i])) {
        a = lp->orig_upbo[i];
        a = unscaled_value(lp, a, i);
        if(putheader) {
          write_data(userhandle, write_modeldata, "BOUNDS\n");
          putheader = FALSE;
        }
        write_data(userhandle, write_modeldata, " FX BND       %s  %s\n",
                        MPSname(get_col_name(lp, j)),
                        formatnumber12((double)a));
      }
      else if(is_binary(lp, j)) {
        if(putheader) {
          write_data(userhandle, write_modeldata, "BOUNDS\n");
          putheader = FALSE;
        }
        write_data(userhandle, write_modeldata, " BV BND       %s\n",
                        MPSname(get_col_name(lp, j)));
      }
      else if(is_unbounded(lp, j)) {
        if(putheader) {
          write_data(userhandle, write_modeldata, "BOUNDS\n");
          putheader = FALSE;
        }
        write_data(userhandle, write_modeldata, " FR BND       %s\n",
                        MPSname(get_col_name(lp, j)));
      }
      else {
        if((lp->orig_upbo[i] < lp->infinite) || (is_semicont(lp, j))) {
          a = lp->orig_upbo[i];
      if(a < lp->infinite)
            a = unscaled_value(lp, a, i);
          if(putheader) {
            write_data(userhandle, write_modeldata, "BOUNDS\n");
            putheader = FALSE;
          }
          if(is_semicont(lp, j)) {
            if(is_int(lp, j))
              write_data(userhandle, write_modeldata, " SI BND       %s  %s\n",
                              MPSname(get_col_name(lp, j)),
                  (a < lp->infinite) ? formatnumber12((double)a) : "            ");
            else
              write_data(userhandle, write_modeldata, " SC BND       %s  %s\n",
                              MPSname(get_col_name(lp, j)),
                              (a < lp->infinite) ? formatnumber12((double)a) : "            ");
          }
          else
            write_data(userhandle, write_modeldata, " UP BND       %s  %s\n",
                            MPSname(get_col_name(lp, j)),
                            formatnumber12((double)a));
        }
        if(lp->orig_lowbo[i] != 0) {
          a = lp->orig_lowbo[i];
          a = unscaled_value(lp, a, i);
          if(putheader) {
            write_data(userhandle, write_modeldata, "BOUNDS\n");
            putheader = FALSE;
          }
          if(lp->orig_lowbo[i] != -lp->infinite)
            write_data(userhandle, write_modeldata, " LO BND       %s  %s\n",
                            MPSname(get_col_name(lp, j)),
                            formatnumber12((double)a));
          else
            write_data(userhandle, write_modeldata, " MI BND       %s\n",
                            MPSname(get_col_name(lp, j)));
        }
      }
    }

 /* Write optional SOS section */
  putheader = TRUE;
  for(i = 0; i < SOS_count(lp); i++) {
    SOSgroup *SOS = lp->SOS;

    if(putheader) {
      write_data(userhandle, write_modeldata, "SOS\n");
      putheader = FALSE;
    }
    write_data(userhandle, write_modeldata, " S%1d SOS       %s  %s\n",
                    SOS->sos_list[i]->type,
                    MPSname(SOS->sos_list[i]->name),
                    formatnumber12((double) SOS->sos_list[i]->priority));
    for(j = 1; j <= SOS->sos_list[i]->size; j++) {
      write_data(userhandle, write_modeldata, "    SOS       %s  %s\n",
                      MPSname(get_col_name(lp, SOS->sos_list[i]->members[j])),
                      formatnumber12((double) SOS->sos_list[i]->weights[j]));
    }
  }

  write_data(userhandle, write_modeldata, "ENDATA\n");

  lp->names_used = names_used;

  return(ok);
}

static int __WINAPI write_lpdata(void *userhandle, char *buf)
{
  fputs(buf, (FILE *) userhandle);
  return(TRUE);
}

MYBOOL MPS_writefile(lprec *lp, int typeMPS, char *filename)
{
  FILE *output = stdout;
  MYBOOL ok;

  if (filename != NULL) {
    ok = ((output = fopen(filename, "w")) != NULL);
    if(!ok)
      return(ok);
  }
  else
    output = lp->outstream;

  ok = MPS_writefileex(lp, typeMPS, (void *) output, write_lpdata);

  if (filename != NULL)
    fclose(output);

  return(ok);
}

MYBOOL MPS_writehandle(lprec *lp, int typeMPS, FILE *output)
{
  MYBOOL ok;

  if (output != NULL)
    set_outputstream(lp, output);

  output = lp->outstream;

  ok = MPS_writefileex(lp, typeMPS, (void *) output, write_lpdata);

  return(ok);
}


/* Read and write BAS files */
/* #define OldNameMatch */
#ifdef OldNameMatch
static int MPS_getnameidx(lprec *lp, char *varname, MYBOOL isrow)
{
  int in = -1;

  in = get_nameindex(lp, varname, isrow);
  if((in < 0) && (strncmp(varname, (isrow ? ROWNAMEMASK : COLNAMEMASK), 1) == 0)) {
    if(sscanf(varname + 1, "%d", &in) != 1)
      in = -1;
  }
  return( in );
}
#else
static int MPS_getnameidx(lprec *lp, char *varname, MYBOOL tryrowfirst)
{
  int in = -1;

  /* Have we defined our own variable names? */
  if(lp->names_used) {
    /* First check the primary name list */
    in = get_nameindex(lp, varname, tryrowfirst);
    if((in > 0) && !tryrowfirst)
      in += lp->rows;
    /* If we were unsuccessful, try the secondary name list */
    else if(in < 0) {
      in = get_nameindex(lp, varname, (MYBOOL) !tryrowfirst);
      if((in > 0) && tryrowfirst)
        in += lp->rows;
    }
  }
  /* If not, see if we can match the standard name mask */

  if(in == -1) {
    if(strncmp(varname, (tryrowfirst ? ROWNAMEMASK : COLNAMEMASK), 1) == 0) {
      /* Fail if we did not successfully scan as a valid integer */
      if((sscanf(varname + 1, "%d", &in) != 1) ||
         (in < (tryrowfirst ? 0 : 1)) || (in > (tryrowfirst ? lp->rows : lp->columns)))
        in = -1;
    }
    else if(strncmp(varname, (!tryrowfirst ? ROWNAMEMASK : COLNAMEMASK), 1) == 0) {
      /* Fail if we did not successfully scan as a valid integer */
      if((sscanf(varname + 1, "%d", &in) != 1) ||
         (in < (tryrowfirst ? 0 : 1)) || (in > (tryrowfirst ? lp->rows : lp->columns)))
        in = -1;
    }
  }
  return( in );
}
#endif

MYBOOL MPS_readBAS(lprec *lp, int typeMPS, char *filename, char *info)
{
  char   field1[BUFSIZ], field2[BUFSIZ], field3[BUFSIZ], field5[BUFSIZ],
         line[BUFSIZ], tmp[BUFSIZ], *ptr;
  double field4, field6;
  int    ib, in, items, Lineno = 0;
  MYBOOL ok;
  FILE   *input = stdin;
  int    (*scan_line)(lprec *lp, int section, char* line, char *field1, char *field2, char *field3,
                      double *field4, char *field5, double *field6);

  switch(typeMPS) {
    case MPSFIXED:
      scan_line = scan_lineFIXED;
      break;
    case MPSFREE:
      scan_line = scan_lineFREE;
      break;
    default:
      report(lp, IMPORTANT, "MPS_readBAS: unrecognized MPS line type.\n");
      return(FALSE);
  }

  ok = (MYBOOL) ((filename != NULL) && ((input = fopen(filename,"r")) != NULL));
  if(!ok)
    return(ok);
  default_basis(lp);

  /* Let's initialize line to all zero's */
  MEMCLEAR(line, BUFSIZ);
  ok = FALSE;
  while(fgets(line, BUFSIZ - 1, input)) {
    Lineno++;

    for(ptr = line; (*ptr) && (isspace((unsigned char) *ptr)); ptr++);

    /* skip lines which start with "*", they are comment */
    if((line[0] == '*') || (*ptr == 0) || (*ptr == '\n') || (*ptr == '\r')) {
      report(lp, FULL, "Comment on line %d: %s", Lineno, line);
      continue;
    }

    report(lp, FULL, "Line %6d: %s", Lineno, line);

    /* first check for "special" lines: in our case only NAME and ENDATA,
       ...this must start in the first position of line */
    if(line[0] != ' ') {
      sscanf(line, "%s", tmp);
      if(strcmp(tmp, "NAME") == 0) {
        if(info != NULL) {
          *info = 0;
          for(ptr = line + 4; (*ptr) && (isspace((unsigned char) *ptr)); ptr++);
          in = (int) strlen(ptr);
          while ((in > 0) && ((ptr[in - 1] == '\r') || (ptr[in - 1] == '\n') || isspace(ptr[in - 1])))
            in--;
          ptr[in] = 0;
          strcpy(info, ptr);
        }
      }
      else if(strcmp(tmp, "ENDATA") == 0) {
        report(lp, FULL, "Finished reading BAS file\n");
        ok = TRUE;
        break;
      }
      else { /* line does not start with space and does not match above */
        report(lp, IMPORTANT, "Unrecognized BAS line %d: %s\n", Lineno, line);
        break;
      }
    }
    else { /* normal line, process */
      items = scan_line(lp, MPSRHS, line, field1, field2, field3, &field4, field5, &field6);
      if(items < 0){
        report(lp, IMPORTANT, "Syntax error on line %d: %s\n", Lineno, line);
        break;
      }
      /* find first variable index value */
      in = MPS_getnameidx(lp, field2, FALSE);
#ifdef OldNameMatch
      if(in < 0)
        in = MPS_getnameidx(lp, field2, TRUE);
      else
        in += lp->rows;
#endif
      if(in < 0)
        break;

      /* check if we have the basic/non-basic variable format */
      if(field1[0] == 'X') {
        /* find second variable index value */
        ib = in;
        in = MPS_getnameidx(lp, field3, FALSE);
#ifdef OldNameMatch
        if(in < 0)
          in = MPS_getnameidx(lp, field3, TRUE);
        else
          in += lp->rows;
#endif
        if(in < 0)
          break;

        lp->is_lower[in] = (MYBOOL) (field1[1] == 'L');
        lp->is_basic[ib] = TRUE;
      }
      else
        lp->is_lower[in] = (MYBOOL) (field1[0] == 'L');

      lp->is_basic[in] = FALSE;

    }
  }
  /* Update the basis index-to-variable array */
  ib = 0;
  items = lp->sum;
  for(in = 1; in <= items; in++)
    if(lp->is_basic[in]) {
      ib++;
      lp->var_basic[ib] = in;
    }

  fclose(input);
  return( ok );
}

MYBOOL MPS_writeBAS(lprec *lp, int typeMPS, char *filename)
{
  int    ib, in;
  MYBOOL ok;
  char   name1[100], name2[100];
  FILE   *output = stdout;
  char * (*MPSname)(char *name);

  /* Set name formatter */
  switch(typeMPS) {
    case MPSFIXED:
      MPSname = MPSnameFIXED;
      break;
    case MPSFREE:
      MPSname = MPSnameFREE;
      break;
    default:
      report(lp, IMPORTANT, "MPS_writeBAS: unrecognized MPS name type.\n");
      return(FALSE);
  }

  /* Open the file for writing */
  ok = (MYBOOL) ((filename == NULL) || ((output = fopen(filename,"w")) != NULL));
  if(!ok)
    return(ok);
  if(filename == NULL && lp->outstream != NULL)
    output = lp->outstream;

  fprintf(output, "NAME          %s Rows %d Cols %d Iters %.0f\n",
                  get_lp_name(lp), lp->rows, lp->columns, (double) get_total_iter(lp));

  ib = lp->rows;
  in = 0;
  while ((ib < lp->sum) || (in < lp->sum)) {

    /* Find next basic variable (skip slacks) */
    ib++;
    while((ib <= lp->sum) && !lp->is_basic[ib])
      ib++;

    /* Find next non-basic variable (skip lower-bounded structural variables) */
    in++;
    while((in <= lp->sum) && (lp->is_basic[in] ||
                              ((in > lp->rows) && lp->is_lower[in])))
      in++;

    /* Check if we have a basic/non-basic variable pair */
    if((ib <= lp->sum) && (in <= lp->sum)) {
      strcpy(name1, MPSname((ib <= lp->rows ? get_row_name(lp, ib) :
                                              get_col_name(lp, ib-lp->rows))));
      strcpy(name2, MPSname((in <= lp->rows ? get_row_name(lp, in) :
                                              get_col_name(lp, in-lp->rows))));
      fprintf(output, " %2s %s  %s\n", (lp->is_lower[in] ? "XL" : "XU"), name1, name2);
    }

    /* Otherwise just write the bound state of the non-basic variable */
    else if(in <= lp->sum) {
      strcpy(name1, MPSname((in <= lp->rows ? get_row_name(lp, in) :
                                              get_col_name(lp, in-lp->rows))));
      fprintf(output, " %2s %s\n", (lp->is_lower[in] ? "LL" : "UL"), name1);
    }

  }
  fprintf(output, "ENDATA\n");

  if(filename != NULL)
    fclose(output);
  return( ok );
}
