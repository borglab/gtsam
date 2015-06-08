/************************************************************************/
/*! \file 

\brief Functions for manipulating strings.

Various functions for manipulating strings. Some of these functions 
provide new functionality, whereas others are drop-in replacements
of standard functions (but with enhanced functionality).

\date Started 11/1/99
\author George
\version $Id: string.c 10711 2011-08-31 22:23:04Z karypis $
*/
/************************************************************************/

#include <GKlib.h>



/************************************************************************/
/*! \brief Replaces certain characters in a string.
 
This function takes a string and replaces all the characters in the
\c fromlist with the corresponding characters from the \c tolist. 
That is, each occurence of <tt>fromlist[i]</tt> is replaced by 
<tt>tolist[i]</tt>. 
If the \c tolist is shorter than \c fromlist, then the corresponding 
characters are deleted. The modifications on \c str are done in place. 
It tries to provide a functionality similar to Perl's \b tr// function.

\param str is the string whose characters will be replaced.
\param fromlist is the set of characters to be replaced.
\param tolist is the set of replacement characters .
\returns A pointer to \c str itself.
*/
/************************************************************************/
char *gk_strchr_replace(char *str, char *fromlist, char *tolist)
{
  gk_idx_t i, j, k; 
  size_t len, fromlen, tolen;

  len     = strlen(str);
  fromlen = strlen(fromlist);
  tolen   = strlen(tolist);

  for (i=j=0; i<len; i++) {
    for (k=0; k<fromlen; k++) {
      if (str[i] == fromlist[k]) {
        if (k < tolen) 
          str[j++] = tolist[k];
        break;
      }
    }
    if (k == fromlen)
      str[j++] = str[i];
  }
  str[j] = '\0';

  return str;
}



/************************************************************************/
/*! \brief Regex-based search-and-replace function
 
This function is a C implementation of Perl's <tt> s//</tt> regular-expression
based substitution function.

\param str 
  is the input string on which the operation will be performed.
\param pattern
  is the regular expression for the pattern to be matched for substitution.
\param replacement
  is the replacement string, in which the possible captured pattern substrings
  are referred to as $1, $2, ..., $9. The entire matched pattern is refered
  to as $0.
\param options
  is a string specified options for the substitution operation. Currently the
  <tt>"i"</tt> (case insensitive) and <tt>"g"</tt> (global substitution) are 
  supported.
\param new_str 
  is a reference to a pointer that will store a pointer to the newly created 
  string that results from the substitutions. This string is allocated via 
  gk_malloc() and needs to be freed using gk_free(). The string is returned 
  even if no substitutions were performed.
\returns
  If successful, it returns 1 + the number of substitutions that were performed.
  Thus, if no substitutions were performed, the returned value will be 1.
  Otherwise it returns 0. In case of error, a meaningful error message is 
  returned in <tt>newstr</tt>, which also needs to be freed afterwards.
*/
/************************************************************************/
int gk_strstr_replace(char *str, char *pattern, char *replacement, char *options,
      char **new_str)
{
  gk_idx_t i; 
  int j, rc, flags, global, nmatches;
  size_t len, rlen, nlen, offset, noffset;
  regex_t re;
  regmatch_t matches[10];

  
  /* Parse the options */
  flags = REG_EXTENDED;
  if (strchr(options, 'i') != NULL)
    flags = flags | REG_ICASE;
  global = (strchr(options, 'g') != NULL ? 1 : 0);


  /* Compile the regex */
  if ((rc = regcomp(&re, pattern, flags)) != 0) { 
    len = regerror(rc, &re, NULL, 0);
    *new_str = gk_cmalloc(len, "gk_strstr_replace: new_str");
    regerror(rc, &re, *new_str, len);
    return 0;
  }

  /* Prepare the output string */
  len = strlen(str);
  nlen = 2*len;
  noffset = 0;
  *new_str = gk_cmalloc(nlen+1, "gk_strstr_replace: new_str");


  /* Get into the matching-replacing loop */
  rlen = strlen(replacement);
  offset = 0;
  nmatches = 0;
  do {
    rc = regexec(&re, str+offset, 10, matches, 0);

    if (rc == REG_ESPACE) {
      gk_free((void **)new_str, LTERM);
      *new_str = gk_strdup("regexec ran out of memory.");
      regfree(&re);
      return 0;
    }
    else if (rc == REG_NOMATCH) {
      if (nlen-noffset < len-offset) {
        nlen += (len-offset) - (nlen-noffset);
        *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
      }
      strcpy(*new_str+noffset, str+offset);
      noffset += (len-offset);
      break;
    }
    else { /* A match was found! */
      nmatches++;

      /* Copy the left unmatched portion of the string */
      if (matches[0].rm_so > 0) {
        if (nlen-noffset < matches[0].rm_so) {
          nlen += matches[0].rm_so - (nlen-noffset);
          *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
        }
        strncpy(*new_str+noffset, str+offset, matches[0].rm_so);
        noffset += matches[0].rm_so;
      }

      /* Go and append the replacement string */
      for (i=0; i<rlen; i++) {
        switch (replacement[i]) {
          case '\\':
            if (i+1 < rlen) {
              if (nlen-noffset < 1) {
                nlen += nlen + 1;
                *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
              }
              *new_str[noffset++] = replacement[++i];
            }
            else {
              gk_free((void **)new_str, LTERM);
              *new_str = gk_strdup("Error in replacement string. Missing character following '\'.");
              regfree(&re);
              return 0;
            }
            break;

          case '$':
            if (i+1 < rlen) {
              j = (int)(replacement[++i] - '0');
              if (j < 0 || j > 9) {
                gk_free((void **)new_str, LTERM);
                *new_str = gk_strdup("Error in captured subexpression specification.");
                regfree(&re);
                return 0;
              }

              if (nlen-noffset < matches[j].rm_eo-matches[j].rm_so) {
                nlen += nlen + (matches[j].rm_eo-matches[j].rm_so);
                *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
              }

              strncpy(*new_str+noffset, str+offset+matches[j].rm_so, matches[j].rm_eo);
              noffset += matches[j].rm_eo-matches[j].rm_so;
            }
            else {
              gk_free((void **)new_str, LTERM);
              *new_str = gk_strdup("Error in replacement string. Missing subexpression number folloing '$'.");
              regfree(&re);
              return 0;
            }
            break;

          default:
            if (nlen-noffset < 1) {
              nlen += nlen + 1;
              *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
            }
            (*new_str)[noffset++] = replacement[i];
        }
      }

      /* Update the offset of str for the next match */
      offset += matches[0].rm_eo;

      if (!global) {
        /* Copy the right portion of the string if no 'g' option */
        if (nlen-noffset < len-offset) {
          nlen += (len-offset) - (nlen-noffset);
          *new_str = (char *)gk_realloc(*new_str, (nlen+1)*sizeof(char), "gk_strstr_replace: new_str");
        }
        strcpy(*new_str+noffset, str+offset);
        noffset += (len-offset);
      }
    }
  } while (global);

  (*new_str)[noffset] = '\0';

  regfree(&re);
  return nmatches + 1;

}



/************************************************************************/
/*! \brief Prunes characters from the end of the string.

This function removes any trailing characters that are included in the
\c rmlist. The trimming stops at the last character (i.e., first character 
from the end) that is not in \c rmlist.  
This function can be used to removed trailing spaces, newlines, etc.
This is a distructive operation as it modifies the string.

\param str is the string that will be trimmed.
\param rmlist contains the set of characters that will be removed.
\returns A pointer to \c str itself.
\sa gk_strhprune()
*/
/*************************************************************************/
char *gk_strtprune(char *str, char *rmlist)
{
  gk_idx_t i, j;
  size_t len;

  len = strlen(rmlist);

  for (i=strlen(str)-1; i>=0; i--) {
    for (j=0; j<len; j++) {
      if (str[i] == rmlist[j])
        break;
    }
    if (j == len)
      break;
  }

  str[i+1] = '\0';

  return str;
}


/************************************************************************/
/*! \brief Prunes characters from the beginning of the string.

This function removes any starting characters that are included in the
\c rmlist. The trimming stops at the first character that is not in 
\c rmlist.
This function can be used to removed leading spaces, tabs, etc.
This is a distructive operation as it modifies the string.

\param str is the string that will be trimmed.
\param rmlist contains the set of characters that will be removed.
\returns A pointer to \c str itself.
\sa gk_strtprune()
*/
/*************************************************************************/
char *gk_strhprune(char *str, char *rmlist)
{
  gk_idx_t i, j;
  size_t len;

  len = strlen(rmlist);

  for (i=0; str[i]; i++) {
    for (j=0; j<len; j++) {
      if (str[i] == rmlist[j])
        break;
    }
    if (j == len)
      break;
  }

  if (i>0) { /* If something needs to be removed */
    for (j=0; str[i]; i++, j++)
      str[j] = str[i];
    str[j] = '\0';
  }

  return str;
}


/************************************************************************/
/*! \brief Converts a string to upper case.

This function converts a string to upper case. This operation modifies the 
string itself.

\param str is the string whose case will be changed.
\returns A pointer to \c str itself.
\sa gk_strtolower()
*/
/*************************************************************************/
char *gk_strtoupper(char *str)
{
  int i;

  for (i=0; str[i]!='\0'; str[i]=toupper(str[i]), i++); 
  return str;
}


/************************************************************************/
/*! \brief Converts a string to lower case.

This function converts a string to lower case. This operation modifies the 
string itself.

\param str is the string whose case will be changed.
\returns A pointer to \c str itself.
\sa gk_strtoupper()
*/
/*************************************************************************/
char *gk_strtolower(char *str)
{
  int i;

  for (i=0; str[i]!='\0'; str[i]=tolower(str[i]), i++); 
  return str;
}


/************************************************************************/
/*! \brief Duplicates a string

This function is a replacement for C's standard <em>strdup()</em> function.
The key differences between the two are that gk_strdup():
  - uses the dynamic memory allocation routines of \e GKlib. 
  - it correctly handles NULL input strings.

The string that is returned must be freed by gk_free().

\param orgstr is the string that will be duplicated.
\returns A pointer to the newly created string.
\sa gk_free()
*/
/*************************************************************************/
char *gk_strdup(char *orgstr)
{
  int len;
  char *str=NULL;

  if (orgstr != NULL) {
    len = strlen(orgstr)+1;
    str = gk_malloc(len*sizeof(char), "gk_strdup: str");
    strcpy(str, orgstr);
  }

  return str;
}


/************************************************************************/
/*! \brief Case insensitive string comparison.

This function compares two strings for equality by ignoring the case of the
strings. 

\warning This function is \b not equivalent to a case-insensitive 
         <em>strcmp()</em> function, as it does not return ordering 
         information.

\todo Remove the above warning.

\param s1 is the first string to be compared.
\param s2 is the second string to be compared.
\retval 1 if the strings are identical,
\retval 0 otherwise.
*/
/*************************************************************************/
int gk_strcasecmp(char *s1, char *s2)
{
  int i=0;

  if (strlen(s1) != strlen(s2))
    return 0;

  while (s1[i] != '\0') {
    if (tolower(s1[i]) != tolower(s2[i]))
      return 0;
    i++;
  }

  return 1;
}


/************************************************************************/
/*! \brief Compare two strings in revere order

This function is similar to strcmp but it performs the comparison as
if the two strings were reversed.

\param s1 is the first string to be compared.
\param s2 is the second string to be compared.
\retval -1, 0, 1, if the s1 < s2, s1 == s2, or s1 > s2.
*/
/*************************************************************************/
int gk_strrcmp(char *s1, char *s2)
{
  int i1 = strlen(s1)-1;
  int i2 = strlen(s2)-1;

  while ((i1 >= 0) && (i2 >= 0)) {
    if (s1[i1] != s2[i2])
      return (s1[i1] - s2[i2]);
    i1--;
    i2--;
  }

  /* i1 == -1 and/or i2 == -1 */

  if (i1 < i2)
    return -1;
  if (i1 > i2)
    return 1;
  return 0;
}



/************************************************************************/
/*! \brief Converts a time_t time into a string 

This function takes a time_t-specified time and returns a string-formated
representation of the corresponding time. The format of the string is
<em>mm/dd/yyyy hh:mm:ss</em>, in which the hours are in military time.

\param time is the time to be converted.
\return It returns a pointer to a statically allocated string that is 
        over-written in successive calls of this function. If the 
        conversion failed, it returns NULL.

*/
/*************************************************************************/
char *gk_time2str(time_t time)
{
  static char datestr[128];
  struct tm *tm;

  tm = localtime(&time);

  if (strftime(datestr, 128, "%m/%d/%Y %H:%M:%S", tm) == 0)
    return NULL;
  else
    return datestr;
}



#if !defined(WIN32) && !defined(__MINGW32__)
/************************************************************************/
/*! \brief Converts a date/time string into its equivalent time_t value

This function takes date and/or time specification and converts it in
the equivalent time_t representation. The conversion is done using the
strptime() function. The format that gk_str2time() understands is
<em>mm/dd/yyyy hh:mm:ss</em>, in which the hours are in military time.

\param str is the date/time string to be converted.
\return If the conversion was successful it returns the time, otherwise 
        it returns -1.
*/
/*************************************************************************/
time_t gk_str2time(char *str)
{
  struct tm time;
  time_t rtime;

  memset(&time, '\0', sizeof(time));
  
  if (strptime(str, "%m/%d/%Y %H:%M:%S", &time) == NULL)
    return -1;

  rtime = mktime(&time);
  return (rtime < 0 ? 0 : rtime);
}
#endif


/*************************************************************************
* This function returns the ID of a particular string based on the 
* supplied StringMap array
**************************************************************************/
int gk_GetStringID(gk_StringMap_t *strmap, char *key)
{
  int i;

  for (i=0; strmap[i].name; i++) {
    if (gk_strcasecmp(key, strmap[i].name))
      return strmap[i].id;
  }

  return -1;
}
