/*!
\file  fs.c
\brief Various file-system functions.

This file contains various functions that deal with interfacing with 
the filesystem in a portable way.

\date Started 4/10/95
\author George
\version\verbatim $Id: fs.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#include <GKlib.h>



/*************************************************************************
* This function checks if a file exists
**************************************************************************/
int gk_fexists(char *fname)
{
  struct stat status;

  if (stat(fname, &status) == -1)
    return 0;

  return S_ISREG(status.st_mode);
}


/*************************************************************************
* This function checks if a directory exists
**************************************************************************/
int gk_dexists(char *dirname)
{
  struct stat status;

  if (stat(dirname, &status) == -1)
    return 0;

  return S_ISDIR(status.st_mode);
}


/*************************************************************************/
/*! \brief Returns the size of the file in bytes

This function returns the size of a file as a 64 bit integer. If there 
were any errors in stat'ing the file, -1 is returned.
\note That due to the -1 return code, the maximum file size is limited to
      63 bits (which I guess is okay for now).
*/
/**************************************************************************/
intmax_t gk_getfsize(char *filename)
{
  struct stat status;

  if (stat(filename, &status) == -1)
    return -1;

  return (intmax_t)(status.st_size);
}


/*************************************************************************/
/*! This function gets some basic statistics about the file. 
    \param fname is the name of the file
    \param r_nlines is the number of lines in the file. If it is NULL,
           this information is not returned.
    \param r_ntokens is the number of tokens in the file. If it is NULL,
           this information is not returned.
    \param r_max_nlntokens is the maximum number of tokens in any line
           in the file. If it is NULL this information is not returned.
    \param r_nbytes is the number of bytes in the file. If it is NULL,
           this information is not returned.
*/
/*************************************************************************/
void gk_getfilestats(char *fname, size_t *r_nlines, size_t *r_ntokens, 
        size_t *r_max_nlntokens, size_t *r_nbytes)
{
  size_t nlines=0, ntokens=0, max_nlntokens=0, nbytes=0, oldntokens=0, nread;
  int intoken=0;
  char buffer[2049], *cptr;
  FILE *fpin;

  fpin = gk_fopen(fname, "r", "gk_GetFileStats");

  while (!feof(fpin)) {
    nread = fread(buffer, sizeof(char), 2048, fpin);
    nbytes += nread;

    buffer[nread] = '\0';  /* There is space for this one */
    for (cptr=buffer; *cptr!='\0'; cptr++) {
      if (*cptr == '\n') {
        nlines++;
        ntokens += intoken;
        intoken = 0;
        if (max_nlntokens < ntokens-oldntokens)
          max_nlntokens = ntokens-oldntokens;
        oldntokens = ntokens;
      }
      else if (*cptr == ' ' || *cptr == '\t') {
        ntokens += intoken;
        intoken = 0;
      }
      else {
        intoken = 1;
      }
    }
  }
  ntokens += intoken;
  if (max_nlntokens < ntokens-oldntokens)
    max_nlntokens = ntokens-oldntokens;

  gk_fclose(fpin);

  if (r_nlines != NULL)
    *r_nlines  = nlines;
  if (r_ntokens != NULL)
    *r_ntokens = ntokens;
  if (r_max_nlntokens != NULL)
    *r_max_nlntokens = max_nlntokens;
  if (r_nbytes != NULL)
    *r_nbytes  = nbytes;
}


/*************************************************************************
* This function takes in a potentially full path specification of a file
* and just returns a string containing just the basename of the file.
* The basename is derived from the actual filename by stripping the last
* .ext part.
**************************************************************************/
char *gk_getbasename(char *path)
{
  char *startptr, *endptr;
  char *basename;

  if ((startptr = strrchr(path, '/')) == NULL) 
    startptr = path;
  else 
    startptr = startptr+1;

  basename = gk_strdup(startptr);

  if ((endptr = strrchr(basename, '.')) != NULL) 
    *endptr = '\0';

  return basename;
}

/*************************************************************************
* This function takes in a potentially full path specification of a file
* and just returns a string corresponding to its file extension. The
* extension of a file is considered to be the string right after the 
* last '.' character.
**************************************************************************/
char *gk_getextname(char *path)
{
  char *startptr;

  if ((startptr = strrchr(path, '.')) == NULL) 
    return gk_strdup(path);
  else 
    return gk_strdup(startptr+1);
}

/*************************************************************************
* This function takes in a potentially full path specification of a file
* and just returns a string containing just the filename.
**************************************************************************/
char *gk_getfilename(char *path)
{
  char *startptr;

  if ((startptr = strrchr(path, '/')) == NULL) 
    return gk_strdup(path);
  else 
    return gk_strdup(startptr+1);
}

/*************************************************************************
* This function takes in a potentially full path specification of a file
* and extracts the directory path component if it exists, otherwise it
* returns "./" as the path. The memory for it is dynamically allocated.
**************************************************************************/
char *getpathname(char *path)
{
  char *endptr, *tmp;

  if ((endptr = strrchr(path, '/')) == NULL) {
    return gk_strdup(".");
  }
  else  {
    tmp = gk_strdup(path);
    *(strrchr(tmp, '/')) = '\0';
    return tmp;
  }
}



/*************************************************************************
* This function creates a path
**************************************************************************/
int gk_mkpath(char *pathname)
{
  char tmp[2048];

  sprintf(tmp, "mkdir -p %s", pathname);
  return system(tmp);
}


/*************************************************************************
* This function deletes a directory tree and all of its contents
**************************************************************************/
int gk_rmpath(char *pathname)
{
  char tmp[2048];

  sprintf(tmp, "rm -r %s", pathname);
  return system(tmp);
}
