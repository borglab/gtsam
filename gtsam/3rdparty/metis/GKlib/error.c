/*!
\file  error.c
\brief Various error-handling functions

This file contains functions dealing with error reporting and termination

\author George
\date 1/1/2007
\version\verbatim $Id: error.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#define _GK_ERROR_C_  /* this is needed to properly declare the gk_jub* variables
                         as an extern function in GKlib.h */

#include <GKlib.h>


/* These are the jmp_buf for the graceful exit in case of severe errors.
   Multiple buffers are defined to allow for recursive invokation. */
#define MAX_JBUFS 128
__thread int gk_cur_jbufs=-1;
__thread jmp_buf gk_jbufs[MAX_JBUFS];
__thread jmp_buf gk_jbuf;

typedef void (*gksighandler_t)(int);

/* These are the holders of the old singal handlers for the trapped signals */
static __thread gksighandler_t old_SIGMEM_handler;  /* Custom signal */
static __thread gksighandler_t old_SIGERR_handler;  /* Custom signal */
static __thread gksighandler_t old_SIGMEM_handlers[MAX_JBUFS];  /* Custom signal */
static __thread gksighandler_t old_SIGERR_handlers[MAX_JBUFS];  /* Custom signal */

/* The following is used to control if the gk_errexit() will actually abort or not.
   There is always a single copy of this variable */
static int gk_exit_on_error = 1;


/*************************************************************************/
/*! This function sets the gk_exit_on_error variable 
 */
/*************************************************************************/
void gk_set_exit_on_error(int value)
{
  gk_exit_on_error = value;
}



/*************************************************************************/
/*! This function prints an error message and exits  
 */
/*************************************************************************/
void errexit(char *f_str,...)
{
  va_list argp;

  va_start(argp, f_str);
  vfprintf(stderr, f_str, argp);
  va_end(argp);

  if (strlen(f_str) == 0 || f_str[strlen(f_str)-1] != '\n')
        fprintf(stderr,"\n");
  fflush(stderr);

  if (gk_exit_on_error)
    exit(-2);

  /* abort(); */
}


/*************************************************************************/
/*! This function prints an error message and raises a signum signal
 */
/*************************************************************************/
void gk_errexit(int signum, char *f_str,...)
{
  va_list argp;

  va_start(argp, f_str);
  vfprintf(stderr, f_str, argp);
  va_end(argp);

  fprintf(stderr,"\n");
  fflush(stderr);

  if (gk_exit_on_error)
    raise(signum);
}


/***************************************************************************/
/*! This function sets a number of signal handlers and sets the return point 
    of a longjmp
*/
/***************************************************************************/
int gk_sigtrap() 
{
  if (gk_cur_jbufs+1 >= MAX_JBUFS)
    return 0;

  gk_cur_jbufs++;

  old_SIGMEM_handlers[gk_cur_jbufs]  = signal(SIGMEM,  gk_sigthrow);
  old_SIGERR_handlers[gk_cur_jbufs]  = signal(SIGERR,  gk_sigthrow);

  return 1;
}
  

/***************************************************************************/
/*! This function sets the handlers for the signals to their default handlers
 */
/***************************************************************************/
int gk_siguntrap() 
{
  if (gk_cur_jbufs == -1)
    return 0;

  signal(SIGMEM,  old_SIGMEM_handlers[gk_cur_jbufs]);
  signal(SIGERR,  old_SIGERR_handlers[gk_cur_jbufs]);

  gk_cur_jbufs--;

  return 1;
}
  

/*************************************************************************/
/*! This function is the custome signal handler, which all it does is to
    perform a longjump to the most recent saved environment 
 */
/*************************************************************************/
void gk_sigthrow(int signum)
{
  longjmp(gk_jbufs[gk_cur_jbufs], signum);
}
  

/***************************************************************************
* This function sets a number of signal handlers and sets the return point 
* of a longjmp
****************************************************************************/
void gk_SetSignalHandlers() 
{
  old_SIGMEM_handler = signal(SIGMEM,  gk_NonLocalExit_Handler);
  old_SIGERR_handler = signal(SIGERR,  gk_NonLocalExit_Handler);
}
  

/***************************************************************************
* This function sets the handlers for the signals to their default handlers
****************************************************************************/
void gk_UnsetSignalHandlers() 
{
  signal(SIGMEM,  old_SIGMEM_handler);
  signal(SIGERR,  old_SIGERR_handler);
}
  

/*************************************************************************
* This function is the handler for SIGUSR1 that implements the cleaning up 
* process prior to a non-local exit.
**************************************************************************/
void gk_NonLocalExit_Handler(int signum)
{
  longjmp(gk_jbuf, signum);
}
  

/*************************************************************************/
/*! \brief Thread-safe implementation of strerror() */
/**************************************************************************/
char *gk_strerror(int errnum)
{
#if defined(WIN32) || defined(__MINGW32__)
  return strerror(errnum);
#else 
#ifndef SUNOS
  static __thread char buf[1024];

  strerror_r(errnum, buf, 1024);

  buf[1023] = '\0';
  return buf;
#else
  return strerror(errnum);
#endif
#endif
}



/*************************************************************************
* This function prints a backtrace of calling functions
**************************************************************************/
void PrintBackTrace()
{
#ifdef HAVE_EXECINFO_H
  void *array[10];
  int i, size;
  char **strings;

  size = backtrace(array, 10);
  strings = backtrace_symbols(array, size);
  
  printf("Obtained %d stack frames.\n", size);
  for (i=0; i<size; i++) {
    printf("%s\n", strings[i]);
  }
  free(strings);
#endif
}
