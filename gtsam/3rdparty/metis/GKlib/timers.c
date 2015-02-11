/*!
\file  timers.c
\brief Various timing functions 

\date   Started 4/12/2007
\author George
\version\verbatim $Id: timers.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#include <GKlib.h>




/*************************************************************************
* This function returns the CPU seconds
**************************************************************************/
double gk_WClockSeconds(void)
{
#ifdef __GNUC__
  struct timeval ctime;

  gettimeofday(&ctime, NULL);

  return (double)ctime.tv_sec + (double).000001*ctime.tv_usec;
#else
  return (double)time(NULL);
#endif
}


/*************************************************************************
* This function returns the CPU seconds
**************************************************************************/
double gk_CPUSeconds(void)
{
//#ifdef __OPENMP__
#ifdef __OPENMPXXXX__
  return omp_get_wtime();
#else
  #if defined(WIN32) || defined(__MINGW32__)
    return((double) clock()/CLOCKS_PER_SEC);
  #else
    struct rusage r;

    getrusage(RUSAGE_SELF, &r);
    return ((r.ru_utime.tv_sec + r.ru_stime.tv_sec) + 1.0e-6*(r.ru_utime.tv_usec + r.ru_stime.tv_usec));
  #endif
#endif
}

