/*
 * struct.h
 *
 * This file contains data structures for the various programs of METIS.
 *
 * Started 8/9/02
 * George
 *
 * $Id: struct.h 13900 2013-03-24 15:27:07Z karypis $
 */

#ifndef _STRUCTBIN_H_
#define _STRUCTBIN_H_


/*************************************************************************/
/*! This data structure stores the various command line arguments */
/*************************************************************************/
typedef struct {
  idx_t ptype;
  idx_t objtype;
  idx_t ctype;
  idx_t iptype;
  idx_t rtype;

  idx_t no2hop;
  idx_t minconn;
  idx_t contig;

  idx_t nooutput;

  idx_t balance;
  idx_t ncuts;
  idx_t niter;

  idx_t gtype;
  idx_t ncommon;

  idx_t seed;
  idx_t dbglvl;

  idx_t nparts;

  idx_t nseps;
  idx_t ufactor;
  idx_t pfactor;
  idx_t compress;
  idx_t ccorder;

  char *filename;
  char *outfile;
  char *xyzfile;
  char *tpwgtsfile;
  char *ubvecstr;

  idx_t wgtflag;
  idx_t numflag;
  real_t *tpwgts;
  real_t *ubvec;

  real_t iotimer;
  real_t parttimer;
  real_t reporttimer;

  size_t maxmemory;
} params_t;


#endif 
