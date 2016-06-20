/*!
\file  
\brief Templates for portable random number generation

\date   Started 5/17/07
\author George
\version\verbatim $Id: gk_mkrandom.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#ifndef _GK_MKRANDOM_H
#define _GK_MKRANDOM_H

/*************************************************************************/\
/*! The generator for the rand() related routines.  \
   \params RNGT  the datatype that defines the range of values over which\
                 random numbers will be generated\
   \params VALT  the datatype that defines the contents of the array to \
                 be permuted by randArrayPermute() \
   \params FPRFX the function prefix \
*/\
/**************************************************************************/\
#define GK_MKRANDOM(FPRFX, RNGT, VALT)\
/*************************************************************************/\
/*! Initializes the generator */ \
/**************************************************************************/\
void FPRFX ## srand(RNGT seed) \
{\
  gk_randinit((uint64_t) seed);\
}\
\
\
/*************************************************************************/\
/*! Returns a random number */ \
/**************************************************************************/\
RNGT FPRFX ## rand() \
{\
  if (sizeof(RNGT) <= sizeof(int32_t)) \
    return (RNGT)gk_randint32(); \
  else \
    return (RNGT)gk_randint64(); \
}\
\
\
/*************************************************************************/\
/*! Returns a random number between [0, max) */ \
/**************************************************************************/\
RNGT FPRFX ## randInRange(RNGT max) \
{\
  return (RNGT)((FPRFX ## rand())%max); \
}\
\
\
/*************************************************************************/\
/*! Randomly permutes the elements of an array p[]. \
    flag == 1, p[i] = i prior to permutation, \
    flag == 0, p[] is not initialized. */\
/**************************************************************************/\
void FPRFX ## randArrayPermute(RNGT n, VALT *p, RNGT nshuffles, int flag)\
{\
  RNGT i, u, v;\
  VALT tmp;\
\
  if (flag == 1) {\
    for (i=0; i<n; i++)\
      p[i] = (VALT)i;\
  }\
\
  if (n < 10) {\
    for (i=0; i<n; i++) {\
      v = FPRFX ## randInRange(n);\
      u = FPRFX ## randInRange(n);\
      gk_SWAP(p[v], p[u], tmp);\
    }\
  }\
  else {\
    for (i=0; i<nshuffles; i++) {\
      v = FPRFX ## randInRange(n-3);\
      u = FPRFX ## randInRange(n-3);\
      /*gk_SWAP(p[v+0], p[u+0], tmp);*/\
      /*gk_SWAP(p[v+1], p[u+1], tmp);*/\
      /*gk_SWAP(p[v+2], p[u+2], tmp);*/\
      /*gk_SWAP(p[v+3], p[u+3], tmp);*/\
      gk_SWAP(p[v+0], p[u+2], tmp);\
      gk_SWAP(p[v+1], p[u+3], tmp);\
      gk_SWAP(p[v+2], p[u+0], tmp);\
      gk_SWAP(p[v+3], p[u+1], tmp);\
    }\
  }\
}\
\
\
/*************************************************************************/\
/*! Randomly permutes the elements of an array p[]. \
    flag == 1, p[i] = i prior to permutation, \
    flag == 0, p[] is not initialized. */\
/**************************************************************************/\
void FPRFX ## randArrayPermuteFine(RNGT n, VALT *p, int flag)\
{\
  RNGT i, v;\
  VALT tmp;\
\
  if (flag == 1) {\
    for (i=0; i<n; i++)\
      p[i] = (VALT)i;\
  }\
\
  for (i=0; i<n; i++) {\
    v = FPRFX ## randInRange(n);\
    gk_SWAP(p[i], p[v], tmp);\
  }\
}\


#define GK_MKRANDOM_PROTO(FPRFX, RNGT, VALT)\
  void FPRFX ## srand(RNGT seed); \
  RNGT FPRFX ## rand(); \
  RNGT FPRFX ## randInRange(RNGT max); \
  void FPRFX ## randArrayPermute(RNGT n, VALT *p, RNGT nshuffles, int flag);\
  void FPRFX ## randArrayPermuteFine(RNGT n, VALT *p, int flag);\


#endif
