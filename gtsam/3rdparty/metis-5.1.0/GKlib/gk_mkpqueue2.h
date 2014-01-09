/*!
\file  gk_mkpqueue2.h
\brief Templates for priority queues that do not utilize locators and as such
       they can use different types of values.

\date   Started 4/09/07
\author George
\version\verbatim $Id: gk_mkpqueue2.h 13005 2012-10-23 22:34:36Z karypis $ \endverbatim
*/


#ifndef _GK_MKPQUEUE2_H
#define _GK_MKPQUEUE2_H


#define GK_MKPQUEUE2(FPRFX, PQT, KT, VT, KMALLOC, VMALLOC, KMAX, KEY_LT)\
/*************************************************************************/\
/*! This function creates and initializes a priority queue */\
/**************************************************************************/\
PQT *FPRFX ## Create2(ssize_t maxnodes)\
{\
  PQT *queue; \
\
  if ((queue = (PQT *)gk_malloc(sizeof(PQT), "gk_pqCreate2: queue")) != NULL) {\
    memset(queue, 0, sizeof(PQT));\
    queue->nnodes   = 0;\
    queue->maxnodes = maxnodes;\
    queue->keys     = KMALLOC(maxnodes, "gk_pqCreate2: keys");\
    queue->vals     = VMALLOC(maxnodes, "gk_pqCreate2: vals");\
\
    if (queue->keys == NULL || queue->vals == NULL)\
      gk_free((void **)&queue->keys, &queue->vals, &queue, LTERM);\
  }\
\
  return queue;\
}\
\
\
/*************************************************************************/\
/*! This function resets the priority queue */\
/**************************************************************************/\
void FPRFX ## Reset2(PQT *queue)\
{\
  queue->nnodes = 0;\
}\
\
\
/*************************************************************************/\
/*! This function frees the internal datastructures of the priority queue */\
/**************************************************************************/\
void FPRFX ## Destroy2(PQT **r_queue)\
{\
  PQT *queue = *r_queue; \
  if (queue == NULL) return;\
  gk_free((void **)&queue->keys, &queue->vals, &queue, LTERM);\
  *r_queue = NULL;\
}\
\
\
/*************************************************************************/\
/*! This function returns the length of the queue */\
/**************************************************************************/\
size_t FPRFX ## Length2(PQT *queue)\
{\
  return queue->nnodes;\
}\
\
\
/*************************************************************************/\
/*! This function adds an item in the priority queue. */\
/**************************************************************************/\
int FPRFX ## Insert2(PQT *queue, VT val, KT key)\
{\
  ssize_t i, j;\
  KT *keys=queue->keys;\
  VT *vals=queue->vals;\
\
  ASSERT2(FPRFX ## CheckHeap2(queue));\
\
  if (queue->nnodes == queue->maxnodes) \
    return 0;\
\
  ASSERT2(FPRFX ## CheckHeap2(queue));\
\
  i = queue->nnodes++;\
  while (i > 0) {\
    j = (i-1)>>1;\
    if (KEY_LT(key, keys[j])) {\
      keys[i] = keys[j];\
      vals[i] = vals[j];\
      i = j;\
    }\
    else\
      break;\
  }\
  ASSERT(i >= 0);\
  keys[i] = key;\
  vals[i] = val;\
\
  ASSERT2(FPRFX ## CheckHeap2(queue));\
\
  return 1;\
}\
\
\
/*************************************************************************/\
/*! This function returns the item at the top of the queue and removes\
    it from the priority queue */\
/**************************************************************************/\
int FPRFX ## GetTop2(PQT *queue, VT *r_val)\
{\
  ssize_t i, j;\
  KT key, *keys=queue->keys;\
  VT val, *vals=queue->vals;\
\
  ASSERT2(FPRFX ## CheckHeap2(queue));\
\
  if (queue->nnodes == 0)\
    return 0;\
\
  queue->nnodes--;\
\
  *r_val = vals[0];\
\
  if ((i = queue->nnodes) > 0) {\
    key = keys[i];\
    val = vals[i];\
    i = 0;\
    while ((j=2*i+1) < queue->nnodes) {\
      if (KEY_LT(keys[j], key)) {\
        if (j+1 < queue->nnodes && KEY_LT(keys[j+1], keys[j]))\
          j = j+1;\
        keys[i] = keys[j];\
        vals[i] = vals[j];\
        i = j;\
      }\
      else if (j+1 < queue->nnodes && KEY_LT(keys[j+1], key)) {\
        j = j+1;\
        keys[i] = keys[j];\
        vals[i] = vals[j];\
        i = j;\
      }\
      else\
        break;\
    }\
\
    keys[i] = key;\
    vals[i] = val;\
  }\
\
  ASSERT2(FPRFX ## CheckHeap2(queue));\
\
  return 1;\
}\
\
\
/*************************************************************************/\
/*! This function returns the item at the top of the queue. The item is not\
    deleted from the queue. */\
/**************************************************************************/\
int FPRFX ## SeeTopVal2(PQT *queue, VT *r_val)\
{\
  if (queue->nnodes == 0) \
    return 0;\
\
  *r_val = queue->vals[0];\
\
  return 1;\
}\
\
\
/*************************************************************************/\
/*! This function returns the key of the top item. The item is not\
    deleted from the queue. */\
/**************************************************************************/\
KT FPRFX ## SeeTopKey2(PQT *queue)\
{\
  return (queue->nnodes == 0 ? KMAX : queue->keys[0]);\
}\
\
\
/*************************************************************************/\
/*! This functions checks the consistency of the heap */\
/**************************************************************************/\
int FPRFX ## CheckHeap2(PQT *queue)\
{\
  ssize_t i;\
  KT *keys=queue->keys;\
\
  if (queue->nnodes == 0)\
    return 1;\
\
  for (i=1; i<queue->nnodes; i++) {\
    ASSERT(!KEY_LT(keys[i], keys[(i-1)/2]));\
  }\
  for (i=1; i<queue->nnodes; i++)\
    ASSERT(!KEY_LT(keys[i], keys[0]));\
\
  return 1;\
}\


#define GK_MKPQUEUE2_PROTO(FPRFX, PQT, KT, VT)\
  PQT *  FPRFX ## Create2(ssize_t maxnodes);\
  void   FPRFX ## Reset2(PQT *queue);\
  void   FPRFX ## Destroy2(PQT **r_queue);\
  size_t FPRFX ## Length2(PQT *queue);\
  int    FPRFX ## Insert2(PQT *queue, VT node, KT key);\
  int    FPRFX ## GetTop2(PQT *queue, VT *r_val);\
  int    FPRFX ## SeeTopVal2(PQT *queue, VT *r_val);\
  KT     FPRFX ## SeeTopKey2(PQT *queue);\
  int    FPRFX ## CheckHeap2(PQT *queue);\


#endif
