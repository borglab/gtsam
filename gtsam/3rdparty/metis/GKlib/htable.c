/*
 * Copyright 2004, Regents of the University of Minnesota
 *
 * This file contains routines for manipulating a direct-access hash table
 *
 * Started 3/22/04
 * George
 *
 */

#include <GKlib.h>

/******************************************************************************
* This function creates the hash-table
*******************************************************************************/
gk_HTable_t *HTable_Create(int nelements)
{
  gk_HTable_t *htable;

  htable            = gk_malloc(sizeof(gk_HTable_t), "HTable_Create: htable");
  htable->harray    = gk_ikvmalloc(nelements, "HTable_Create: harray");
  htable->nelements = nelements;

  HTable_Reset(htable);

  return htable;
}


/******************************************************************************
* This function resets the data-structures associated with the hash-table
*******************************************************************************/
void HTable_Reset(gk_HTable_t *htable)
{
  int i;

  for (i=0; i<htable->nelements; i++)
    htable->harray[i].key = HTABLE_EMPTY;
  htable->htsize = 0;

}

/******************************************************************************
* This function resizes the hash-table
*******************************************************************************/
void HTable_Resize(gk_HTable_t *htable, int nelements)
{
  int i, old_nelements;
  gk_ikv_t *old_harray;

  old_nelements = htable->nelements;
  old_harray = htable->harray;

  /* prepare larger hash */
  htable->nelements = nelements;
  htable->htsize = 0;
  htable->harray = gk_ikvmalloc(nelements, "HTable_Resize: harray");
  for (i=0; i<nelements; i++)
    htable->harray[i].key = HTABLE_EMPTY;

  /* reassign the values */
  for (i=0; i<old_nelements; i++)
    if (old_harray[i].key != HTABLE_EMPTY)
       HTable_Insert(htable, old_harray[i].key, old_harray[i].val);

  /* remove old harray */
  gk_free((void **)&old_harray, LTERM);
}


/******************************************************************************
* This function inserts a key-value pair in the array
*******************************************************************************/
void HTable_Insert(gk_HTable_t *htable, int key, int val)
{
  int i, first;

  if (htable->htsize > htable->nelements/2)
    HTable_Resize(htable, 2*htable->nelements);

  first = HTable_HFunction(htable->nelements, key);

  for (i=first; i<htable->nelements; i++) {
    if (htable->harray[i].key == HTABLE_EMPTY || htable->harray[i].key == HTABLE_DELETED) {
      htable->harray[i].key = key;
      htable->harray[i].val = val;
      htable->htsize++;
      return;
    }
  }

  for (i=0; i<first; i++) {
    if (htable->harray[i].key == HTABLE_EMPTY || htable->harray[i].key == HTABLE_DELETED) {
      htable->harray[i].key = key;
      htable->harray[i].val = val;
      htable->htsize++;
      return;
    }
  }

}


/******************************************************************************
* This function deletes key from the htable
*******************************************************************************/
void HTable_Delete(gk_HTable_t *htable, int key)
{
  int i, first;

  first = HTable_HFunction(htable->nelements, key);

  for (i=first; i<htable->nelements; i++) {
    if (htable->harray[i].key == key) {
      htable->harray[i].key = HTABLE_DELETED;
      htable->htsize--;
      return;
    }
  }

  for (i=0; i<first; i++) {
    if (htable->harray[i].key == key) {
      htable->harray[i].key = HTABLE_DELETED;
      htable->htsize--;
      return;
    }
  }

}


/******************************************************************************
* This function returns the data associated with the key in the hastable
*******************************************************************************/
int HTable_Search(gk_HTable_t *htable, int key)
{
  int i, first;

  first = HTable_HFunction(htable->nelements, key);

  for (i=first; i<htable->nelements; i++) {
    if (htable->harray[i].key == key) 
      return htable->harray[i].val;
    else if (htable->harray[i].key == HTABLE_EMPTY)
      return -1;
  }

  for (i=0; i<first; i++) {
    if (htable->harray[i].key == key) 
      return htable->harray[i].val;
    else if (htable->harray[i].key == HTABLE_EMPTY)
      return -1;
  }

  return -1;
}


/******************************************************************************
* This function returns the next key/val
*******************************************************************************/
int HTable_GetNext(gk_HTable_t *htable, int key, int *r_val, int type)
{
  int i;
  static int first, last;

  if (type == HTABLE_FIRST)
    first = last = HTable_HFunction(htable->nelements, key);

  if (first > last) {
    for (i=first; i<htable->nelements; i++) {
      if (htable->harray[i].key == key) {
        *r_val = htable->harray[i].val;
        first = i+1;
        return 1;
      }
      else if (htable->harray[i].key == HTABLE_EMPTY)
        return -1;
    }
    first = 0;
  }

  for (i=first; i<last; i++) {
    if (htable->harray[i].key == key) {
      *r_val = htable->harray[i].val;
      first = i+1;
      return 1;
    }
    else if (htable->harray[i].key == HTABLE_EMPTY)
      return -1;
  }

  return -1;
}


/******************************************************************************
* This function returns the data associated with the key in the hastable
*******************************************************************************/
int HTable_SearchAndDelete(gk_HTable_t *htable, int key)
{
  int i, first;

  first = HTable_HFunction(htable->nelements, key);

  for (i=first; i<htable->nelements; i++) {
    if (htable->harray[i].key == key) {
      htable->harray[i].key = HTABLE_DELETED;
      htable->htsize--;
      return htable->harray[i].val;
    }
    else if (htable->harray[i].key == HTABLE_EMPTY)
      gk_errexit(SIGERR, "HTable_SearchAndDelete: Failed to find the key!\n");
  }

  for (i=0; i<first; i++) {
    if (htable->harray[i].key == key) {
      htable->harray[i].key = HTABLE_DELETED;
      htable->htsize--;
      return htable->harray[i].val;
    }
    else if (htable->harray[i].key == HTABLE_EMPTY)
      gk_errexit(SIGERR, "HTable_SearchAndDelete: Failed to find the key!\n");
  }

  return -1;

}



/******************************************************************************
* This function destroys the data structures associated with the hash-table
*******************************************************************************/
void HTable_Destroy(gk_HTable_t *htable)
{
  gk_free((void **)&htable->harray, &htable, LTERM);
}


/******************************************************************************
* This is the hash-function. Based on multiplication
*******************************************************************************/
int HTable_HFunction(int nelements, int key)
{
  return (int)(key%nelements);
}
