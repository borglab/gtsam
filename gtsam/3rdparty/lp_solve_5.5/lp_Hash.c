
#include <stdlib.h>
#include <string.h>
#include "lp_lib.h"
#include "lp_utils.h"
#include "lp_report.h"
#include "lp_Hash.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


#define HASH_START_SIZE  5000  /* Hash table size for row and column name storage */
#define NUMHASHPRIMES      45

STATIC hashtable *create_hash_table(int size, int base)
{
  int i;
  int HashPrimes[ ] = {
             29,     229,     883,    1671,    2791,    4801,    8629,   10007,
          15289,   25303,   34843,   65269,   99709,  129403,  147673,  166669,
         201403,  222163,  242729,  261431,  303491,  320237,  402761,  501131,
         602309,  701507,  800999,  900551, 1000619, 1100837, 1200359, 1300021,
        1400017, 1500007, 1750009, 2000003, 2500009, 3000017, 4000037, 5000011,
        6000011, 7000003, 8000009, 9000011, 9999991};
  hashtable *ht;

  /* Find a good size for the hash table */
  if(size < HASH_START_SIZE)
    size = HASH_START_SIZE;
  for(i = 0; i < NUMHASHPRIMES-1; i++)
    if(HashPrimes[i] > size)
      break;
  size = HashPrimes[i];

  /* Then allocate and initialize memory */
  ht = (hashtable *) calloc(1 , sizeof(*ht));
  ht->table = (hashelem **) calloc(size, sizeof(*(ht->table)));
  ht->size = size;
  ht->base = base;
  ht->count = base-1;

  return(ht);
}

STATIC void free_hash_item(hashelem **hp)
{
  free((*hp)->name);
  free(*hp);
  *hp = NULL;
}

STATIC void free_hash_table(hashtable *ht)
{
  hashelem *hp, *thp;

  hp = ht->first;
  while(hp != NULL) {
    thp = hp;
    hp = hp->nextelem;
    free_hash_item(&thp);
  }
  free(ht->table);
  free(ht);
}


/* make a good hash function for any int size */
/* inspired by Aho, Sethi and Ullman, Compilers ..., p436 */
#define HASH_1 sizeof(unsigned int)
#define HASH_2 (sizeof(unsigned int) * 6)
#define HASH_3 (((unsigned int)0xF0) << ((sizeof(unsigned int) - 1) * CHAR_BIT))

STATIC int hashval(const char *string, int size)
{
  unsigned int result = 0, tmp;

  for(; *string; string++) {
    result = (result << HASH_1) + *string;
    if((tmp = result & HASH_3) != 0) {
      /* if any of the most significant bits is on */
      result ^= tmp >> HASH_2; /* xor them in in a less significant part */
      result ^= tmp; /* and reset the most significant bits to 0 */
    }
  }
  return(result % size);
} /* hashval */


STATIC hashelem *findhash(const char *name, hashtable *ht)
{
  hashelem *h_tab_p;
  for(h_tab_p = ht->table[hashval(name, ht->size)];
      h_tab_p != NULL;
      h_tab_p = h_tab_p->next)
    if(strcmp(name, h_tab_p->name) == 0) /* got it! */
      break;
  return(h_tab_p);
} /* findhash */


STATIC hashelem *puthash(const char *name, int index, hashelem **list, hashtable *ht)
{
  hashelem *hp = NULL;
  int      hashindex;

  if(list != NULL) {
    hp = list[index];
    if(hp != NULL)
      list[index] = NULL;
  }

  if((hp = findhash(name, ht)) == NULL) {

    hashindex = hashval(name, ht->size);
    hp = (hashelem *) calloc(1, sizeof(*hp));
    allocCHAR(NULL, &hp->name, (int) (strlen(name) + 1), FALSE);
    strcpy(hp->name, name);
    hp->index = index;
    ht->count++;
    if(list != NULL)
      list[index] = hp;

    hp->next = ht->table[hashindex];
    ht->table[hashindex] = hp;
    if(ht->first == NULL)
      ht->first = hp;
    if(ht->last != NULL)
      ht->last->nextelem = hp;
    ht->last = hp;

  }
  return(hp);
}

STATIC void drophash(const char *name, hashelem **list, hashtable *ht) {
  hashelem *hp, *hp1, *hp2;
  int      hashindex;

  if((hp = findhash(name, ht)) != NULL) {
    hashindex = hashval(name, ht->size);
    if((hp1 = ht->table[hashindex]) != NULL) {
      hp2 = NULL;
      while((hp1 != NULL) && (hp1 != hp)) {
        hp2 = hp1;
        hp1 = hp1->next;
      }
      if(hp1 == hp) {
        if(hp2 != NULL)
          hp2->next = hp->next;
        else
          ht->table[hashindex] = hp->next;
      }

      hp1 = ht->first;
      hp2 = NULL;
      while((hp1 != NULL) && (hp1 != hp)) {
        hp2 = hp1;
        hp1 = hp1->nextelem;
      }
      if(hp1 == hp) {
        if(hp2 != NULL)
          hp2->nextelem = hp->nextelem;
        else {
          ht->first = hp->nextelem;
          if (ht->first == NULL)
            ht->last = NULL;
        }
      }
      if(list != NULL)
        list[hp->index] = NULL;
      free_hash_item(&hp);
      ht->count--;
    }
  }
}

STATIC hashtable *copy_hash_table(hashtable *ht, hashelem **list, int newsize)
{
  hashtable *copy;
  hashelem  *elem, *new_elem;

  if(newsize < ht->size)
    newsize = ht->size;

  copy = create_hash_table(newsize, ht->base);
  if (copy != NULL) {
    elem = ht->first;
    while (elem != NULL) {
      if((new_elem = puthash(elem->name, elem->index, list, copy)) == NULL) {
        free_hash_table(copy);
        return(NULL);
      }
      elem = elem ->nextelem;
    }
  }

  return(copy);
}

STATIC int find_row(lprec *lp, char *name, MYBOOL Unconstrained_rows_found)
{
  hashelem *hp;

  if (lp->rowname_hashtab != NULL)
      hp = findhash(name, lp->rowname_hashtab);
  else
      hp = NULL;

  if (hp == NULL) {
    if(Unconstrained_rows_found) { /* just ignore them in this case */
         return(-1);
    }
    else {
      return(-1);
    }
  }
  return(hp->index);
}

STATIC int find_var(lprec *lp, char *name, MYBOOL verbose)
{
  hashelem *hp;

  if (lp->colname_hashtab != NULL)
      hp = findhash(name, lp->colname_hashtab);
  else
      hp = NULL;

  if (hp == NULL) {
    if(verbose)
      report(lp, SEVERE, "find_var: Unknown variable name '%s'\n", name);
    return(-1);
  }
  return(hp->index);
}

