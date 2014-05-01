#ifndef HEADER_lp_hash
#define HEADER_lp_hash

/* For row and column name hash tables */

typedef struct _hashelem
{
  char             *name;
  int               index;
  struct _hashelem *next;
  struct _hashelem *nextelem;
} hashelem;

typedef struct _hashtable
{
  hashelem         **table;
  int              size;
  int              base;
  int              count;
  struct _hashelem *first;
  struct _hashelem *last;
} hashtable;

#ifdef __cplusplus
extern "C" {
#endif

STATIC hashtable *create_hash_table(int size, int base);
STATIC void      free_hash_table(hashtable *ht);
STATIC hashelem  *findhash(const char *name, hashtable *ht);
STATIC hashelem  *puthash(const char *name, int index, hashelem **list, hashtable *ht);
STATIC void      drophash(const char *name, hashelem **list, hashtable *ht);
STATIC void      free_hash_item(hashelem **hp);
STATIC hashtable *copy_hash_table(hashtable *ht, hashelem **list, int newsize);
STATIC int find_var(lprec *lp, char *name, MYBOOL verbose);
STATIC int find_row(lprec *lp, char *name, MYBOOL Unconstrained_rows_found);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_hash */

