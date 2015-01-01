/*!
\file  tokenizer.c
\brief String tokenization routines

This file contains various routines for splitting an input string into
tokens and returning them in form of a list. The goal is to mimic perl's 
split function.

\date   Started 11/23/04
\author George
\version\verbatim $Id: tokenizer.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#include <GKlib.h>


/************************************************************************
* This function tokenizes a string based on the user-supplied delimiters
* list. The resulting tokens are returned into an array of strings.
*************************************************************************/
void gk_strtokenize(char *str, char *delim, gk_Tokens_t *tokens)
{
  int i, ntoks, slen;

  tokens->strbuf = gk_strdup(str);

  slen  = strlen(str);
  str   = tokens->strbuf;

  /* Scan once to determine the number of tokens */
  for (ntoks=0, i=0; i<slen;) {
    /* Consume all the consecutive characters from the delimiters list */
    while (i<slen && strchr(delim, str[i])) 
      i++;

    if (i == slen)
      break;

    ntoks++;

    /* Consume all the consecutive characters from the token */
    while (i<slen && !strchr(delim, str[i])) 
      i++;
  }


  tokens->ntoks = ntoks;
  tokens->list  = (char **)gk_malloc(ntoks*sizeof(char *), "strtokenize: tokens->list");


  /* Scan a second time to mark and link the tokens */
  for (ntoks=0, i=0; i<slen;) {
    /* Consume all the consecutive characters from the delimiters list */
    while (i<slen && strchr(delim, str[i])) 
      str[i++] = '\0';

    if (i == slen)
      break;

    tokens->list[ntoks++] = str+i;

    /* Consume all the consecutive characters from the token */
    while (i<slen && !strchr(delim, str[i])) 
      i++;
  }
}


/************************************************************************
* This function frees the memory associated with a gk_Tokens_t
*************************************************************************/
void gk_freetokenslist(gk_Tokens_t *tokens)
{
  gk_free((void *)&tokens->list, &tokens->strbuf, LTERM);
}

