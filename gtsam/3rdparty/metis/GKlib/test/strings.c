/*!
\file strings.c
\brief Testing module for the string functions in GKlib

\date Started 3/5/2007
\author George
\version\verbatim $Id: strings.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#include <GKlib.h>


/*************************************************************************/
/*! Testing module for gk_strstr_replace()  */
/*************************************************************************/
void test_strstr_replace()
{
  char *new_str;
  int rc;

  rc = gk_strstr_replace("This is a simple string", "s", "S", "", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);


  rc = gk_strstr_replace("This is a simple string", "s", "S", "g", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);


  rc = gk_strstr_replace("This is a simple SS & ss string", "s", "T", "g", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);


  rc = gk_strstr_replace("This is a simple SS & ss string", "s", "T", "ig", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);

  rc = gk_strstr_replace("This is a simple SS & ss string", "\\b\\w(\\w+)\\w\\b", "$1", "ig", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);

  rc = gk_strstr_replace("This is a simple SS & ss string", "\\b\\w+\\b", "word", "ig", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);

  rc = gk_strstr_replace("http://www.cs.umn.edu/This-is-something-T12323?pp=20&page=4",
                          "(http://www\\.cs\\.umn\\.edu/)(.*)-T(\\d+)", "$1$2-P$3", "g", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);

  rc = gk_strstr_replace("http://www.cs.umn.edu/This-is-something-T12323?pp=20&page=4",
                          "(\\d+)", "number:$1", "ig", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);


  rc = gk_strstr_replace("http://www.cs.umn.edu/This-is-something-T12323?pp=20&page=4",
                          "(http://www\\.cs\\.umn\\.edu/)", "[$1]", "g", &new_str);
  printf("%d, %s.\n", rc, new_str);
  gk_free((void **)&new_str, LTERM);



}



int main()
{
  test_strstr_replace();

/*
  {
  int i;
  for (i=0; i<1000; i++)
    printf("%d\n", RandomInRange(3));
  }
*/
}

