#ifndef HEADER_lp_lp
#define HEADER_lp_lp

#include "lp_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Put function headers here */
MYBOOL LP_writefile(lprec *lp, char *filename);
MYBOOL LP_writehandle(lprec *lp, FILE *output);


#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_lp */

