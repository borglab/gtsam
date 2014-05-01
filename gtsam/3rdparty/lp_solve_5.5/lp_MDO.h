#ifndef HEADER_MDO
#define HEADER_MDO

#include "lp_types.h"


#ifdef __cplusplus
extern "C" {
#endif

int __WINAPI getMDO(lprec *lp, MYBOOL *usedpos, int *colorder, int *size, MYBOOL symmetric);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_MDO */

