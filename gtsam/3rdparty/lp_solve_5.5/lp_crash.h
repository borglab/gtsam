
#ifndef HEADER_lp_crash
#define HEADER_lp_crash


#include "lp_types.h"

#define CRASH_SIMPLESCALE       /* Specify if we should use a simple absolute scaling threshold */

#define CRASH_THRESHOLD  0.167
#define CRASH_SPACER        10
#define CRASH_WEIGHT     0.500



#ifdef __cplusplus
__EXTERN_C {
#endif

STATIC MYBOOL crash_basis(lprec *lp);

#ifdef __cplusplus
}
#endif

#endif /* HEADER_lp_crash */

