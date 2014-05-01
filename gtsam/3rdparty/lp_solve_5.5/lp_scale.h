#ifndef HEADER_lp_scale
#define HEADER_lp_scale

#include "lp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Put function headers here */
STATIC MYBOOL scale_updatecolumns(lprec *lp, REAL *scalechange, MYBOOL updateonly);
STATIC MYBOOL scale_updaterows(lprec *lp, REAL *scalechange, MYBOOL updateonly);
STATIC MYBOOL scale_rows(lprec *lp, REAL *scaledelta);
STATIC MYBOOL scale_columns(lprec *lp, REAL *scaledelta);
STATIC void unscale_columns(lprec *lp);
STATIC REAL scale(lprec *lp, REAL *scaledelta);
STATIC REAL scaled_mat(lprec *lp, REAL value, int rownr, int colnr);
STATIC REAL unscaled_mat(lprec *lp, REAL value, int rownr, int colnr);
STATIC REAL scaled_value(lprec *lp, REAL value, int index);
STATIC REAL unscaled_value(lprec *lp, REAL value, int index);
STATIC MYBOOL scaleCR(lprec *lp, REAL *scaledelta);
STATIC MYBOOL finalize_scaling(lprec *lp, REAL *scaledelta);
STATIC REAL auto_scale(lprec *lp);
void undoscale(lprec *lp);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_scale */

