#ifndef HEADER_lp_MPS
#define HEADER_lp_MPS

#include "lp_types.h"

/* For MPS file reading and writing */
#define ROWNAMEMASK          "R%d"
#define ROWNAMEMASK2         "r%d"
#define COLNAMEMASK          "C%d"
#define COLNAMEMASK2         "c%d"

#ifdef __cplusplus
extern "C" {
#endif

/* Read an MPS file */
MYBOOL MPS_readfile(lprec **newlp, char *filename, int typeMPS, int verbose);
MYBOOL __WINAPI MPS_readhandle(lprec **newlp, FILE *filehandle, int typeMPS, int verbose);

/* Write a MPS file to output */
MYBOOL MPS_writefile(lprec *lp, int typeMPS, char *filename);
MYBOOL MPS_writehandle(lprec *lp, int typeMPS, FILE *output);

/* Read and write BAS files */
MYBOOL MPS_readBAS(lprec *lp, int typeMPS, char *filename, char *info);
MYBOOL MPS_writeBAS(lprec *lp, int typeMPS, char *filename);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_MPS */

