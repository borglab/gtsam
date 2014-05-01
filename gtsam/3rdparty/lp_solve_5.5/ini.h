#include <stdio.h>

#ifdef __cplusplus
__EXTERN_C {
#endif

extern FILE *ini_create(char *filename);
extern FILE *ini_open(char *filename);
extern void ini_writecomment(FILE *fp, char *comment);
extern void ini_writeheader(FILE *fp, char *header, int addnewline);
extern void ini_writedata(FILE *fp, char *name, char *data);
extern int ini_readdata(FILE *fp, char *data, int szdata, int withcomment);
extern void ini_close(FILE *fp);

#ifdef __cplusplus
}
#endif
