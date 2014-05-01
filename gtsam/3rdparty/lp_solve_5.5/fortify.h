#ifndef __FORTIFY_H__
#define __FORTIFY_H__
/*
 * FILE:
 *   fortify.h
 *
 * DESCRIPTION:
 *     Header file for fortify.c - A fortified shell for malloc, realloc,
 *   calloc, strdup, getcwd, tempnam & free
 *
 * WRITTEN:
 *   spb 29/4/94
 *
 * VERSION:
 *   1.0 29/4/94
 */
#include <stdlib.h>

#include "declare.h"

#if defined HP9000 || defined AViiON || defined ALPHA || defined SIGNED_UNKNOWN
# define signed
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef FORTIFY

typedef void (*OutputFuncPtr) __OF((char *));

extern char *_Fortify_file;
extern int _Fortify_line;

#define Fortify_FILE(file) _Fortify_file=file
#define Fortify_LINE(line) _Fortify_line=line

#define _Fortify_FILE (_Fortify_file==(char *) 0 ? __FILE__ : _Fortify_file)
#define _Fortify_LINE (_Fortify_line==0 ? __LINE__ : _Fortify_line)

void  _Fortify_Init __OF((char *file, unsigned long line));
void *_Fortify_malloc __OF((size_t size, char *file, unsigned long line));
void *_Fortify_realloc __OF((void *ptr, size_t new_size, char *file, unsigned long line));
void *_Fortify_calloc __OF((size_t nitems, size_t size, char *file, unsigned long line));
char *_Fortify_strdup __OF((char *str, char *file, unsigned long line));
void *_Fortify_memcpy __OF((void *to, void *from, size_t size, char *file, unsigned long line));
void *_Fortify_memmove __OF((void *to, void *from, size_t size, char *file, unsigned long line));
void *_Fortify_memccpy __OF((void *to, void *from, int c, size_t size, char *file, unsigned long line));
void *_Fortify_memset __OF((void *buffer, int c, size_t size, char *file, unsigned long line));
void *_Fortify_memchr __OF((void *buffer, int c, size_t size, char *file, unsigned long line));
int   _Fortify_memcmp __OF((void *buffer1, void *buffer2, size_t size, char *file, unsigned long line));
int   _Fortify_memicmp __OF((void *buffer1, void *buffer2, size_t size, char *file, unsigned long line));
char *_Fortify_strchr __OF((char *buffer, int c, char *file, unsigned long line));
char *_Fortify_strrchr __OF((char *buffer, int c, char *file, unsigned long line));
char *_Fortify_strlwr __OF((char *buffer, char *file, unsigned long line));
char *_Fortify_strupr __OF((char *buffer, char *file, unsigned long line));
char *_Fortify_strrev __OF((char *buffer, char *file, unsigned long line));
char *_Fortify_strset __OF((char *buffer, int c, char *file, unsigned long line));
char *_Fortify_strnset __OF((char *buffer, int c, size_t size, char *file, unsigned long line));
char *_Fortify_strstr __OF((char *to, char *from, char *file, unsigned long line));
char *_Fortify_strcpy __OF((char *to, char *from, char *file, unsigned long line));
char *_Fortify_strncpy __OF((char *to, char *from, size_t size, char *file, unsigned long line));
int   _Fortify_strcmp __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
double _Fortify_strtod __OF((char *buffer1, char **buffer2, char *file, unsigned long line));
long _Fortify_strtol __OF((char *buffer1, char **buffer2, int n, char *file, unsigned long line));
int _Fortify_atoi __OF((char *buffer1, char *file, unsigned long line));
long _Fortify_atol __OF((char *buffer1, char *file, unsigned long line));
double _Fortify_atof __OF((char *buffer1, char *file, unsigned long line));
unsigned long _Fortify_strtoul __OF((char *buffer1, char **buffer2, int n, char *file, unsigned long line));
size_t _Fortify_strcspn __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
int   _Fortify_strcoll __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
int   _Fortify_strcmpi __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
int   _Fortify_strncmp __OF((char *buffer1, char *buffer2, size_t size, char *file, unsigned long line));
int   _Fortify_strnicmp __OF((char *buffer1, char *buffer2, size_t size, char *file, unsigned long line));
char *_Fortify_strcat __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
char *_Fortify_strpbrk __OF((char *buffer1, char *buffer2, char *file, unsigned long line));
size_t _Fortify_strlen __OF((char *buffer, char *file, unsigned long line));
char *_Fortify_getcwd __OF((char *buf, int size, char *file, unsigned long line));
char *_Fortify_tempnam __OF((char *dir, char *pfx, char *file, unsigned long line));
void  _Fortify_free __OF((void *uptr, char *file, unsigned long line));

int   _Fortify_OutputAllMemory __OF((char *file, unsigned long line));
int   _Fortify_CheckAllMemory __OF((char *file, unsigned long line));
int   _Fortify_CheckPointer __OF((void *uptr, char *file, unsigned long line));
int   _Fortify_Disable __OF((char *file, unsigned long line, int how));
int   _Fortify_SetMallocFailRate __OF((int Percent));
int   _Fortify_EnterScope __OF((char *file, unsigned long line));
int   _Fortify_LeaveScope __OF((char *file, unsigned long line));
int   _Fortify_DumpAllMemory __OF((int scope, char *file, unsigned long line));

typedef void (*Fortify_OutputFuncPtr) __OF((/* const */ char *));
Fortify_OutputFuncPtr _Fortify_SetOutputFunc __OF((Fortify_OutputFuncPtr Output));

#endif /* FORTIFY */

#ifdef __cplusplus
}
#endif

#ifndef __FORTIFY_C__ /* Only define the macros if we're NOT in fortify.c */

#ifdef FORTIFY /* Add file and line information to the fortify calls */

#if defined malloc
# undef malloc
#endif
#if defined realloc
# undef realloc
#endif
#if defined calloc
# undef calloc
#endif
#if defined strdup
# undef strdup
#endif
#if defined memcpy
# undef memcpy
#endif
#if defined memmove
# undef memmove
#endif
#if defined memccpy
# undef memccpy
#endif
#if defined memset
# undef memset
#endif
#if defined memchr
# undef memchr
#endif
#if defined memcmp
# undef memcmp
#endif
#if defined memicmp
# undef memicmp
#endif
#if defined strcoll
# undef strcoll
#endif
#if defined strcspn
# undef strcspn
#endif
#if defined strcmp
# undef strcmp
#endif
#if defined strcmpi
# undef strcmpi
#endif
#if defined stricmp
# undef stricmp
#endif
#if defined strncmp
# undef strncmp
#endif
#if defined strnicmp
# undef strnicmp
#endif
#if defined strlwr
# undef strlwr
#endif
#if defined strupr
# undef strupr
#endif
#if defined strrev
# undef strrev
#endif
#if defined strchr
# undef strchr
#endif
#if defined strrchr
# undef strrchr
#endif
#if defined strcat
# undef strcat
#endif
#if defined strpbrk
# undef strpbrk
#endif
#if defined strcpy
# undef strcpy
#endif
#if defined atoi
# undef atoi
#endif
#if defined atol
# undef atol
#endif
#if defined atof
# undef atof
#endif
#if defined strtol
# undef strtol
#endif
#if defined strtoul
# undef strtoul
#endif
#if defined strtod
# undef strtod
#endif
#if defined strstr
# undef strstr
#endif
#if defined strncpy
# undef strncpy
#endif
#if defined strset
# undef strset
#endif
#if defined strnset
# undef strnset
#endif
#if defined strlen
# undef strlen
#endif
#if defined getcwd
# undef getcwd
#endif
#if defined tempnam
# undef tempnam
#endif
#if defined free
# undef free
#endif

#define malloc(size)                  _Fortify_malloc(size, _Fortify_FILE, _Fortify_LINE)
#define realloc(ptr,new_size)         _Fortify_realloc(ptr, new_size, _Fortify_FILE, _Fortify_LINE)
#define calloc(num,size)              _Fortify_calloc(num, size, _Fortify_FILE, _Fortify_LINE)
#define strdup(str)                   _Fortify_strdup(str, _Fortify_FILE, _Fortify_LINE)
#define memcpy(to,from,size)          _Fortify_memcpy((void *)(to),(void *)(from),size, _Fortify_FILE, _Fortify_LINE)
#define memmove(to,from,size)         _Fortify_memmove((void *)(to),(void *)(from),size, _Fortify_FILE, _Fortify_LINE)
#define memccpy(to,from,c,size)       _Fortify_memccpy((void *)(to),(void *)(from),c,size, _Fortify_FILE, _Fortify_LINE)
#define memset(buffer,c,size)         _Fortify_memset(buffer,c,size, _Fortify_FILE, _Fortify_LINE)
#define memchr(buffer,c,size)         _Fortify_memchr(buffer,c,size, _Fortify_FILE, _Fortify_LINE)
#define memcmp(buffer1,buffer2,size)  _Fortify_memcmp((void *)buffer1,(void *)buffer2,size, _Fortify_FILE, _Fortify_LINE)
#define memicmp(buffer1,buffer2,size) _Fortify_memicmp((void *)buffer1,(void *)buffer2,size, _Fortify_FILE, _Fortify_LINE)
#define strlwr(buffer)                _Fortify_strlwr(buffer, _Fortify_FILE, _Fortify_LINE)
#define strupr(buffer)                _Fortify_strupr(buffer, _Fortify_FILE, _Fortify_LINE)
#define strrev(buffer)                _Fortify_strrev(buffer, _Fortify_FILE, _Fortify_LINE)
#define strchr(buffer,c)              _Fortify_strchr(buffer,c, _Fortify_FILE, _Fortify_LINE)
#define strrchr(buffer,c)             _Fortify_strrchr(buffer,c, _Fortify_FILE, _Fortify_LINE)
#define strset(buffer,c)              _Fortify_strset(buffer,c, _Fortify_FILE, _Fortify_LINE)
#define strnset(buffer,c)             _Fortify_strnset(buffer,c, _Fortify_FILE, _Fortify_LINE)
#define strstr(buffer1,buffer2)       _Fortify_strstr(buffer1,buffer2, _Fortify_FILE, _Fortify_LINE)
#define atoi(buffer)                  _Fortify_atoi(buffer, _Fortify_FILE, _Fortify_LINE)
#define atol(buffer)                  _Fortify_atol(buffer, _Fortify_FILE, _Fortify_LINE)
#define atof(buffer)                  _Fortify_atof(buffer, _Fortify_FILE, _Fortify_LINE)
#define strtol(buffer1,buffer2,n)     _Fortify_strtol(buffer1,buffer2,n, _Fortify_FILE, _Fortify_LINE)
#define strtoul(buffer1,buffer2,n)    _Fortify_strtoul(buffer1,buffer2,n, _Fortify_FILE, _Fortify_LINE)
#define strtod(buffer1,buffer2)       _Fortify_strtod(buffer1,buffer2, _Fortify_FILE, _Fortify_LINE)
#define strcpy(to,from)               _Fortify_strcpy((char *)(to),(char *)(from), _Fortify_FILE, _Fortify_LINE)
#define strncpy(to,from,size)         _Fortify_strncpy((char *)(to),(char *)(from),size, _Fortify_FILE, _Fortify_LINE)
#define strcoll(buffer1,buffer2)      _Fortify_strcoll((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strcspn(buffer1,buffer2)      _Fortify_strcspn((char*)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strcmp(buffer1,buffer2)       _Fortify_strcmp((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strcmpi(buffer1,buffer2)      _Fortify_strcmpi((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define stricmp(buffer1,buffer2)      _Fortify_strcmpi((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strncmp(buffer1,buffer2,size) _Fortify_strncmp((char *)(buffer1),(char *)(buffer2),size, _Fortify_FILE, _Fortify_LINE)
#define strnicmp(buffer1,buffer2,size) _Fortify_strnicmp((char *)(buffer1),(char *)(buffer2),size, _Fortify_FILE, _Fortify_LINE)
#define strcat(buffer1,buffer2)       _Fortify_strcat((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strpbrk(buffer1,buffer2)      _Fortify_strpbrk((char *)(buffer1),(char *)(buffer2), _Fortify_FILE, _Fortify_LINE)
#define strlen(buffer)                _Fortify_strlen((char*)(buffer), _Fortify_FILE, _Fortify_LINE)
#define getcwd(buf,size)              _Fortify_getcwd(buf, size, _Fortify_FILE, _Fortify_LINE)
#define tempnam(dir,pfx)              _Fortify_tempnam(dir, pfx, _Fortify_FILE, _Fortify_LINE)
#define free(ptr)                     _Fortify_free(ptr, _Fortify_FILE, _Fortify_LINE)

#define Fortify_Init()                _Fortify_Init(_Fortify_FILE, _Fortify_LINE)
#define Fortify_OutputAllMemory()     _Fortify_OutputAllMemory(_Fortify_FILE, _Fortify_LINE)
#define Fortify_CheckAllMemory()      _Fortify_CheckAllMemory(_Fortify_FILE, _Fortify_LINE)
#define Fortify_CheckPointer(ptr)     _Fortify_CheckPointer(ptr, _Fortify_FILE, _Fortify_LINE)
#define Fortify_Disable(how)          _Fortify_Disable(_Fortify_FILE, _Fortify_LINE,how)
#define Fortify_EnterScope()          _Fortify_EnterScope(_Fortify_FILE, _Fortify_LINE)
#define Fortify_LeaveScope()          _Fortify_LeaveScope(_Fortify_FILE, _Fortify_LINE)
#define Fortify_DumpAllMemory(s)      _Fortify_DumpAllMemory(s,_Fortify_FILE, _Fortify_LINE)

#else /* FORTIFY Define the special fortify functions away to nothing */

#define Fortify_FILE(file)
#define Fortify_LINE(line)
#define Fortify_Init()
#define Fortify_OutputAllMemory()     0
#define Fortify_CheckAllMemory()      0
#define Fortify_CheckPointer(ptr)     1
#define Fortify_Disable(how)          1
#define Fortify_SetOutputFunc()       0
#define Fortify_SetMallocFailRate(p)  0
#define Fortify_EnterScope()          0
#define Fortify_LeaveScope()          0
#define Fortify_DumpAllMemory(s)      0

#endif /*   FORTIFY     */
#endif /* __FORTIFY_C__ */
#endif /* __FORTIFY_H__ */
