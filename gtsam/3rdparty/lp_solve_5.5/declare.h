#ifndef __DECLARE_H__
#define __DECLARE_H__

#if !defined ANSI_PROTOTYPES
# if defined MSDOS || defined __BORLANDC__ || defined __HIGHC__ || defined SCO_UNIX || defined AViiON
#  define ANSI_PROTOTYPES 1
# endif
#endif

#if ANSI_PROTOTYPES!=0
# define __OF(args)  args
#else
# define __OF(args)  ()
#endif

#if defined __HIGHC__
# define VARARG    ...
#else
# define VARARG
#endif

#endif
