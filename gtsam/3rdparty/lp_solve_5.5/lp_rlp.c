
/*  A Bison parser, made from lp_rlp.y
    by GNU Bison version 1.28  */

#define YYBISON 1  /* Identify Bison output.  */

#define	VAR	257
#define	CONS	258
#define	INTCONS	259
#define	VARIABLECOLON	260
#define	INF	261
#define	SEC_INT	262
#define	SEC_BIN	263
#define	SEC_SEC	264
#define	SEC_SOS	265
#define	SOSDESCR	266
#define	SEC_FREE	267
#define	SIGN	268
#define	AR_M_OP	269
#define	RE_OPLE	270
#define	RE_OPGE	271
#define	END_C	272
#define	COMMA	273
#define	COLON	274
#define	MINIMISE	275
#define	MAXIMISE	276
#define	UNDEFINED	277


#include <string.h>
#include <ctype.h>

#include "lpkit.h"
#include "yacc_read.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

static int HadVar0, HadVar1, HadVar2, HasAR_M_OP, do_add_row, Had_lineair_sum0, HadSign;
static char *Last_var = NULL, *Last_var0 = NULL;
static REAL f, f0, f1;
static int x;
static int state, state0;
static int Sign;
static int isign, isign0;      /* internal_sign variable to make sure nothing goes wrong */
                /* with lookahead */
static int make_neg;   /* is true after the relational operator is seen in order */
                /* to remember if lin_term stands before or after re_op */
static int Within_int_decl = FALSE; /* TRUE when we are within an int declaration */
static int Within_bin_decl = FALSE; /* TRUE when we are within an bin declaration */
static int Within_sec_decl = FALSE; /* TRUE when we are within a sec declaration */
static int Within_sos_decl = FALSE; /* TRUE when we are within a sos declaration */
static int Within_sos_decl1;
static int Within_free_decl = FALSE; /* TRUE when we are within a free declaration */
static short SOStype0; /* SOS type */
static short SOStype; /* SOS type */
static int SOSNr;
static int SOSweight = 0; /* SOS weight */

static int HadConstraint;
static int HadVar;
static int Had_lineair_sum;

extern FILE *lp_yyin;

#define YY_FATAL_ERROR lex_fatal_error

/* let's please C++ users */
#ifdef __cplusplus
extern "C" {
#endif

static int wrap(void)
{
  return(1);
}

static int __WINAPI lp_input_lp_yyin(void *fpin, char *buf, int max_size)
{
  int result;

  if ( (result = fread( (char*)buf, sizeof(char), max_size, (FILE *) fpin)) < 0)
    YY_FATAL_ERROR( "read() in flex scanner failed");

  return(result);
}

static read_modeldata_func *lp_input;

#undef YY_INPUT
#define YY_INPUT(buf,result,max_size) result = lp_input((void *) lp_yyin, buf, max_size);

#ifdef __cplusplus
};
#endif

#define lp_yywrap wrap
#define lp_yyerror read_error

#include "lp_rlp.h"

#ifndef YYSTYPE
#define YYSTYPE int
#endif
#include <stdio.h>

#ifndef __cplusplus
#ifndef __STDC__
#define const
#endif
#endif



#define	YYFINAL		122
#define	YYFLAG		-32768
#define	YYNTBASE	24

#define YYTRANSLATE(x) ((unsigned)(x) <= 277 ? lp_yytranslate[x] : 79)

static const char lp_yytranslate[] = {     0,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
     2,     2,     2,     2,     2,     1,     3,     4,     5,     6,
     7,     8,     9,    10,    11,    12,    13,    14,    15,    16,
    17,    18,    19,    20,    21,    22,    23
};

#if YYDEBUG != 0
static const short lp_yyprhs[] = {     0,
     0,     1,     2,     7,    10,    13,    15,    18,    20,    22,
    24,    26,    28,    31,    33,    34,    38,    39,    40,    41,
    50,    52,    53,    54,    60,    62,    64,    66,    67,    71,
    72,    75,    77,    80,    83,    85,    86,    90,    92,    94,
    97,    99,   101,   103,   105,   107,   109,   111,   113,   115,
   117,   119,   122,   124,   126,   128,   130,   132,   133,   137,
   138,   144,   146,   149,   151,   152,   156,   158,   159,   164,
   166,   169,   171,   173,   175,   179,   181,   183,   185,   186,
   188,   190,   193,   197,   200,   203,   206
};

static const short lp_yyrhs[] = {    -1,
     0,    26,    27,    30,    56,     0,    22,    28,     0,    21,
    28,     0,    28,     0,    29,    18,     0,    24,     0,    44,
     0,    24,     0,    31,     0,    32,     0,    31,    32,     0,
    34,     0,     0,     6,    33,    34,     0,     0,     0,     0,
    41,    35,    50,    36,    42,    37,    38,    18,     0,    24,
     0,     0,     0,    50,    39,    51,    40,    55,     0,    24,
     0,    42,     0,    44,     0,     0,     7,    43,    55,     0,
     0,    45,    46,     0,    47,     0,    46,    47,     0,    53,
    48,     0,    52,     0,     0,    54,    49,     3,     0,    16,
     0,    17,     0,    53,    52,     0,     7,     0,     5,     0,
     4,     0,    24,     0,    14,     0,    24,     0,    15,     0,
    24,     0,    24,     0,    57,     0,    59,     0,    57,    59,
     0,     8,     0,     9,     0,    10,     0,    11,     0,    13,
     0,     0,    58,    60,    63,     0,     0,    62,    64,    69,
    66,    18,     0,    61,     0,    63,    61,     0,    24,     0,
     0,    12,    65,    75,     0,    24,     0,     0,    16,     5,
    67,    68,     0,    24,     0,    20,     5,     0,    24,     0,
    70,     0,    76,     0,    70,    71,    76,     0,    24,     0,
    19,     0,    24,     0,     0,    24,     0,    24,     0,     3,
    72,     0,     6,    73,    77,     0,    52,    74,     0,    75,
    78,     0,     3,    72,     0,     6,    73,    52,    74,     0
};

#endif

#if YYDEBUG != 0
static const short lp_yyrline[] = { 0,
    88,    91,   100,   114,   118,   122,   125,   136,   137,   167,
   168,   171,   172,   176,   177,   184,   186,   192,   199,   223,
   239,   245,   256,   260,   281,   291,   297,   298,   303,   305,
   310,   324,   325,   329,   365,   369,   377,   382,   382,   385,
   387,   398,   398,   401,   406,   413,   417,   423,   440,   442,
   445,   446,   449,   449,   449,   449,   449,   452,   458,   460,
   470,   482,   484,   487,   488,   494,   496,   503,   517,   519,
   523,   530,   531,   534,   535,   540,   541,   544,   569,   588,
   610,   624,   627,   632,   634,   638,   641
};
#endif


#if YYDEBUG != 0 || defined (YYERROR_VERBOSE)

static const char * const lp_yytname[] = {   "$","error","$undefined.","VAR","CONS",
"INTCONS","VARIABLECOLON","INF","SEC_INT","SEC_BIN","SEC_SEC","SEC_SOS","SOSDESCR",
"SEC_FREE","SIGN","AR_M_OP","RE_OPLE","RE_OPGE","END_C","COMMA","COLON","MINIMISE",
"MAXIMISE","UNDEFINED","EMPTY","inputfile","@1","objective_function","real_of",
"lineair_sum","constraints","x_constraints","constraint","@2","real_constraint",
"@3","@4","@5","optionalrange","@6","@7","x_lineair_sum2","x_lineair_sum3","@8",
"x_lineair_sum","@9","x_lineair_sum1","x_lineair_term","x_lineair_term1","@10",
"RE_OP","cons_term","REALCONS","x_SIGN","optional_AR_M_OP","RHS_STORE","int_bin_sec_sos_free_declarations",
"real_int_bin_sec_sos_free_decls","SEC_INT_BIN_SEC_SOS_FREE","int_bin_sec_sos_free_declaration",
"@11","xx_int_bin_sec_sos_free_declaration","@12","x_int_bin_sec_sos_free_declaration",
"optionalsos","@13","optionalsostype","@14","optionalSOSweight","vars","x_vars",
"optionalcomma","variable","variablecolon","sosweight","sosdescr","onevarwithoptionalweight",
"INTCONSorVARIABLE","x_onevarwithoptionalweight", NULL
};
#endif

static const short lp_yyr1[] = {     0,
    24,    26,    25,    27,    27,    27,    28,    29,    29,    30,
    30,    31,    31,    32,    33,    32,    35,    36,    37,    34,
    38,    39,    40,    38,    41,    41,    42,    43,    42,    45,
    44,    46,    46,    47,    48,    49,    48,    50,    50,    51,
    51,    52,    52,    53,    53,    54,    54,    55,    56,    56,
    57,    57,    58,    58,    58,    58,    58,    60,    59,    62,
    61,    63,    63,    64,    65,    64,    66,    67,    66,    68,
    68,    69,    69,    70,    70,    71,    71,    72,    73,    74,
    75,    76,    76,    77,    77,    78,    78
};

static const short lp_yyr2[] = {     0,
     0,     0,     4,     2,     2,     1,     2,     1,     1,     1,
     1,     1,     2,     1,     0,     3,     0,     0,     0,     8,
     1,     0,     0,     5,     1,     1,     1,     0,     3,     0,
     2,     1,     2,     2,     1,     0,     3,     1,     1,     2,
     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,
     1,     2,     1,     1,     1,     1,     1,     0,     3,     0,
     5,     1,     2,     1,     0,     3,     1,     0,     4,     1,
     2,     1,     1,     1,     3,     1,     1,     1,     0,     1,
     1,     2,     3,     2,     2,     2,     4
};

static const short lp_yydefact[] = {     2,
    30,    30,    30,     8,     1,     6,     0,     9,     1,     5,
     4,    15,    28,    10,     1,    11,    12,    14,    17,    26,
    27,     7,    45,    44,     1,    32,     1,    30,     1,    53,
    54,    55,    56,    57,    49,     3,    50,    58,    51,    25,
    13,     0,    33,    43,    42,    47,    46,    34,    35,    36,
    16,    48,    29,    52,    60,    38,    39,    18,     0,    62,
     1,    59,    30,    37,    65,    64,     1,    63,    19,     1,
     1,    79,    72,     1,     1,    74,     1,    81,    66,    78,
    82,     1,     0,    67,     0,    77,    76,     0,    21,     0,
    22,     1,     0,    83,    68,    61,    75,    20,     1,    80,
    84,     1,    79,    85,     1,    41,    23,     0,    86,     0,
     0,    70,    69,     1,    40,     1,    71,    24,    87,     0,
     0,     0
};

static const short lp_yydefgoto[] = {     4,
   120,     1,     5,     6,     7,    15,    16,    17,    28,    18,
    42,    63,    77,    90,    99,   114,    19,    20,    29,    21,
     9,    25,    26,    48,    59,    58,   107,    49,    27,    50,
    53,    36,    37,    38,    39,    55,    60,    61,    62,    67,
    70,    85,   105,   113,    74,    75,    88,    81,    82,   101,
    79,    76,    94,   104
};

static const short lp_yypact[] = {-32768,
    42,   -16,   -16,-32768,    32,-32768,   -11,-32768,    -1,-32768,
-32768,-32768,-32768,     9,    40,    27,-32768,-32768,-32768,-32768,
-32768,-32768,-32768,-32768,    41,-32768,     4,    -2,-32768,-32768,
-32768,-32768,-32768,-32768,-32768,-32768,    40,-32768,-32768,-32768,
-32768,    51,-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,
-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,    37,-32768,
     5,     0,    38,-32768,-32768,-32768,    72,-32768,-32768,-32768,
-32768,-32768,-32768,    36,    55,-32768,    51,-32768,-32768,-32768,
-32768,    80,    49,-32768,    43,-32768,-32768,    72,-32768,    65,
-32768,-32768,    73,-32768,-32768,-32768,-32768,-32768,    14,-32768,
-32768,-32768,-32768,-32768,    66,-32768,-32768,    80,-32768,    80,
    85,-32768,-32768,-32768,-32768,-32768,-32768,-32768,-32768,    91,
    92,-32768
};

static const short lp_yypgoto[] = {    -5,
-32768,-32768,-32768,    86,-32768,-32768,-32768,    77,-32768,    67,
-32768,-32768,-32768,-32768,-32768,-32768,-32768,    33,-32768,    79,
-32768,-32768,    74,-32768,-32768,    21,-32768,   -81,     2,-32768,
   -12,-32768,-32768,-32768,    68,-32768,    44,-32768,-32768,-32768,
-32768,-32768,-32768,-32768,-32768,-32768,-32768,     1,     7,    -9,
    22,    20,-32768,-32768
};


#define	YYLAST		111


static const short lp_yytable[] = {    14,
    92,    -1,   -60,    24,    13,   -60,    22,    44,    45,    35,
    40,   -60,    23,    -1,    -1,   -60,    65,   -60,    46,    24,
   106,    47,    40,    52,   -25,   -25,   115,    23,   116,   -30,
   -30,   -30,    12,    13,   -30,   -30,   -30,    12,    13,    64,
   -30,   -30,    -1,    -1,    13,   -30,   -30,    30,    31,    32,
    33,    83,    34,    95,    23,    66,   -31,   -31,   -31,    -1,
    96,    73,     2,     3,    78,    80,    56,    57,    84,    87,
   -73,    89,   -73,    86,    71,   102,    78,    72,   103,     8,
     8,     8,    98,    44,    45,   111,   100,    10,    11,   117,
   121,   122,    41,    24,    51,    69,    80,    91,    43,   112,
   108,   118,   109,    93,    54,    68,   119,    97,    52,   110,
   100
};

static const short lp_yycheck[] = {     5,
    82,    18,     3,     9,     7,     6,    18,     4,     5,    15,
    16,    12,    14,    16,    17,    16,    12,    18,    15,    25,
     7,    27,    28,    29,    16,    17,   108,    14,   110,     3,
     4,     5,     6,     7,     3,     4,     5,     6,     7,     3,
    14,    15,    16,    17,     7,    14,    15,     8,     9,    10,
    11,    16,    13,     5,    14,    61,    16,    17,    18,    18,
    18,    67,    21,    22,    70,    71,    16,    17,    74,    75,
    16,    77,    18,    19,     3,     3,    82,     6,     6,     1,
     2,     3,    18,     4,     5,    20,    92,     2,     3,     5,
     0,     0,    16,    99,    28,    63,   102,    77,    25,   105,
    99,   114,   102,    82,    37,    62,   116,    88,   114,   103,
   116
};
/* -*-C-*-  Note some compilers choke on comments on `#line' lines.  */

/* This file comes from bison-1.28.  */

/* Skeleton output parser for bison,
   Copyright (C) 1984, 1989, 1990 Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

/* As a special exception, when this file is copied by Bison into a
   Bison output file, you may use that output file without restriction.
   This special exception was added by the Free Software Foundation
   in version 1.24 of Bison.  */

/* This is the parser code that is written into each bison parser
  when the %semantic_parser declaration is not specified in the grammar.
  It was written by Richard Stallman by simplifying the hairy parser
  used when %semantic_parser is specified.  */

#ifndef YYSTACK_USE_ALLOCA
#ifdef alloca
#define YYSTACK_USE_ALLOCA
#else /* alloca not defined */
#ifdef __GNUC__
#define YYSTACK_USE_ALLOCA
#define alloca __builtin_alloca
#else /* not GNU C.  */
#if (!defined (__STDC__) && defined (sparc)) || defined (__sparc__) || defined (__sparc) || defined (__sgi) || (defined (__sun) && defined (__i386))
#define YYSTACK_USE_ALLOCA
#include <alloca.h>
#else /* not sparc */
/* We think this test detects Watcom and Microsoft C.  */
/* This used to test MSDOS, but that is a bad idea
   since that symbol is in the user namespace.  */
#if (defined (_MSDOS) || defined (_MSDOS_)) && !defined (__TURBOC__)
#if 0 /* No need for malloc.h, which pollutes the namespace;
	 instead, just don't use alloca.  */
#include <malloc.h>
#endif
#else /* not MSDOS, or __TURBOC__ */
#if defined(_AIX)
/* I don't know what this was needed for, but it pollutes the namespace.
   So I turned it off.   rms, 2 May 1997.  */
/* #include <malloc.h>  */
 #pragma alloca
#define YYSTACK_USE_ALLOCA
#else /* not MSDOS, or __TURBOC__, or _AIX */
#if 0
#ifdef __hpux /* haible@ilog.fr says this works for HPUX 9.05 and up,
		 and on HPUX 10.  Eventually we can turn this on.  */
#define YYSTACK_USE_ALLOCA
#define alloca __builtin_alloca
#endif /* __hpux */
#endif
#endif /* not _AIX */
#endif /* not MSDOS, or __TURBOC__ */
#endif /* not sparc */
#endif /* not GNU C */
#endif /* alloca not defined */
#endif /* YYSTACK_USE_ALLOCA not defined */

#ifdef YYSTACK_USE_ALLOCA
#define YYSTACK_ALLOC alloca
#else
#define YYSTACK_ALLOC malloc
#endif

/* Note: there must be only one dollar sign in this file.
   It is replaced by the list of actions, each action
   as one case of the switch.  */

#define lp_yyerrok		(lp_yyerrstatus = 0)
#define lp_yyclearin	(lp_yychar = YYEMPTY)
#define YYEMPTY		-2
#define YYEOF		0
#define YYACCEPT	goto lp_yyacceptlab
#define YYABORT 	goto lp_yyabortlab
#define YYERROR		goto lp_yyerrlab1
/* Like YYERROR except do call lp_yyerror.
   This remains here temporarily to ease the
   transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */
#define YYFAIL		goto lp_yyerrlab
#define YYRECOVERING()  (!!lp_yyerrstatus)
#define YYBACKUP(token, value) \
do								\
  if (lp_yychar == YYEMPTY && lp_yylen == 1)				\
    { lp_yychar = (token), lp_yylval = (value);			\
      lp_yychar1 = YYTRANSLATE (lp_yychar);				\
      YYPOPSTACK;						\
      goto lp_yybackup;						\
    }								\
  else								\
    { lp_yyerror ("syntax error: cannot back up"); YYERROR; }	\
while (0)

#define YYTERROR	1
#define YYERRCODE	256

#ifndef YYPURE
#define YYLEX		lp_yylex()
#endif

#ifdef YYPURE
#ifdef YYLSP_NEEDED
#ifdef YYLEX_PARAM
#define YYLEX		lp_yylex(&lp_yylval, &lp_yylloc, YYLEX_PARAM)
#else
#define YYLEX		lp_yylex(&lp_yylval, &lp_yylloc)
#endif
#else /* not YYLSP_NEEDED */
#ifdef YYLEX_PARAM
#define YYLEX		lp_yylex(&lp_yylval, YYLEX_PARAM)
#else
#define YYLEX		lp_yylex(&lp_yylval)
#endif
#endif /* not YYLSP_NEEDED */
#endif

/* If nonreentrant, generate the variables here */

#ifndef YYPURE

int	lp_yychar;			/*  the lookahead symbol		*/
YYSTYPE	lp_yylval;			/*  the semantic value of the		*/
				/*  lookahead symbol			*/

#ifdef YYLSP_NEEDED
YYLTYPE lp_yylloc;			/*  location data for the lookahead	*/
				/*  symbol				*/
#endif

int lp_yynerrs;			/*  number of parse errors so far       */
#endif  /* not YYPURE */

#if YYDEBUG != 0
int lp_yydebug;			/*  nonzero means print parse trace	*/
/* Since this is uninitialized, it does not stop multiple parsers
   from coexisting.  */
#endif

/*  YYINITDEPTH indicates the initial size of the parser's stacks	*/

#ifndef	YYINITDEPTH
#define YYINITDEPTH 200
#endif

/*  YYMAXDEPTH is the maximum size the stacks can grow to
    (effective only if the built-in stack extension method is used).  */

#if YYMAXDEPTH == 0
#undef YYMAXDEPTH
#endif

#ifndef YYMAXDEPTH
#define YYMAXDEPTH 10000
#endif

/* Define __lp_yy_memcpy.  Note that the size argument
   should be passed with type unsigned int, because that is what the non-GCC
   definitions require.  With GCC, __builtin_memcpy takes an arg
   of type size_t, but it can handle unsigned int.  */

#if __GNUC__ > 1		/* GNU C and GNU C++ define this.  */
#define __lp_yy_memcpy(TO,FROM,COUNT)	__builtin_memcpy(TO,FROM,COUNT)
#else				/* not GNU C or C++ */
#ifndef __cplusplus

/* This is the most reliable way to avoid incompatibilities
   in available built-in functions on various systems.  */
static void
__lp_yy_memcpy (to, from, count)
     char *to;
     char *from;
     unsigned int count;
{
  register char *f = from;
  register char *t = to;
  register int i = count;

  while (i-- > 0)
    *t++ = *f++;
}

#else /* __cplusplus */

/* This is the most reliable way to avoid incompatibilities
   in available built-in functions on various systems.  */
static void
__lp_yy_memcpy (char *to, char *from, unsigned int count)
{
  register char *t = to;
  register char *f = from;
  register int i = count;

  while (i-- > 0)
    *t++ = *f++;
}

#endif
#endif



/* The user can define YYPARSE_PARAM as the name of an argument to be passed
   into lp_yyparse.  The argument should have type void *.
   It should actually point to an object.
   Grammar actions can access the variable by casting it
   to the proper pointer type.  */

#ifdef YYPARSE_PARAM
#ifdef __cplusplus
#define YYPARSE_PARAM_ARG void *YYPARSE_PARAM
#define YYPARSE_PARAM_DECL
#else /* not __cplusplus */
#define YYPARSE_PARAM_ARG YYPARSE_PARAM
#define YYPARSE_PARAM_DECL void *YYPARSE_PARAM;
#endif /* not __cplusplus */
#else /* not YYPARSE_PARAM */
#define YYPARSE_PARAM_ARG
#define YYPARSE_PARAM_DECL
#endif /* not YYPARSE_PARAM */

/* Prevent warning if -Wstrict-prototypes.  */
#ifdef __GNUC__
#ifdef YYPARSE_PARAM
int lp_yyparse (void *);
#else
int lp_yyparse (void);
#endif
#endif

int
lp_yyparse(YYPARSE_PARAM_ARG)
     YYPARSE_PARAM_DECL
{
  register int lp_yystate;
  register int lp_yyn;
  register short *lp_yyssp;
  register YYSTYPE *lp_yyvsp;
  int lp_yyerrstatus;	/*  number of tokens to shift before error messages enabled */
  int lp_yychar1 = 0;		/*  lookahead token as an internal (translated) token number */

  short	lp_yyssa[YYINITDEPTH];	/*  the state stack			*/
  YYSTYPE lp_yyvsa[YYINITDEPTH];	/*  the semantic value stack		*/

  short *lp_yyss = lp_yyssa;		/*  refer to the stacks thru separate pointers */
  YYSTYPE *lp_yyvs = lp_yyvsa;	/*  to allow lp_yyoverflow to reallocate them elsewhere */

#ifdef YYLSP_NEEDED
  YYLTYPE lp_yylsa[YYINITDEPTH];	/*  the location stack			*/
  YYLTYPE *lp_yyls = lp_yylsa;
  YYLTYPE *lp_yylsp;

#define YYPOPSTACK   (lp_yyvsp--, lp_yyssp--, lp_yylsp--)
#else
#define YYPOPSTACK   (lp_yyvsp--, lp_yyssp--)
#endif

  int lp_yystacksize = YYINITDEPTH;
  int lp_yyfree_stacks = 0;

#ifdef YYPURE
  int lp_yychar;
  YYSTYPE lp_yylval;
  int lp_yynerrs;
#ifdef YYLSP_NEEDED
  YYLTYPE lp_yylloc;
#endif
#endif

  YYSTYPE lp_yyval = 0;		/*  the variable used to return		*/
				/*  semantic values from the action	*/
				/*  routines				*/

  int lp_yylen;

#if YYDEBUG != 0
  if (lp_yydebug)
    fprintf(stderr, "Starting parse\n");
#endif

  lp_yystate = 0;
  lp_yyerrstatus = 0;
  lp_yynerrs = 0;
  lp_yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  lp_yyssp = lp_yyss - 1;
  lp_yyvsp = lp_yyvs;
#ifdef YYLSP_NEEDED
  lp_yylsp = lp_yyls;
#endif

/* Push a new state, which is found in  lp_yystate  .  */
/* In all cases, when you get here, the value and location stacks
   have just been pushed. so pushing a state here evens the stacks.  */
lp_yynewstate:

  *++lp_yyssp = lp_yystate;

  if (lp_yyssp >= lp_yyss + lp_yystacksize - 1)
    {
      /* Give user a chance to reallocate the stack */
      /* Use copies of these so that the &'s don't force the real ones into memory. */
      YYSTYPE *lp_yyvs1 = lp_yyvs;
      short *lp_yyss1 = lp_yyss;
#ifdef YYLSP_NEEDED
      YYLTYPE *lp_yyls1 = lp_yyls;
#endif

      /* Get the current used size of the three stacks, in elements.  */
      int size = lp_yyssp - lp_yyss + 1;

#ifdef lp_yyoverflow
      /* Each stack pointer address is followed by the size of
	 the data in use in that stack, in bytes.  */
#ifdef YYLSP_NEEDED
      /* This used to be a conditional around just the two extra args,
	 but that might be undefined if lp_yyoverflow is a macro.  */
      lp_yyoverflow("parser stack overflow",
		 &lp_yyss1, size * sizeof (*lp_yyssp),
		 &lp_yyvs1, size * sizeof (*lp_yyvsp),
		 &lp_yyls1, size * sizeof (*lp_yylsp),
		 &lp_yystacksize);
#else
      lp_yyoverflow("parser stack overflow",
		 &lp_yyss1, size * sizeof (*lp_yyssp),
		 &lp_yyvs1, size * sizeof (*lp_yyvsp),
		 &lp_yystacksize);
#endif

      lp_yyss = lp_yyss1; lp_yyvs = lp_yyvs1;
#ifdef YYLSP_NEEDED
      lp_yyls = lp_yyls1;
#endif
#else /* no lp_yyoverflow */
      /* Extend the stack our own way.  */
      if (lp_yystacksize >= YYMAXDEPTH)
	{
	  lp_yyerror("parser stack overflow");
	  if (lp_yyfree_stacks)
	    {
	      free (lp_yyss);
	      free (lp_yyvs);
#ifdef YYLSP_NEEDED
	      free (lp_yyls);
#endif
	    }
	  return 2;
	}
      lp_yystacksize *= 2;
      if (lp_yystacksize > YYMAXDEPTH)
	lp_yystacksize = YYMAXDEPTH;
#ifndef YYSTACK_USE_ALLOCA
      lp_yyfree_stacks = 1;
#endif
      lp_yyss = (short *) YYSTACK_ALLOC (lp_yystacksize * sizeof (*lp_yyssp));
      __lp_yy_memcpy ((char *)lp_yyss, (char *)lp_yyss1,
		   size * (unsigned int) sizeof (*lp_yyssp));
      lp_yyvs = (YYSTYPE *) YYSTACK_ALLOC (lp_yystacksize * sizeof (*lp_yyvsp));
      __lp_yy_memcpy ((char *)lp_yyvs, (char *)lp_yyvs1,
		   size * (unsigned int) sizeof (*lp_yyvsp));
#ifdef YYLSP_NEEDED
      lp_yyls = (YYLTYPE *) YYSTACK_ALLOC (lp_yystacksize * sizeof (*lp_yylsp));
      __lp_yy_memcpy ((char *)lp_yyls, (char *)lp_yyls1,
		   size * (unsigned int) sizeof (*lp_yylsp));
#endif
#endif /* no lp_yyoverflow */

      lp_yyssp = lp_yyss + size - 1;
      lp_yyvsp = lp_yyvs + size - 1;
#ifdef YYLSP_NEEDED
      lp_yylsp = lp_yyls + size - 1;
#endif

#if YYDEBUG != 0
      if (lp_yydebug)
	fprintf(stderr, "Stack size increased to %d\n", lp_yystacksize);
#endif

      if (lp_yyssp >= lp_yyss + lp_yystacksize - 1)
	YYABORT;
    }

#if YYDEBUG != 0
  if (lp_yydebug)
    fprintf(stderr, "Entering state %d\n", lp_yystate);
#endif

  goto lp_yybackup;
 lp_yybackup:

/* Do appropriate processing given the current state.  */
/* Read a lookahead token if we need one and don't already have one.  */
/* lp_yyresume: */

  /* First try to decide what to do without reference to lookahead token.  */

  lp_yyn = lp_yypact[lp_yystate];
  if (lp_yyn == YYFLAG)
    goto lp_yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* lp_yychar is either YYEMPTY or YYEOF
     or a valid token in external form.  */

  if (lp_yychar == YYEMPTY)
    {
#if YYDEBUG != 0
      if (lp_yydebug)
	fprintf(stderr, "Reading a token: ");
#endif
      lp_yychar = YYLEX;
    }

  /* Convert token to internal form (in lp_yychar1) for indexing tables with */

  if (lp_yychar <= 0)		/* This means end of input. */
    {
      lp_yychar1 = 0;
      lp_yychar = YYEOF;		/* Don't call YYLEX any more */

#if YYDEBUG != 0
      if (lp_yydebug)
	fprintf(stderr, "Now at end of input.\n");
#endif
    }
  else
    {
      lp_yychar1 = YYTRANSLATE(lp_yychar);

#if YYDEBUG != 0
      if (lp_yydebug)
	{
	  fprintf (stderr, "Next token is %d (%s", lp_yychar, lp_yytname[lp_yychar1]);
	  /* Give the individual parser a way to print the precise meaning
	     of a token, for further debugging info.  */
#ifdef YYPRINT
	  YYPRINT (stderr, lp_yychar, lp_yylval);
#endif
	  fprintf (stderr, ")\n");
	}
#endif
    }

  lp_yyn += lp_yychar1;
  if (lp_yyn < 0 || lp_yyn > YYLAST || lp_yycheck[lp_yyn] != lp_yychar1)
    goto lp_yydefault;

  lp_yyn = lp_yytable[lp_yyn];

  /* lp_yyn is what to do for this token type in this state.
     Negative => reduce, -lp_yyn is rule number.
     Positive => shift, lp_yyn is new state.
       New state is final state => don't bother to shift,
       just return success.
     0, or most negative number => error.  */

  if (lp_yyn < 0)
    {
      if (lp_yyn == YYFLAG)
	goto lp_yyerrlab;
      lp_yyn = -lp_yyn;
      goto lp_yyreduce;
    }
  else if (lp_yyn == 0)
    goto lp_yyerrlab;

  if (lp_yyn == YYFINAL)
    YYACCEPT;

  /* Shift the lookahead token.  */

#if YYDEBUG != 0
  if (lp_yydebug)
    fprintf(stderr, "Shifting token %d (%s), ", lp_yychar, lp_yytname[lp_yychar1]);
#endif

  /* Discard the token being shifted unless it is eof.  */
  if (lp_yychar != YYEOF)
    lp_yychar = YYEMPTY;

  *++lp_yyvsp = lp_yylval;
#ifdef YYLSP_NEEDED
  *++lp_yylsp = lp_yylloc;
#endif

  /* count tokens shifted since error; after three, turn off error status.  */
  if (lp_yyerrstatus) lp_yyerrstatus--;

  lp_yystate = lp_yyn;
  goto lp_yynewstate;

/* Do the default action for the current state.  */
lp_yydefault:

  lp_yyn = lp_yydefact[lp_yystate];
  if (lp_yyn == 0)
    goto lp_yyerrlab;

/* Do a reduction.  lp_yyn is the number of a rule to reduce with.  */
lp_yyreduce:
  lp_yylen = lp_yyr2[lp_yyn];
  if (lp_yylen > 0)
    lp_yyval = lp_yyvsp[1-lp_yylen]; /* implement default value of the action */

#if YYDEBUG != 0
  if (lp_yydebug)
    {
      int i;

      fprintf (stderr, "Reducing via rule %d (line %d), ",
	       lp_yyn, lp_yyrline[lp_yyn]);

      /* Print the symbols being reduced, and their result.  */
      for (i = lp_yyprhs[lp_yyn]; lp_yyrhs[i] > 0; i++)
	fprintf (stderr, "%s ", lp_yytname[lp_yyrhs[i]]);
      fprintf (stderr, " -> %s\n", lp_yytname[lp_yyr1[lp_yyn]]);
    }
#endif


  switch (lp_yyn) {

case 2:
{
  isign = 0;
  make_neg = 0;
  Sign = 0;
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
;
    break;}
case 4:
{
  set_obj_dir(TRUE);
;
    break;}
case 5:
{
  set_obj_dir(FALSE);
;
    break;}
case 7:
{
  add_row();
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
  isign = 0;
  make_neg = 0;
;
    break;}
case 15:
{
  if(!add_constraint_name(Last_var))
    YYABORT;
  HadConstraint = TRUE;
;
    break;}
case 17:
{
  HadVar1 = HadVar0;
  HadVar0 = FALSE;
;
    break;}
case 18:
{
  if(!store_re_op((char *) lp_yytext, HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
  make_neg = 1;
  f1 = 0;
;
    break;}
case 19:
{
  Had_lineair_sum0 = Had_lineair_sum;
  Had_lineair_sum = TRUE;
  HadVar2 = HadVar0;
  HadVar0 = FALSE;
  do_add_row = FALSE;
  if(HadConstraint && !HadVar ) {
    /* it is a range */
    /* already handled */
  }
  else if(!HadConstraint && HadVar) {
    /* it is a bound */

    if(!store_bounds(TRUE))
      YYABORT;
  }
  else {
    /* it is a row restriction */
    if(HadConstraint && HadVar)
      store_re_op("", HadConstraint, HadVar, Had_lineair_sum); /* makes sure that data stored in temporary buffers is treated correctly */
    do_add_row = TRUE;
  }
;
    break;}
case 20:
{
  if((!HadVar) && (!HadConstraint)) {
    lp_yyerror("parse error");
    YYABORT;
  }
  if(do_add_row)
    add_row();
  HadConstraint = FALSE;
  HadVar = HadVar0 = FALSE;
  isign = 0;
  make_neg = 0;
  null_tmp_store(TRUE);
;
    break;}
case 21:
{
  if((!HadVar1) && (Had_lineair_sum0))
    if(!negate_constraint())
      YYABORT;
;
    break;}
case 22:
{
  make_neg = 0;
  isign = 0;
  if(HadConstraint)
    HadVar = Had_lineair_sum = FALSE;
  HadVar0 = FALSE;
  if(!store_re_op((char *) ((*lp_yytext == '<') ? ">" : (*lp_yytext == '>') ? "<" : lp_yytext), HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
;
    break;}
case 23:
{
  f -= f1;
;
    break;}
case 24:
{
  if((HadVar1) || (!HadVar2) || (HadVar0)) {
    lp_yyerror("parse error");
    YYABORT;
  }

  if(HadConstraint && !HadVar ) {
    /* it is a range */
    /* already handled */
    if(!negate_constraint())
      YYABORT;
  }
  else if(!HadConstraint && HadVar) {
    /* it is a bound */

    if(!store_bounds(TRUE))
      YYABORT;
  }
;
    break;}
case 25:
{
  /* to allow a range */
  /* constraint: < max */
  if(!HadConstraint) {
    lp_yyerror("parse error");
    YYABORT;
  }
  Had_lineair_sum = FALSE;
;
    break;}
case 26:
{
  Had_lineair_sum = TRUE;
;
    break;}
case 28:
{
  isign = Sign;
;
    break;}
case 30:
{
  state = state0 = 0;
;
    break;}
case 31:
{
  if (state == 1) {
    /* RHS_STORE */
    if (    (isign0 || !make_neg)
        && !(isign0 && !make_neg)) /* but not both! */
      f0 = -f0;
    if(make_neg)
      f1 += f0;
    if(!rhs_store(f0, HadConstraint, HadVar, Had_lineair_sum))
      YYABORT;
  }
;
    break;}
case 34:
{
  if ((HadSign || state == 1) && (state0 == 1)) {
    /* RHS_STORE */
    if (    (isign0 || !make_neg)
        && !(isign0 && !make_neg)) /* but not both! */
      f0 = -f0;
    if(make_neg)
      f1 += f0;
    if(!rhs_store(f0, HadConstraint, HadVar, Had_lineair_sum))
      YYABORT;
  }
  if (state == 1) {
    f0 = f;
    isign0 = isign;
  }
  if (state == 2) {
    if((HadSign) || (state0 != 1)) {
     isign0 = isign;
     f0 = 1.0;
    }
    if (    (isign0 || make_neg)
        && !(isign0 && make_neg)) /* but not both! */
      f0 = -f0;
    if(!var_store(Last_var, f0, HadConstraint, HadVar, Had_lineair_sum)) {
      lp_yyerror("var_store failed");
      YYABORT;
    }
    HadConstraint |= HadVar;
    HadVar = HadVar0 = TRUE;
  }
  state0 = state;
;
    break;}
case 35:
{
  state = 1;
;
    break;}
case 36:
{
  if ((HasAR_M_OP) && (state != 1)) {
    lp_yyerror("parse error");
    YYABORT;
  }
;
    break;}
case 37:
{
  state = 2;
;
    break;}
case 41:
{
  isign = Sign;
;
    break;}
case 44:
{
  isign = 0;
  HadSign = FALSE;
;
    break;}
case 45:
{
  isign = Sign;
  HadSign = TRUE;
;
    break;}
case 46:
{
  HasAR_M_OP = FALSE;
;
    break;}
case 47:
{
  HasAR_M_OP = TRUE;
;
    break;}
case 48:
{
  if (    (isign || !make_neg)
      && !(isign && !make_neg)) /* but not both! */
    f = -f;
  if(!rhs_store(f, HadConstraint, HadVar, Had_lineair_sum))
    YYABORT;
  isign = 0;
;
    break;}
case 58:
{
  Within_sos_decl1 = Within_sos_decl;
;
    break;}
case 60:
{
  if((!Within_int_decl) && (!Within_sec_decl) && (!Within_sos_decl1) && (!Within_free_decl)) {
    lp_yyerror("parse error");
    YYABORT;
  }
  SOStype = SOStype0;
  check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, Within_sos_decl1 = (Within_sos_decl1 ? 1 : 0), Within_free_decl);
;
    break;}
case 61:
{
  if((Within_sos_decl1) && (SOStype == 0))
  {
    lp_yyerror("Unsupported SOS type (0)");
    YYABORT;
  }
;
    break;}
case 65:
{
  FREE(Last_var0);
  Last_var0 = strdup(Last_var);
;
    break;}
case 67:
{
  if(Within_sos_decl1) {
    set_sos_type(SOStype);
    set_sos_weight((double) SOSweight, 1);
  }
;
    break;}
case 68:
{
  if((Within_sos_decl1) && (!SOStype))
  {
    set_sos_type(SOStype = (short) (f + .1));
  }
  else
  {
    lp_yyerror("SOS type not expected");
    YYABORT;
  }
;
    break;}
case 70:
{
  set_sos_weight((double) SOSweight, 1);
;
    break;}
case 71:
{
  set_sos_weight(f, 1);
;
    break;}
case 78:
{
  if(Within_sos_decl1 == 1)
  {
    char buf[16];

    SOSweight++;
    sprintf(buf, "SOS%d", SOSweight);
    storevarandweight(buf);

    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;
  }

  storevarandweight(Last_var);

  if(Within_sos_decl1 == 2)
  {
    SOSNr++;
    set_sos_weight((double) SOSNr, 2);
  }
;
    break;}
case 79:
{
  if(!Within_sos_decl1) {
    lp_yyerror("parse error");
    YYABORT;
  }
  if(Within_sos_decl1 == 1) {
    FREE(Last_var0);
    Last_var0 = strdup(Last_var);
  }
  if(Within_sos_decl1 == 2)
  {
    storevarandweight(Last_var);
    SOSNr++;
    set_sos_weight((double) SOSNr, 2);
  }
;
    break;}
case 80:
{
  if(Within_sos_decl1 == 1)
  {
    char buf[16];

    SOSweight++;
    sprintf(buf, "SOS%d", SOSweight);
    storevarandweight(buf);

    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;

    storevarandweight(Last_var0);
    SOSNr++;
  }

  set_sos_weight(f, 2);
;
    break;}
case 81:
{ /* SOS name */
  if(Within_sos_decl1 == 1)
  {
    storevarandweight(Last_var0);
    set_sos_type(SOStype);
    check_int_sec_sos_free_decl(Within_int_decl, Within_sec_decl, 2, Within_free_decl);
    Within_sos_decl1 = 2;
    SOSNr = 0;
    SOSweight++;
  }
;
    break;}
}
   /* the action file gets copied in in place of this dollarsign */


  lp_yyvsp -= lp_yylen;
  lp_yyssp -= lp_yylen;
#ifdef YYLSP_NEEDED
  lp_yylsp -= lp_yylen;
#endif

#if YYDEBUG != 0
  if (lp_yydebug)
    {
      short *ssp1 = lp_yyss - 1;
      fprintf (stderr, "state stack now");
      while (ssp1 != lp_yyssp)
	fprintf (stderr, " %d", *++ssp1);
      fprintf (stderr, "\n");
    }
#endif

  *++lp_yyvsp = lp_yyval;

#ifdef YYLSP_NEEDED
  lp_yylsp++;
  if (lp_yylen == 0)
    {
      lp_yylsp->first_line = lp_yylloc.first_line;
      lp_yylsp->first_column = lp_yylloc.first_column;
      lp_yylsp->last_line = (lp_yylsp-1)->last_line;
      lp_yylsp->last_column = (lp_yylsp-1)->last_column;
      lp_yylsp->text = 0;
    }
  else
    {
      lp_yylsp->last_line = (lp_yylsp+lp_yylen-1)->last_line;
      lp_yylsp->last_column = (lp_yylsp+lp_yylen-1)->last_column;
    }
#endif

  /* Now "shift" the result of the reduction.
     Determine what state that goes to,
     based on the state we popped back to
     and the rule number reduced by.  */

  lp_yyn = lp_yyr1[lp_yyn];

  lp_yystate = lp_yypgoto[lp_yyn - YYNTBASE] + *lp_yyssp;
  if (lp_yystate >= 0 && lp_yystate <= YYLAST && lp_yycheck[lp_yystate] == *lp_yyssp)
    lp_yystate = lp_yytable[lp_yystate];
  else
    lp_yystate = lp_yydefgoto[lp_yyn - YYNTBASE];

  goto lp_yynewstate;

lp_yyerrlab:   /* here on detecting error */

  if (! lp_yyerrstatus)
    /* If not already recovering from an error, report this error.  */
    {
      ++lp_yynerrs;

#ifdef YYERROR_VERBOSE
      lp_yyn = lp_yypact[lp_yystate];

      if (lp_yyn > YYFLAG && lp_yyn < YYLAST)
	{
	  int size = 0;
	  char *msg;
	  int x, count;

	  count = 0;
	  /* Start X at -lp_yyn if nec to avoid negative indexes in lp_yycheck.  */
	  for (x = (lp_yyn < 0 ? -lp_yyn : 0);
	       x < (sizeof(lp_yytname) / sizeof(char *)); x++)
	    if (lp_yycheck[x + lp_yyn] == x)
	      size += strlen(lp_yytname[x]) + 15, count++;
	  msg = (char *) malloc(size + 15);
	  if (msg != 0)
	    {
	      strcpy(msg, "parse error");

	      if (count < 5)
		{
		  count = 0;
		  for (x = (lp_yyn < 0 ? -lp_yyn : 0);
		       x < (sizeof(lp_yytname) / sizeof(char *)); x++)
		    if (lp_yycheck[x + lp_yyn] == x)
		      {
			strcat(msg, count == 0 ? ", expecting `" : " or `");
			strcat(msg, lp_yytname[x]);
			strcat(msg, "'");
			count++;
		      }
		}
	      lp_yyerror(msg);
	      free(msg);
	    }
	  else
	    lp_yyerror ("parse error; also virtual memory exceeded");
	}
      else
#endif /* YYERROR_VERBOSE */
	lp_yyerror("parse error");
    }

  goto lp_yyerrlab1;
lp_yyerrlab1:   /* here on error raised explicitly by an action */

  if (lp_yyerrstatus == 3)
    {
      /* if just tried and failed to reuse lookahead token after an error, discard it.  */

      /* return failure if at end of input */
      if (lp_yychar == YYEOF)
	YYABORT;

#if YYDEBUG != 0
      if (lp_yydebug)
	fprintf(stderr, "Discarding token %d (%s).\n", lp_yychar, lp_yytname[lp_yychar1]);
#endif

      lp_yychar = YYEMPTY;
    }

  /* Else will try to reuse lookahead token
     after shifting the error token.  */

  lp_yyerrstatus = 3;		/* Each real token shifted decrements this */

  goto lp_yyerrhandle;

lp_yyerrdefault:  /* current state does not do anything special for the error token. */

#if 0
  /* This is wrong; only states that explicitly want error tokens
     should shift them.  */
  lp_yyn = lp_yydefact[lp_yystate];  /* If its default is to accept any token, ok.  Otherwise pop it.*/
  if (lp_yyn) goto lp_yydefault;
#endif

lp_yyerrpop:   /* pop the current state because it cannot handle the error token */

  if (lp_yyssp == lp_yyss) YYABORT;
  lp_yyvsp--;
  lp_yystate = *--lp_yyssp;
#ifdef YYLSP_NEEDED
  lp_yylsp--;
#endif

#if YYDEBUG != 0
  if (lp_yydebug)
    {
      short *ssp1 = lp_yyss - 1;
      fprintf (stderr, "Error: state stack now");
      while (ssp1 != lp_yyssp)
	fprintf (stderr, " %d", *++ssp1);
      fprintf (stderr, "\n");
    }
#endif

lp_yyerrhandle:

  lp_yyn = lp_yypact[lp_yystate];
  if (lp_yyn == YYFLAG)
    goto lp_yyerrdefault;

  lp_yyn += YYTERROR;
  if (lp_yyn < 0 || lp_yyn > YYLAST || lp_yycheck[lp_yyn] != YYTERROR)
    goto lp_yyerrdefault;

  lp_yyn = lp_yytable[lp_yyn];
  if (lp_yyn < 0)
    {
      if (lp_yyn == YYFLAG)
	goto lp_yyerrpop;
      lp_yyn = -lp_yyn;
      goto lp_yyreduce;
    }
  else if (lp_yyn == 0)
    goto lp_yyerrpop;

  if (lp_yyn == YYFINAL)
    YYACCEPT;

#if YYDEBUG != 0
  if (lp_yydebug)
    fprintf(stderr, "Shifting error token, ");
#endif

  *++lp_yyvsp = lp_yylval;
#ifdef YYLSP_NEEDED
  *++lp_yylsp = lp_yylloc;
#endif

  lp_yystate = lp_yyn;
  goto lp_yynewstate;

 lp_yyacceptlab:
  /* YYACCEPT comes here.  */
  if (lp_yyfree_stacks)
    {
      free (lp_yyss);
      free (lp_yyvs);
#ifdef YYLSP_NEEDED
      free (lp_yyls);
#endif
    }
  return 0;

 lp_yyabortlab:
  /* YYABORT comes here.  */
  if (lp_yyfree_stacks)
    {
      free (lp_yyss);
      free (lp_yyvs);
#ifdef YYLSP_NEEDED
      free (lp_yyls);
#endif
    }
  return 1;
}


static void lp_yy_delete_allocated_memory(void)
{
  /* free memory allocated by flex. Otherwise some memory is not freed.
     This is a bit tricky. There is not much documentation about this, but a lot of
     reports of memory that keeps allocated */

  /* If you get errors on this function call, just comment it. This will only result
     in some memory that is not being freed. */

# if defined YY_CURRENT_BUFFER
    /* flex defines the macro YY_CURRENT_BUFFER, so you should only get here if lp_rlp.h is
       generated by flex */
    /* lex doesn't define this macro and thus should not come here, but lex doesn't has
       this memory leak also ...*/

    lp_yy_delete_buffer(YY_CURRENT_BUFFER); /* comment this line if you have problems with it */
    lp_yy_init = 1; /* make sure that the next time memory is allocated again */
    lp_yy_start = 0;
# endif

  FREE(Last_var);
  FREE(Last_var0);
}

static int parse(void)
{
  return(lp_yyparse());
}

lprec *read_lp1(lprec *lp, void *userhandle, read_modeldata_func read_modeldata, int verbose, char *lp_name)
{
  lp_yyin = (FILE *) userhandle;
  lp_yyout = NULL;
  lp_yylineno = 1;
  lp_input = read_modeldata;
  return(yacc_read(lp, verbose, lp_name, &lp_yylineno, parse, lp_yy_delete_allocated_memory));
}

lprec * __WINAPI read_lp(FILE *filename, int verbose, char *lp_name)
{
  return(read_lp1(NULL, filename, lp_input_lp_yyin, verbose, lp_name));
}

lprec * __WINAPI read_lpex(void *userhandle, read_modeldata_func read_modeldata, int verbose, char *lp_name)
{
  return(read_lp1(NULL, userhandle, read_modeldata, verbose, lp_name));
}

lprec *read_LP1(lprec *lp, char *filename, int verbose, char *lp_name)
{
  FILE *fpin;

  if((fpin = fopen(filename, "r")) != NULL) {
    lp = read_lp1(lp, fpin, lp_input_lp_yyin, verbose, lp_name);
    fclose(fpin);
  }
  else
    lp = NULL;
  return(lp);
}

lprec * __WINAPI read_LP(char *filename, int verbose, char *lp_name)
{
  return(read_LP1(NULL, filename, verbose, lp_name));
}

MYBOOL __WINAPI LP_readhandle(lprec **lp, FILE *filename, int verbose, char *lp_name)
{
  if(lp != NULL)
    *lp = read_lp1(*lp, filename, lp_input_lp_yyin, verbose, lp_name);

  return((lp != NULL) && (*lp != NULL));
}
