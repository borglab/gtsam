
#ifdef StaticMemAlloc

#if 0
  #define MAXNZ        300000
  #define MAXROWS       15000
#elif 0
  #define MAXNZ         60000
  #define MAXROWS       12500
#elif 1
  #define MAXNZ         40000
  #define MAXROWS        3500
#else
  #define MAXNZ          1000
  #define MAXROWS          50
#endif

#else

  #define MAXNZ          1000
  #define MAXROWS          50

#endif

#define MAXCOLS     MAXROWS
#define MAXLU      (LUSOL_MULT_nz_a*MAXNZ)
