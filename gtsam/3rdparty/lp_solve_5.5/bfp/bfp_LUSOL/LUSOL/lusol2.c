
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   File  lusol2 LUSOL heap management routines
   Hbuild   Hchange  Hdelete  Hdown    Hinsert  Hup
   Heap-management routines for LUSOL's lu1fac.
   May be useful for other applications.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   For LUSOL, the heap structure involves three arrays of length N.
   N        is the current number of entries in the heap.
   Ha(1:N)  contains the values that the heap is partially sorting.
            For LUSOL they are double precision values -- the largest
            element in each remaining column of the updated matrix.
            The biggest entry is in Ha(1), the top of the heap.
   Hj(1:N)  contains column numbers j.
            Ha(k) is the biggest entry in column j = Hj(k).
   Hk(1:N)  contains indices within the heap.  It is the
            inverse of Hj(1:N), so  k = Hk(j)  <=>  j = Hj(k).
            Column j is entry k in the heap.
   hops     is the number of heap operations,
            i.e., the number of times an entry is moved
            (the number of "hops" up or down the heap).
   Together, Hj and Hk let us find values inside the heap
   whenever we want to change one of the values in Ha.
   For other applications, Ha may need to be some other data type,
   like the keys that sort routines operate on.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   11 Feb 2002: MATLAB  version derived from "Algorithms" by
                R. Sedgewick
   03 Mar 2002: F77     version derived from MATLAB version.
   07 May 2002: Safeguard input parameters k, N, Nk.
                We don't want them to be output!
   07 May 2002: Current version of lusol2.f.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* ==================================================================
   Hdown  updates heap by moving down tree from node k.
   ------------------------------------------------------------------
   01 May 2002: Need Nk for length of Hk.
   05 May 2002: Change input paramter k to kk to stop k being output.
   05 May 2002: Current version of Hdown.
   ================================================================== */
void HDOWN(REAL HA[], int HJ[], int HK[], int N, int K, int *HOPS)
{
  int  J, JJ, JV, N2;
  REAL V;

  *HOPS = 0;
  V = HA[K];
  JV = HJ[K];
 N2 = N/2;
/*      while 1
        break */
x100:
  if(K>N2)
    goto x200;
  (*HOPS)++;
  J = K+K;
  if(J<N) {
    if(HA[J]<HA[J+1])
      J++;
  }
/*      break */
  if(V>=HA[J])
    goto x200;
  HA[K] = HA[J];
  JJ = HJ[J];
  HJ[K] = JJ;
  HK[JJ] = K;
  K = J;
  goto x100;
/*      end while */
x200:
  HA[K] = V;
  HJ[K] = JV;
  HK[JV] = K;
}

/* ==================================================================
   Hup updates heap by moving up tree from node k.
   ------------------------------------------------------------------
   01 May 2002: Need Nk for length of Hk.
   05 May 2002: Change input paramter k to kk to stop k being output.
   05 May 2002: Current version of Hup.
   ================================================================== */
void HUP(REAL HA[], int HJ[], int HK[], int K, int *HOPS)
{
  int  J, JV, K2;
  REAL V;

  *HOPS = 0;
  V = HA[K];
 JV = HJ[K];
/*      while 1
        break */
x100:
  if(K<2)
    goto x200;
  K2 = K/2;
/*      break */
  if(V<HA[K2])
    goto x200;
  (*HOPS)++;
  HA[K] = HA[K2];
  J = HJ[K2];
  HJ[K] = J;
  HK[J] = K;
  K = K2;
  goto x100;
/*      end while */
x200:
  HA[K] = V;
  HJ[K] = JV;
  HK[JV] = K;
}

/* ==================================================================
   Hinsert inserts (v,jv) into heap of length N-1
   to make heap of length N.
   ------------------------------------------------------------------
   03 Apr 2002: First version of Hinsert.
   01 May 2002: Require N to be final length, not old length.
                Need Nk for length of Hk.
   07 May 2002: Protect input parameters N, Nk.
   07 May 2002: Current version of Hinsert.
   ================================================================== */
void HINSERT(REAL HA[], int HJ[], int HK[], int N,
             REAL V, int JV, int *HOPS)
{
  HA[N] = V;
  HJ[N] = JV;
  HK[JV] = N;
  HUP(HA,HJ,HK,N,HOPS);
}

/* ==================================================================
   Hchange changes Ha(k) to v in heap of length N.
   ------------------------------------------------------------------
   01 May 2002: Need Nk for length of Hk.
   07 May 2002: Protect input parameters N, Nk, k.
   07 May 2002: Current version of Hchange.
   ================================================================== */
void HCHANGE(REAL HA[], int HJ[], int HK[], int N, int K,
             REAL V, int JV, int *HOPS)
{
  REAL V1;

  V1 = HA[K];
  HA[K] = V;
  HJ[K] = JV;
  HK[JV] = K;
  if(V1<V)
    HUP  (HA,HJ,HK,  K,HOPS);
  else
    HDOWN(HA,HJ,HK,N,K,HOPS);
}

/* ==================================================================
   Hdelete deletes Ha(k) from heap of length N.
   ------------------------------------------------------------------
   03 Apr 2002: Current version of Hdelete.
   01 May 2002: Need Nk for length of Hk.
   07 May 2002: Protect input parameters N, Nk, k.
   07 May 2002: Current version of Hdelete.
   ================================================================== */
void HDELETE(REAL HA[], int HJ[], int HK[], int *N, int K, int *HOPS)
{

  int  JV, NX;
  REAL V;

  NX = *N;
  V = HA[NX];
  JV = HJ[NX];
  (*N)--;
  *HOPS = 0;
  if(K<NX)
    HCHANGE(HA,HJ,HK,NX,K,V,JV,HOPS);
}

/* ==================================================================
   Hbuild initializes the heap by inserting each element of Ha.
   Input:  Ha, Hj.
   Output: Ha, Hj, Hk, hops.
   ------------------------------------------------------------------
   01 May 2002: Use k for new length of heap, not k-1 for old length.
   05 May 2002: Use kk in call to stop loop variable k being altered.
                (Actually Hinsert no longer alters that parameter.)
   07 May 2002: ftnchek wants us to protect Nk, Ha(k), Hj(k) too.
   07 May 2002: Current version of Hbuild.
   ================================================================== */
void HBUILD(REAL HA[], int HJ[], int HK[], int N, int *HOPS)
{
  int  H, JV, K, KK;
  REAL V;

  *HOPS = 0;
  for(K = 1; K <= N; K++) {
    KK = K;
    V = HA[K];
    JV = HJ[K];
    HINSERT(HA,HJ,HK,KK,V,JV,&H);
    (*HOPS) += H;
  }
}
