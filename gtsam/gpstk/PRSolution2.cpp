#pragma ident "$Id$"

//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

/**
 * @file PRSolution2.cpp
 * Autonomous pseudorange navigation solution, including RAIM algorithm
 */
 
#include "MathBase.hpp"
#include "PRSolution2.hpp"
#include "EphemerisRange.hpp"
#include "GPSEllipsoid.hpp"
#include "GPSWeekSecond.hpp"

//#define DEBUG_PRINT_WARNINGS

// ----------------------------------------------------------------------------
// Combinations.hpp
// Find all the Combinations of n things taken k at a time.

#include "Exception.hpp"

/// Class Combinations will compute C(n,k), all the Combinations of n things
/// taken k at a time (where k <= n).
/// Let n 'things' be indexed by i (i=0...n-1), e.g. stored in an array of length n.
/// This class computes C(n,k) as sets of k indexes into the 'things' array.
/// These indexes are accessible via member functions Selection() or isSelected().
/// Next() computes the next combination until there are no more (when it returns -1).

class Combinations
{

public:

      /// Default constructor
   Combinations(void)
      throw()
   {
      nc = n = k = 0;
      Index = NULL;
   }
   
      /// Constructor for C(n,k) = Combinations of n things taken k at a time (k <= n)
      /// @throw on invalid input (k>n).
   Combinations(int N, int K)
      throw(gpstk::Exception)
   {
      try
      {
         init(N,K);
      }
      catch(gpstk::Exception& e)
      {
         GPSTK_RETHROW(e);
      }
   }
   
      /// copy constructor
   Combinations(const Combinations& right)
      throw()
   {
      *this = right;
   }
   
      /// destructor
   ~Combinations(void)
   {
      if (Index)
         delete[] Index;
      Index = NULL;
   }
   
      /// Assignment operator.
   Combinations& operator=(const Combinations& right)
      throw()
   {
      this->~Combinations();
      init(right.n,right.k);
      nc = right.nc;
      for (int j=0; j<k; j++)
      {
         Index[j] = right.Index[j];
      }
      return *this;
   }
   
      /// Compute the next combination, returning the number of Combinations computed
      /// so far; if there are no more Combinations, return -1.
   int Next(void) throw();
   
      /// Return index i (0 <= i < n) of jth selection (0 <= j < k);
      /// if j is out of range, return -1.
   int Selection(int j)
      throw()
   {
      if (j < 0 || j >= k)
         return -1;
      return Index[j];
   }
   
      /// Return true if the given index j (0 <= j < n) is
      /// currently selected (i.e. if j = Selection(i) for some i)
   bool isSelected(int j)
      throw()
   {
      for (int i=0; i<k; i++)
      {
         if (Index[i] == j)
            return true;
      }
      return false;
   }

private:

      /// The initialization routine used by constructors.
      /// @throw on invalid input (k>n or either n or k < 0).
   void init(int N, int K)
      throw(gpstk::Exception);
   
      /// Recursive function to increment Index[j].
   int Increment(int j) throw();
   
   int nc;         ///< number of Combinations computed so far
   int k,n;        ///< Combinations of n things taken k at a time, given at c'tor
   int* Index;     ///< Index[j] = index of jth selection (j=0...k-1; I[j]=0...n-1)

};

// ----------------------------------------------------------------------------

int Combinations::Next(void)
   throw()
{
   if (k < 1)
      return -1;
   if (Increment(k-1) != -1)
      return ++nc;
   return -1;
}

int Combinations::Increment(int j)
   throw()
{
   if (Index[j] < n-k+j) {        // can this index be incremented?
      Index[j]++;
      for (int m=j+1; m<k; m++)
      {
         Index[m]=Index[m-1]+1;
      }
      return 0;
   }
      // is this the last index?
   if (j-1 < 0) return -1;
      // increment the next lower index
   return Increment(j-1);
}

void Combinations::init(int N, int K)
   throw(gpstk::Exception)
{
   if (K > N || N < 0 || K < 0)
   {
      gpstk::Exception e("Combinations(n,k) must have k <= n, with n,k >= 0");
      GPSTK_THROW(e);
   }
   
   if (K > 0)
   {
      Index = new int[K];
      if (!Index)
      {
         gpstk::Exception e("Could not allocate");
         GPSTK_THROW(e);
      }
   }
   else
      Index = NULL;
   
   nc = 0;
   k = K;
   n = N;
   for (int j=0; j<k; j++)
   {
      Index[j]=j;
   }
}

// ----------------------------------------------------------------------------

using namespace std;
using namespace gpstk;

namespace gpstk
{
   int PRSolution2::RAIMCompute(const CommonTime& Tr,
                               vector<SatID>& Satellite,
                               const vector<double>& Pseudorange,
                               const XvtStore<SatID>& Eph,
                               TropModel *pTropModel)
      throw(Exception)
   {
      try
      {
         int iret,N,Nreject,MinSV,stage;
         size_t i, j;
         vector<bool> UseSat, UseSave;
         vector<int> GoodIndexes;
         // Use these to save the 'best' solution within the loop.
         int BestNIter=0;
         double BestRMS=0.0, BestSL=0.0, BestConv=0.0;
         Vector<double> BestSol(3,0.0);
         vector<bool> BestUse;
         BestRMS = -1.0;      // this marks the 'Best' set as unused.
	 Matrix<double> BestCovariance;

         // ----------------------------------------------------------------
         // initialize

         Valid = false;

         bool hasGlonass = false;
         bool hasOther = false;
         Vector<int> satSystems(Satellite.size());
         //Check if we have a mixed system
         for ( i = 0; i < Satellite.size(); ++i)
         {
            satSystems[i] = ((Satellite[i].system == SatID::systemGlonass) ? (1) : (0) );

            if (satSystems[i] == 1) hasGlonass = true;
            else hasOther = true;
         }

         // Save the input solution
         // (for use in rejection when ResidualCriterion is false).
         if (!hasGlonass && Solution.size() != 4)
         {
            Solution.resize(4);
         }
         else if (hasGlonass && hasOther && Solution.size() != 5)
         {
            Solution.resize(5);
         }
         Solution = 0.0;
         APrioriSolution = Solution;

         // ----------------------------------------------------------------
         // fill the SVP matrix, and use it for every solution
         // NB this routine can set Satellite[.]=negative when no ephemeris

         i = PrepareAutonomousSolution(Tr, Satellite, Pseudorange, Eph, SVP,
             Debug?pDebugStream:0);
         if(Debug && pDebugStream) {
            *pDebugStream << "In RAIMCompute after PAS(): Satellites:";
            for(j=0; j<Satellite.size(); j++) {
               RinexSatID rs(::abs(Satellite[j].id), Satellite[j].system);
               *pDebugStream << " " << (Satellite[j].id < 0 ? "-":"") << rs;
            }
            *pDebugStream << endl;
            *pDebugStream << " SVP matrix("
               << SVP.rows() << "," << SVP.cols() << ")" << endl;
            *pDebugStream << fixed << setw(16) << setprecision(3) << SVP << endl;
         }
         if (i) return i;  // return is 0 (ok) or -4 (no ephemeris)
         
         // count how many good satellites we have
         // Initialize UseSat based on Satellite, and build GoodIndexes.
         // UseSat is used to mark good sats (true) and those to ignore (false).
         // UseSave saves the original so it can be reused for each solution.
         // Let GoodIndexes be all the indexes of Satellites that are good:
         // UseSat[GoodIndexes[.]] == true by definition

         for (N=0,i=0; i<Satellite.size(); i++)
         {
            if (Satellite[i].id > 0)
            {
               N++;
               UseSat.push_back(true);
               GoodIndexes.push_back(i);
            }
            else
            {
               UseSat.push_back(false);
            }
         }
         UseSave = UseSat;
         
         // don't even try if there are not 4 good satellites

         if (!hasGlonass && N < 4)
            return -3;
         else if (hasGlonass && hasOther && N < 5)
            return -3;

         // minimum number of sats needed for algorithm
         MinSV = 5;   // this would be RAIM
         // ( not really RAIM || not RAIM at all - just one solution)
         if (!ResidualCriterion || NSatsReject==0)
            MinSV=4;
         
         // how many satellites can RAIM reject, if we have to?
         // default is -1, meaning as many as possible
         Nreject = NSatsReject;
         if (Nreject == -1 || Nreject > N-MinSV)
            Nreject=N-MinSV;
            
         // ----------------------------------------------------------------
         // now compute the solution, first with all the data. If this fails,
         // reject 1 satellite at a time and try again, then 2, etc.
         
         // Slopes for each satellite are computed (cf. the RAIM algorithm)
         Vector<double> Slopes(Pseudorange.size());
         
         // Residuals stores the post-fit data residuals.
         Vector<double> Residuals(Satellite.size(),0.0);
         
         // stage is the number of satellites to reject.
         stage = 0;
         
         do{
            // compute all the Combinations of N satellites taken stage at a time
            Combinations Combo(N,stage);
            
            // compute a solution for each combination of marked satellites
            do{
               // Mark the satellites for this combination
               UseSat = UseSave;
               for (i = 0; i < GoodIndexes.size(); i++)
               {
                  if (Combo.isSelected(i))
                     UseSat[i]=false;
               }
               // ----------------------------------------------------------------
               // Compute a solution given the data; ignore ranges for marked
               // satellites. Fill Vector 'Slopes' with slopes for each unmarked
               // satellite.
               // Return 0  ok
               //       -1  failed to converge
               //       -2  singular problem
               //       -3  not enough good data
               NIterations = MaxNIterations;             // pass limits in
               Convergence = ConvergenceLimit;
               iret = AutonomousPRSolution(Tr, UseSat, SVP, pTropModel, Algebraic,
                  NIterations, Convergence, Solution, Covariance, Residuals, Slopes,
                  pDebugStream, ((hasGlonass && hasOther)?(&satSystems):(NULL)) );
               
               // ----------------------------------------------------------------
               // Compute RMS residual...
               if (!ResidualCriterion)
               {
                  // when 'distance from a priori' is the criterion.
                  Vector<double> D=Solution-APrioriSolution;
                  RMSResidual = RMS(D);
               }
               else
               {
                  // and in the usual case
                  RMSResidual = RMS(Residuals);
               }
               // ... and find the maximum slope
               MaxSlope = 0.0;
               if (iret == 0)
               {
                  for (i=0; i<UseSat.size(); i++)
                  {
                     if (UseSat[i] && Slopes(i)>MaxSlope)
                        MaxSlope=Slopes[i];
                  }
               }
               // ----------------------------------------------------------------
               // print solution with diagnostic information
               if (Debug && pDebugStream)
               {
                  GPSWeekSecond weeksec(Tr);
                  *pDebugStream << "RPS " << setw(2) << stage
                     << " " << setw(4) << weeksec.week
                     << " " << fixed << setw(10) << setprecision(3) << weeksec.sow
                     << " " << setw(2) << N-stage << setprecision(6)
                     << " " << setw(16) << Solution(0)
                     << " " << setw(16) << Solution(1)
                     << " " << setw(16) << Solution(2)
                     << " " << setw(14) << Solution(3);
                  if (hasGlonass && hasOther)
                     *pDebugStream << " " << setw(16) << Solution(4);
                  *pDebugStream << " " << setw(12) << RMSResidual
                     << " " << fixed << setw(5) << setprecision(1) << MaxSlope
                     << " " << NIterations
                     << " " << scientific << setw(8) << setprecision(2)<< Convergence;
                     // print the SatID for good sats
                  for (i=0; i<UseSat.size(); i++)
                  {
                     if (UseSat[i])
                        *pDebugStream << " " << setw(3)<< Satellite[i].id;
                     else
                        *pDebugStream << " " << setw(3) << -::abs(Satellite[i].id);
                  }
                     // also print the return value
                  *pDebugStream << " (" << iret << ")" << endl;
               }// end debug print
               
               // ----------------------------------------------------------------
               // deal with the results of AutonomousPRSolution()
               if (iret)
               {     // failure for this combination
                  RMSResidual = 0.0;
                  Solution = 0.0;
               }
               else
               {  // success
                     // save 'best' solution for later
                  if (BestRMS < 0.0 || RMSResidual < BestRMS)
                  {
                     BestRMS = RMSResidual;
                     BestSol = Solution;
                     BestUse = UseSat;
                     BestSL = MaxSlope;
                     BestConv = Convergence;
                     BestNIter = NIterations;
                  }
                     // quit immediately?
                  if ((stage==0 || ReturnAtOnce) && RMSResidual < RMSLimit)
                     break;
               }
               
               // get the next Combinations and repeat
            } while(Combo.Next() != -1);
            
            // end of the stage
            if (BestRMS > 0.0 && BestRMS < RMSLimit)
            {     // success
               iret=0;
               break;
            }
            
            // go to next stage
            stage++;
            if (stage > Nreject)
               break;
               
         } while(iret == 0);        // end loop over stages
         
         // ----------------------------------------------------------------
         // copy out the best solution and return
         Convergence = BestConv;
         NIterations = BestNIter;
         RMSResidual = BestRMS;
         Solution = BestSol;
         MaxSlope = BestSL;
	 Covariance =BestCovariance;
         for (Nsvs=0,i=0; i<BestUse.size(); i++)
         {
            if (!BestUse[i])
               Satellite[i].id = -::abs(Satellite[i].id);
            else
               Nsvs++;
         }
         
         if (iret==0 && BestSL > SlopeLimit)
            iret=1;
         if (iret==0 && BestSL > SlopeLimit/2.0 && Nsvs == 5)
            iret=1;
         if (iret>=0 && BestRMS >= RMSLimit)
            iret=2;
            
         if (iret==0)
            Valid=true;
         
         return iret;
      }
      catch(Exception& e)
      {
         GPSTK_RETHROW(e);
      }
   }  // end PRSolution2::RAIMCompute()

///-------------------------------------------------------------------------///
///----------------------PrepareAutonomousSolution--------------------------///
///-------------------------------------------------------------------------///

   int PRSolution2::PrepareAutonomousSolution(const CommonTime& Tr,
                                             vector<SatID>& Satellite,
                                             const vector<double>& Pseudorange,
                                             const XvtStore<SatID>& Eph,
                                             Matrix<double>& SVP,
                                             ostream *pDebugStream)
      throw()
   {
      int i,j,nsvs,N=Satellite.size();
      CommonTime tx;                // transmit time
      Xvt PVT;
      
      if (N <= 0)
         return 0;
      SVP = Matrix<double>(N,4);
      SVP = 0.0;
      
      for (nsvs=0,i=0; i<N; i++)
      {
            // skip marked satellites
         if (Satellite[i].id <= 0) continue;

            // test the system
            if(Satellite[i].system == SatID::systemGPS)
               ;
            else {
               Satellite[i].id = -::abs(Satellite[i].id);
               if(pDebugStream) *pDebugStream
                  << "Warning: Ignoring satellite (system) " << Satellite[i];
               continue;
            }

         
            // first estimate of transmit time
         tx = Tr;
         tx -= Pseudorange[i]/C_MPS;
            // get ephemeris range, etc
         try
         {
            PVT = Eph.getXvt(Satellite[i], tx);
         }
         catch(InvalidRequest& e)
         {
            ///Negate SatID because getXvt failed.
            #ifdef DEBUG_PRINT_WARNINGS
               cout << "Eph.getXvt(Satellite[" << i << "], tx); threw InvalidRequest:" << endl;
               cout << e << endl << endl;
            #endif
            Satellite[i].id = -::abs(Satellite[i].id);
               if(pDebugStream) *pDebugStream
                  << "Warning: PRSolution2 ignores satellite (ephemeris) "
                  << Satellite[i] << endl;
            continue;
         }
         
            // update transmit time and get ephemeris range again
         tx -= PVT.clkbias + PVT.relcorr;     // clk+rel
         try
         {
            PVT = Eph.getXvt(Satellite[i], tx);
         }
         catch(InvalidRequest& e)
         {
            ///Negate SatID because getXvt failed.
            Satellite[i].id = -::abs(Satellite[i].id);
            continue;
         }
         
            // SVP = {SV position at transmit time}, raw range + clk + rel
         for (j=0; j<3; j++)
         {
            SVP(i,j) = PVT.x[j];
         }
         SVP(i,3) = Pseudorange[i] + C_MPS * (PVT.clkbias + PVT.relcorr);
         nsvs++;
      }
      
      if (nsvs == 0)
         return -4;
      return 0;
   } // end PrepareAutonomousPRSolution

///-------------------------------------------------------------------------///
///-------------------------------------------------------------------------///

   int PRSolution2::AlgebraicSolution(Matrix<double>& A,
                                     Vector<double>& Q,
                                     Vector<double>& X,
                                     Vector<double>& R)
   {
       try
       {
         int N=A.rows();
         Matrix<double> AT=transpose(A);
         Matrix<double> B=AT,C(4,4);
         
         C = AT * A;
         // invert
         try
         {
            //double big,small;
            //condNum(C,big,small);
            //if (small < 1.e-15 || big/small > 1.e15) return -2;
            C = inverseSVD(C);
         }
         catch(SingularMatrixException& sme)
         {
            return -2;
         }
         
         B = C * AT;
         
         Vector<double> One(N,1.0),V(4),U(4);
         double E,F,G,d,lam;
         
         U = B * One;
         V = B * Q;
         E = Minkowski(U,U);
         F = Minkowski(U,V) - 1.0;
         G = Minkowski(V,V);
         d = F*F-E*G;
            // avoid imaginary solutions: what does this really mean?
         if (d < 0.0)
            d=0.0;
         d = SQRT(d);
         
            // first solution ...
         lam = (-F+d)/E;
         X = lam*U + V;
         X(3) = -X(3);
            // ... and its residual
         R(0) = A(0,3)-X(3) - RSS(X(0)-A(0,0), X(1)-A(0,1), X(2)-A(0,2));
         
            // second solution ...
         lam = (-F-d)/E;
         X = lam*U + V;
         X(3) = -X(3);
            // ... and its residual
         R(1) = A(0,3)-X(3) - RSS(X(0)-A(0,0), X(1)-A(0,1), X(2)-A(0,2));
         
            // pick the right solution
         if (ABS(R(1)) > ABS(R(0)))
         {
            lam = (-F+d)/E;
            X = lam*U + V;
            X(3) = -X(3);
         }
         
            // compute the residuals
         for (int i=0; i<N; i++)
         {
            R(i) = A(i,3)-X(3) - RSS(X(0)-A(i,0), X(1)-A(i,1), X(2)-A(i,2));
         }
         
         return 0;
         
      }
      catch(Exception& e)
      {
         GPSTK_RETHROW(e);
      }
   }  // end PRSolution2::AlgebraicSolution

///-------------------------------------------------------------------------///
///--------------------------AutonomousPRSolution---------------------------///
///-------------------------------------------------------------------------///

   int PRSolution2::AutonomousPRSolution(const CommonTime& T,
                                        const vector<bool>& Use,
                                        const Matrix<double> SVP,
                                        TropModel *pTropModel,
                                        const bool Algebraic,
                                        int& n_iterate,
                                        double& converge,
                                        Vector<double>& Sol,
                                        Matrix<double>& Cov,
                                        Vector<double>& Resid,
                                        Vector<double>& Slope,
                                        ostream *pDebugStream,
                                        Vector<int>* satSystems)   ///Change
   throw(Exception)
   {
      if (!pTropModel)
      {
         Exception e("Undefined tropospheric model");
         GPSTK_THROW(e);
      }
      
      int size;
      if (satSystems != NULL)
         size = 5;
      else
         size = 4;
      
      try
      {
         int iret,j,n,N;
         size_t i;
         double rho,wt,svxyz[3];
         GPSEllipsoid ell;               // WGS84?
         
         //if (pDebugStream)
         //   *pDebugStream << "Enter APRS " << n_iterate << " "
         //                << scientific << setprecision(3) << converge << endl;
         
            // find the number of good satellites
         for (N=0,i=0; i<Use.size(); i++)
         {
            if (Use[i])
               N++;
         }
         if ((satSystems != NULL) && N < 5)
            return -3;
         else if (N < 4)
            return -3;
            
            // define for computation
         Vector<double> CRange(N),dX(size);
         Matrix<double> P(N,size),PT,G(size,N),PG(N,N);
         Xvt SV,RX;
         
         Sol.resize(size);
         Cov.resize(size,size);
         Resid.resize(N);
         Slope.resize(Use.size());
         
            // define for algebraic solution
         Vector<double> U(size),Q(N);
         Matrix<double> A(P);
            // and for linearized least squares
         int niter_limit = (n_iterate < 2 ? 2 : n_iterate);
         double conv_limit = converge;
         
            // prepare for iteration loop
         Sol = 0.0;                                // initial guess: center of earth
         n_iterate = 0;
         converge = 0.0;
         
            // iteration loop
            // do at least twice (even for algebraic solution) so that
            // trop model gets evaluated
      	 bool applied_trop;
         	do {
            		applied_trop = true;

               // current estimate of position solution
            for(i=0; i<3; i++) RX.x[i]=Sol(i);
            
               // loop over satellites, computing partials matrix
            for (n=0,i=0; i<Use.size(); i++)
            {
                  // ignore marked satellites
               if (!Use[i])
                  continue;
                  
                  // time of flight (sec)
               if (n_iterate == 0)
                  rho = 0.070;             // initial guess: 70ms
               else
                  rho = RSS(SVP(i,0)-Sol(0), SVP(i,1)-Sol(1), SVP(i,2)-Sol(2))
                            / ell.c();
                            
                  // correct for earth rotation
               wt = ell.angVelocity()*rho;             // radians
               svxyz[0] =  ::cos(wt)*SVP(i,0) + ::sin(wt)*SVP(i,1);
               svxyz[1] = -::sin(wt)*SVP(i,0) + ::cos(wt)*SVP(i,1);
               svxyz[2] = SVP(i,2);
               
                  // corrected pseudorange (m)
               CRange(n) = SVP(i,3);
               
                  // correct for troposphere (but not on the first iteration)
               if (n_iterate > 0)
               {
                  SV.x[0] = svxyz[0];
                  SV.x[1] = svxyz[1];
                  SV.x[2] = svxyz[2];
                  // must test RX for reasonableness to avoid corrupting TropModel
                  Position R(RX),S(SV);
                  double tc=R.getHeight(), elev = R.elevation(SV);
                  if (elev < 0.0 || tc > 10000.0 || tc < -1000) {
                     tc=0.0;
                     applied_trop = false;
                  }
                  else tc = pTropModel->correction(R,S,T);
                  //if(pDebugStream) *pDebugStream << "Trop " << i << " "
                  //   << fixed << setprecision(3) << tc << endl;
                  CRange(n) -= tc;

               }
               
                  // geometric range
               rho = RSS(svxyz[0]-Sol(0),svxyz[1]-Sol(1),svxyz[2]-Sol(2));
               
                  // partials matrix
               P(n,0) = (Sol(0)-svxyz[0])/rho;           // x direction cosine
               P(n,1) = (Sol(1)-svxyz[1])/rho;           // y direction cosine
               P(n,2) = (Sol(2)-svxyz[2])/rho;           // z direction cosine
               P(n,3) = 1.0;
               if (satSystems != NULL)
                  P(n,4) = (*satSystems)[i]; // 1 if GLONASS, 0 if GPS
               
                  // data vector: corrected range residual
               Resid(n) = CRange(n) - rho - Sol(3);
               if (satSystems != NULL && (*satSystems)[i] == 1)
               {
                  Resid(n) -= Sol(4);
               }
             
                  // TD: allow weight matrix = measurement covariance
               // P *= MCov;
               // Resid *= MCov;
               
                  // compute intermediate quantities for algebraic solution
               if (satSystems == NULL && Algebraic)
               {
                  U(0) = A(n,0) = svxyz[0];
                  U(1) = A(n,1) = svxyz[1];
                  U(2) = A(n,2) = svxyz[2];
                  U(3) = A(n,3) = CRange(n);
                  U(4) = A(n,4) = P(n,4);
                  Q(n) = 0.5 * Minkowski(U,U);
               }
               
               n++;        // n is number of good satellites - used for Slope
            }  // end loop over satellites
            
               // compute information matrix = inverse covariance matrix
            PT = transpose(P);
            Cov = PT * P;
            
               // invert using SVD
            //double big,small;
            //condNum(PT*P,big,small);
            //if (small < 1.e-15 || big/small > 1.e15)
            //   return -2;
            try
            {
               Cov = inverseSVD(Cov);
            }
            //try { Cov = inverseLUD(Cov); }
            catch(SingularMatrixException& sme)
            {
               return -2;
            }
            
               // generalized inverse
            G = Cov * PT;
            PG = P * G;                         // used for Slope
            
            n_iterate++;                        // increment number iterations
            
               // ----------------- algebraic solution -----------------------
            if (satSystems == NULL && Algebraic)
            {
               iret = PRSolution2::AlgebraicSolution(A,Q,Sol,Resid);
               if (iret)
                  return iret;                     // (singular)
               if (n_iterate > 1)
               {                       // need 2, for trop
                  iret = 0;
                  break;
               }
            }
               // ----------------- linearized least squares solution --------
            else
            {
               dX = G * Resid;
               Sol += dX;
                  // test for convergence
               converge = norm(dX);
                  // success: quit
               if (n_iterate > 1 && converge < conv_limit)
               {
                  iret = 0;
                  break;
               }
                  // failure: quit
               if (n_iterate >= niter_limit || converge > 1.e10)
               {
                  iret = -1;
                  break;
               }
            }
            
         }
	
	while(1);    // end iteration loop

         //if(!applied_trop && pDebugStream)
            //*pDebugStream << "Warning - trop correction not applied at time "
               //<< T.printf("%4F %10.3g\n");

         
            // compute slopes
         Slope = 0.0;
         if (iret == 0)
         {
            for (j=0,i=0; i<Use.size(); i++)
            {
               if (!Use[i])
                  continue;
               for (int k=0; k<4; k++)
               {
                  Slope(i) += G(k,j)*G(k,j);
               }
               Slope(i) = SQRT(Slope(i)*double(n-4)/(1.0-PG(j,j)));
               j++;
            }
         }
         
         return iret;
      } 
      catch(Exception& e)
      {
         GPSTK_RETHROW(e);
      }
   } // end PRSolution2::AutonomousPRSolution

///-------------------------------------------------------------------------///
///-------------------------------------------------------------------------///

} // namespace gpstk

