/// @file PRSolution.cpp
/// Pseudorange navigation solution, either a simple solution using all the
/// given data, or a solution including editing via a RAIM algorithm.
 
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

#include "MathBase.hpp"
#include "PRSolution.hpp"
#include "GPSEllipsoid.hpp"
#include "Combinations.hpp"
#include "TimeString.hpp"
#include "stl_helpers.hpp"
#include "logstream.hpp"

using namespace std;
using namespace gpstk;

namespace gpstk
{
   const string PRSolution::calfmt = string("%04Y/%02m/%02d %02H:%02M:%02S %P");
   const string PRSolution::gpsfmt = string("%4F %10.3g");
   const string PRSolution::timfmt = gpsfmt + string(" ") + calfmt;

   ostream& operator<<(ostream& os, const WtdAveStats& was)
      { was.dump(os,was.getMessage()); return os;}
 
   // -------------------------------------------------------------------------
   // Prepare for the autonomous solution by computing direction cosines,
   // corrected pseudoranges and satellite system.
   int PRSolution::PreparePRSolution(const CommonTime& Tr,
                                     vector<SatID>& Sats,
                                     vector<SatID::SatelliteSystem>& Syss,
                                     const vector<double>& Pseudorange,
                                     const XvtStore<SatID> *pEph,
                                     Matrix<double>& SVP) const
      throw()
   {
      LOG(DEBUG) << "PreparePRSolution at time " << printTime(Tr,timfmt);

      int j,noeph(0),N,NSVS;
      size_t i;
      CommonTime tx;
      Xvt PVT;

      // if necessary, define the SystemIDs vector (but NOT the member data one)
      if(Syss.size() == 0) {
         for(i=0; i<Sats.size(); i++) {
            SatID::SatelliteSystem sys=Sats[i].system;
            if(vectorindex(Syss, sys) == -1)     // not found
               Syss.push_back(sys);              // add it
         }
      }

      // must ignore and mark satellites if system is not found in Syss
      for(N=0,i=0; i<Sats.size(); i++) {
         if(Sats[i].id <= 0) continue;                // already marked
         if(vectorindex(Syss, Sats[i].system) == -1) {
            LOG(DEBUG) << " PRSolution ignores satellite (system) "
               << RinexSatID(Sats[i]) << " at time " << printTime(Tr,timfmt);
            Sats[i].id = -Sats[i].id;                 // don't have system - mark it
            continue;
         }
         LOG(DEBUG) << " Count sat " << RinexSatID(Sats[i]);
         ++N;                                         // count good sat
      }

      LOG(DEBUG) << "Sats.size is " << Sats.size();
      SVP = Matrix<double>(Sats.size(),4,0.0);        // define the matrix to return
      if(N <= 0) return 0;                            // nothing to do
      NSVS = 0;                                       // count good sats w/ ephem

      // loop over all satellites
      for(i=0; i<Sats.size(); i++) {

         // skip marked satellites
         if(Sats[i].id <= 0) {
            LOG(DEBUG) << " PRSolution ignores marked satellite "
               << RinexSatID(Sats[i]) << " at time " << printTime(Tr,timfmt);
            continue;
         }
         LOG(DEBUG) << " Process sat " << RinexSatID(Sats[i]);

         // first estimate of transmit time
         tx = Tr;

         // must align time systems.
         // know system of Tr, and must assume system of pEph(sat) is system(sat).
         // pEph must do calc in its sys, so must transform Tr to system(sat).
         // convert time system of tx to that of Sats[i]

         tx -= Pseudorange[i]/C_MPS;
         try {
            LOG(DEBUG) << " go to getXvt with time " << printTime(tx,timfmt);
            PVT = pEph->getXvt(Sats[i], tx);          // get ephemeris range, etc
            LOG(DEBUG) << " returned from getXvt";
         }
         catch(InvalidRequest& e) {
            LOG(DEBUG) << "Warning - PRSolution ignores satellite (no ephemeris) "
               << RinexSatID(Sats[i]) << " at time " << printTime(tx,timfmt)
               << " [" << e.getText() << "]";
            Sats[i].id = -::abs(Sats[i].id);
            ++noeph;
            continue;
         }
         catch(Exception& e) {
            LOG(DEBUG) << "Oops - Exception " << e.getText();
         }

         // update transmit time and get ephemeris range again
         tx -= PVT.clkbias + PVT.relcorr;
         try {
            PVT = pEph->getXvt(Sats[i], tx);
         }
         catch(InvalidRequest& e) {                   // unnecessary....you'd think!
            LOG(DEBUG) << "Warning - PRSolution ignores satellite (no ephemeris 2) "
               << RinexSatID(Sats[i]) << " at time " << printTime(tx,timfmt)
               << " [" << e.getText() << "]";
            Sats[i].id = -::abs(Sats[i].id);
            ++noeph;
            continue;
         }

         // SVP = {SV position at transmit time}, raw range + clk + rel
         for(j=0; j<3; j++) SVP(i,j) = PVT.x[j];
         SVP(i,3) = Pseudorange[i] + C_MPS * (PVT.clkbias + PVT.relcorr);

         LOG(DEBUG) << "SVP: Sat " << RinexSatID(Sats[i])
            << " PR " << fixed << setprecision(3) << Pseudorange[i]
            << " clkbias " << C_MPS*PVT.clkbias
            << " relcorr " << C_MPS*PVT.relcorr;

         // count the good satellites
         NSVS++;
      }

      if(noeph == N) return -4;                       // no ephemeris for any good sat

      return NSVS;
  
   } // end PreparePRPRSolution


   // -------------------------------------------------------------------------
   // Compute a straightforward solution using all the unmarked data.
   // Call PreparePRSolution first.
   int PRSolution::SimplePRSolution(const CommonTime& T,
                                    const vector<SatID>& Sats,
                                    const Matrix<double>& SVP,
                                    const Matrix<double>& invMC,
                                    TropModel *pTropModel,
                                    const int& niterLimit,
                                    const double& convLimit,
                                    const vector<SatID::SatelliteSystem>& Syss,
                                    Vector<double>& Resids,
                                    Vector<double>& Slopes)
      throw(Exception)
   {
      if(!pTropModel) {
         Exception e("Undefined tropospheric model");
         GPSTK_THROW(e);
      }
      if(Sats.size() != SVP.rows() ||
         (invMC.rows() > 0 && invMC.rows() != Sats.size())) {
         LOG(ERROR) << "Sats has length " << Sats.size();
         LOG(ERROR) << "SVP has dimension " << SVP.rows() << "x" << SVP.cols();
         LOG(ERROR) << "invMC has dimension " << invMC.rows() << "x" << invMC.cols();
         Exception e("Invalid dimensions");
         GPSTK_THROW(e);
      }
      if(Syss.size() == 0) {
         Exception e("Allowed satellite systems input (Syss) is empty!");
         GPSTK_THROW(e);
      }

      int iret(0),k,n;
      size_t i, j;
      double rho,wt,svxyz[3];
      GPSEllipsoid ellip;

      Valid = Mixed = false;

      try {
         // -----------------------------------------------------------
         // counts, systems and dimensions
         vector<SatID::SatelliteSystem> mySyss;
         {
            // define the Syss (system IDs) vector, and count good satellites
            vector<SatID::SatelliteSystem> tempSyss;
            for(Nsvs=0,i=0; i<Sats.size(); i++) {
               if(Sats[i].id <= 0)                          // reject marked sats
                  continue;

               SatID::SatelliteSystem sys(Sats[i].system);  // get this system
               if(vectorindex(Syss, sys) == -1)             // reject disallowed sys
                  continue;

               Nsvs++;                                      // count it
               if(vectorindex(tempSyss, sys) == -1)         // add unique system
                  tempSyss.push_back(sys);
            }

            // must sort as in Syss
            for(i=0; i<Syss.size(); i++)
               if(vectorindex(tempSyss, Syss[i]) != -1)
                  mySyss.push_back(Syss[i]);
         }

         // dimension of the solution vector (3 pos + 1 clk/sys)
         const int dim(3 + mySyss.size());
         if(dim > 4) Mixed = true;

         // require number of good satellites to be >= number unknowns (no RAIM here)
         if(Nsvs < dim) return -3;

         // -----------------------------------------------------------
         // build the measurement covariance matrix
         Matrix<double> iMC;
         if(invMC.rows() > 0) {
            LOG(DEBUG) << "Build inverse MCov";
            iMC = Matrix<double>(Nsvs,Nsvs,0.0);
            for(n=0,i=0; i<Sats.size(); i++) {
               if(Sats[i].id <= 0) continue;
               for(k=0,j=0; j<Sats.size(); j++) {
                  if(Sats[j].id <= 0) continue;
                  iMC(n,k) = invMC(i,j);
                  ++k;
               }
               ++n;
            }
            LOG(DEBUG) << "inv MCov matrix is\n" << fixed << setprecision(4) << iMC;
         }

         // -----------------------------------------------------------
         // define for computation
         Vector<double> CRange(Nsvs),dX(dim);
         Matrix<double> P(Nsvs,dim,0.0),PT,G(dim,Nsvs),PG(Nsvs,Nsvs),Rotation;
         Triple dirCos;
         Xvt SV,RX;

         Solution.resize(dim);
         Covariance.resize(dim,dim);
         Resids.resize(Nsvs);
         Slopes.resize(Nsvs);
         LOG(DEBUG) << " Solution dimension is " << dim << " and Nsvs is " << Nsvs;

         // prepare for iteration loop
         // iterate at least twice so that trop model gets evaluated
         int n_iterate(0), niter_limit(niterLimit < 2 ? 2 : niterLimit);
#pragma unused(niter_limit)
          double converge(0.0);

         // start with solution = apriori
         Vector<double> APSolution;
         if(hasMemory) {
            APSolution = Solution = memory.getAprioriSolution(mySyss);
            LOG(DEBUG) << " apriori solution (" << Solution.size() << ") is [ "
               << fixed << setprecision(3) << Solution << " ]";
         }
         else {
            Solution = Vector<double>(dim,0.0);
            LOG(DEBUG) << " no memory - no apriori solution";
         }

         // -----------------------------------------------------------
         // iteration loop
         do {
            TropFlag = false;       // true means the trop corr was NOT applied

            // current estimate of position solution
            RX.x = Triple(Solution(0),Solution(1),Solution(2));

            // loop over satellites, computing partials matrix
            for(n=0,i=0; i<Sats.size(); i++) {
               // ignore marked satellites
               if(Sats[i].id <= 0) continue;

               // ------------ ephemeris
               // rho is time of flight (sec)
               if(n_iterate == 0)
                  rho = 0.070;             // initial guess: 70ms
               else
                  rho = RSS(SVP(i,0)-Solution(0),
                           SVP(i,1)-Solution(1), SVP(i,2)-Solution(2))/ellip.c();

               // correct for earth rotation
               wt = ellip.angVelocity()*rho;             // radians
               svxyz[0] =  ::cos(wt)*SVP(i,0) + ::sin(wt)*SVP(i,1);
               svxyz[1] = -::sin(wt)*SVP(i,0) + ::cos(wt)*SVP(i,1);
               svxyz[2] = SVP(i,2);

               // rho is now geometric range
               rho = RSS(svxyz[0]-Solution(0),
                         svxyz[1]-Solution(1),
                         svxyz[2]-Solution(2));

               // direction cosines
               dirCos[0] = (Solution(0)-svxyz[0])/rho;
               dirCos[1] = (Solution(1)-svxyz[1])/rho;
               dirCos[2] = (Solution(2)-svxyz[2])/rho;

               // ------------ data
               // corrected pseudorange (m) minus geometric range
               CRange(n) = SVP(i,3) - rho;

               // correct for troposphere and PCOs (but not on the first iteration)
               if(n_iterate > 0) {
                  SV.x = Triple(svxyz[0],svxyz[1],svxyz[2]);
                  Position R,S;
                  R.setECEF(RX.x[0],RX.x[1],RX.x[2]);
                  S.setECEF(SV.x[0],SV.x[1],SV.x[2]);

                  // trop
                  double tc(R.getHeight());  // tc is a dummy here
                  // must test R for reasonableness to avoid corrupting TropModel
                  if(R.elevation(S) < 0.0 || tc > 100000.0 || tc < -1000.0) {
                     tc = 0.0;
                     TropFlag = true;        // true means failed to apply trop corr
                  }
                  else
                     tc = pTropModel->correction(R,S,T);    // pTropModel not const

                  CRange(n) -= tc;
                  LOG(DEBUG) << "Trop " << i << " " << Sats[i] << " "
                     << fixed << setprecision(3) << tc;

               }  // end if n_iterate > 0

               // get the index, for this clock, in the solution vector
               j = 3 + vectorindex(mySyss, Sats[i].system); // Solution ~ X,Y,Z,clks

               // find the clock for the sat's system
               const double clk(Solution(j));
               LOG(DEBUG) << "Clock is (" << j << ") " << clk;

               // data vector: corrected range residual
               Resids(n) = CRange(n) - clk;

               // ------------ least squares
               // partials matrix
               P(n,0) = dirCos[0];           // x direction cosine
               P(n,1) = dirCos[1];           // y direction cosine
               P(n,2) = dirCos[2];           // z direction cosine
               P(n,j) = 1.0;                 // clock

               // ------------ increment index
               // n is index and number of good satellites - also used for Slope
               n++;

            }  // end loop over satellites

            if(n != Nsvs) {
               Exception e("Counting error after satellite loop");
               GPSTK_THROW(e);
            }

            LOG(DEBUG) << "Partials (" << P.rows() << "x" << P.cols() << ")\n"
               << fixed << setprecision(4) << P;
            LOG(DEBUG) << "Resids (" << Resids.size() << ") "
               << fixed << setprecision(3) << Resids;

            // ------------------------------------------------------
            // compute information matrix (inverse covariance) and generalized inverse
            PT = transpose(P);

            // weight matrix = measurement covariance inverse
            if(invMC.rows() > 0) Covariance = PT * iMC * P;
            else                 Covariance = PT * P;

            // invert using SVD
            try {
               Covariance = inverseSVD(Covariance);
            }
            catch(SingularMatrixException& sme) { return -2; }
            LOG(DEBUG) << "InvCov (" << Covariance.rows() << "x" << Covariance.cols()
               << ")\n" << fixed << setprecision(4) << Covariance;

            // generalized inverse
            if(invMC.rows() > 0) G = Covariance * PT * iMC;
            else                 G = Covariance * PT;

            // PG is used for Slope computation
            PG = P * G;
            LOG(DEBUG) << "PG (" << PG.rows() << "x" << PG.cols()
               << ")\n" << fixed << setprecision(4) << PG;

            n_iterate++;                        // increment number iterations

            // ------------------------------------------------------
            // compute solution
            dX = G * Resids;
            LOG(DEBUG) << "Computed dX(" << dX.size() << ")";
            Solution += dX;

            // ------------------------------------------------------
            // test for convergence
            converge = norm(dX);
            if(n_iterate > 1 && converge < convLimit) {              // success: quit
               iret = 0;
               break;
            }
            if(n_iterate >= niterLimit || converge > 1.e10) {        // failure: quit
               iret = -1;
               break;
            }

         } while(1);    // end iteration loop
         LOG(DEBUG) << "Out of iteration loop";

         if(TropFlag) LOG(DEBUG) << "Trop correction not applied at time "
                                 << printTime(T,timfmt);

         // compute slopes and find max member
         MaxSlope = 0.0;
         Slopes = 0.0;
         if(iret == 0) for(j=0,i=0; i<Sats.size(); i++) {
            if(Sats[i].id <= 0) continue;

            // NB when one (few) sats have their own clock, PG(j,j) = 1 (nearly 1)
            // and slope is inf (large)
            if(::fabs(1.0-PG(j,j)) < 1.e-8) continue;

            for(int k=0; k<dim; k++) Slopes(j) += G(k,j)*G(k,j); // TD dim=4 here?
            Slopes(j) = SQRT(Slopes(j)*double(n-dim)/(1.0-PG(j,j)));
            if(Slopes(j) > MaxSlope) MaxSlope = Slopes(j);
            j++;
         }

         // compute pre-fit residuals
         if(hasMemory)
            PreFitResidual = P*(Solution-APSolution) - Resids;

         // Compute RMS residual (member)
         RMSResidual = RMS(Resids);

         // Find the maximum slope (member)
         //MaxSlope = 0.0;
         //for(i=0; i<Slopes.size(); i++)
         //   if(Slopes(i) > MaxSlope)
         //      MaxSlope = Slopes[i];

         // save to member data
         currTime = T;
         SatelliteIDs = Sats;
         SystemIDs = mySyss;
         invMeasCov = iMC;
         Partials = P;
         NIterations = n_iterate;
         Convergence = converge;
         Valid = true;

         return iret;
      
      } catch(Exception& e) { GPSTK_RETHROW(e); }

   } // end PRSolution::SimplePRSolution


   // -------------------------------------------------------------------------
   // Compute a solution using RAIM.
   int PRSolution::RAIMCompute(const CommonTime& Tr,
                               vector<SatID>& Sats,
                               vector<SatID::SatelliteSystem>& Syss,
                               const vector<double>& Pseudorange,
                               const Matrix<double>& invMC,
                               const XvtStore<SatID> *pEph,
                               TropModel *pTropModel)
      throw(Exception)
   {
      try {
         LOG(DEBUG) << "RAIMCompute at time " << printTime(Tr,gpsfmt);

         int iret,N;
         size_t i,j;
         vector<int> GoodIndexes;
         // use these to save the 'best' solution within the loop.
         // BestRMS marks the 'Best' set as unused.
         bool BestTropFlag(false);
         int BestNIter(0),BestIret(-5);
         double BestRMS(-1.0),BestSL(0.0),BestConv(0.0);
         Vector<double> BestSol(3,0.0),BestPFR;
         vector<SatID> BestSats,SaveSats;
         Matrix<double> SVP,BestCov,BestInvMCov,BestPartials;
         vector<SatID::SatelliteSystem> BestSyss;

         // initialize
         Valid = false;
         currTime = Tr;
         TropFlag = SlopeFlag = RMSFlag = false;

         // ----------------------------------------------------------------
         // fill the SVP matrix, and use it for every solution
         // NB this routine will: define Syss if it is empty,
         //    reject sat systems not found in Syss, and
         //    reject sats without ephemeris.
         N = PreparePRSolution(Tr, Sats, Syss, Pseudorange, pEph, SVP);

         if(LOGlevel >= ConfigureLOG::Level("DEBUG")) {
            LOG(DEBUG) << "Prepare returns " << N;
            ostringstream oss;
            oss << "RAIMCompute: after PrepareAS(): Satellites:";
            for(i=0; i<Sats.size(); i++) {
               RinexSatID rs(::abs(Sats[i].id), Sats[i].system);
               oss << " " << (Sats[i].id < 0 ? "-" : "") << rs;
            }
            oss << endl;

            oss << " SVP matrix("
               << SVP.rows() << "," << SVP.cols() << ")" << endl;
            oss << fixed << setw(16) << setprecision(5) << SVP;

            LOG(DEBUG) << oss.str();
         }

         // return is >=0(number of good sats) or -4(no ephemeris)
         if(N <= 0) return -4;

         // ----------------------------------------------------------------
         // Build GoodIndexes based on Sats; save Sats as SaveSats.
         // Sats is used to mark good sats (id > 0) and those to ignore (id <= 0).
         // SaveSats saves the original so it can be reused for each RAIM solution.
         // Let GoodIndexes be all the indexes of Sats that are good:
         // Sats[GoodIndexes[.]].id > 0 by definition.
         SaveSats = Sats;
         for(i=0; i<Sats.size(); i++)
            if(Sats[i].id > 0)
               GoodIndexes.push_back(i);

         // dump good satellites for debug
         if(LOGlevel >= ConfigureLOG::Level("DEBUG")) {
            ostringstream oss;
            oss << " Good satellites (" << N << ") are:";
            for(i=0; i<GoodIndexes.size(); i++)
               oss << " " << RinexSatID(Sats[GoodIndexes[i]]);
            LOG(DEBUG) << oss.str();
         }

         // ----------------------------------------------------------------
         // now compute the solution, first with all the data. If this fails,
         // RAIM: reject 1 satellite at a time and try again, then 2, etc.

         // Slopes for each satellite are computed (cf. the RAIM algorithm)
         Vector<double> Slopes;
         // Resids stores the post-fit data residuals.
         Vector<double> Resids;

         // stage is the number of satellites to reject.
         int stage(0);

         do {
            // compute all the combinations of N satellites taken stage at a time
            Combinations Combo(N,stage);

            // compute a solution for each combination of marked satellites
            do {
               // Mark the satellites for this combination
               Sats = SaveSats;
               for(i=0; i<GoodIndexes.size(); i++)
                  if(Combo.isSelected(i))
                     Sats[GoodIndexes[i]].id = -::abs(Sats[GoodIndexes[i]].id);

               if(LOGlevel >= ConfigureLOG::Level("DEBUG")) {
                  ostringstream oss;
                  oss << " RAIM: Try the combo ";
                  for(i=0; i<Sats.size(); i++) {
                     RinexSatID rs(::abs(Sats[i].id), Sats[i].system);
                     oss << " " << (Sats[i].id < 0 ? "-" : " ") << rs;
                  }
                  LOG(DEBUG) << oss.str();
               }

               // ----------------------------------------------------------------
               // Compute a solution given the data; ignore ranges for marked
               // satellites. Fill Vector 'Slopes' with slopes for each unmarked
               // satellite.
               // Return 0  ok
               //       -1  failed to converge
               //       -2  singular problem
               //       -3  not enough good data
               //       -4  no ephemeris
               iret = SimplePRSolution(Tr, Sats, SVP, invMC, pTropModel,
                       MaxNIterations, ConvergenceLimit, Syss, Resids, Slopes);

               LOG(DEBUG) << " RAIM: SimplePRS returns " << iret;
               if(iret <= 0 && iret > BestIret) BestIret = iret;

               // ----------------------------------------------------------------
               // if error, either quit or continue with next combo (SPS sets Valid F)
               if(iret < 0) {
                  if(iret == -1) {
                     LOG(DEBUG) << " SPS: Failed to converge - go on";
                     continue;
                  }
                  else if(iret == -2) {
                     LOG(DEBUG) << " SPS: singular - go on";
                     continue;
                  }
                  else if(iret == -3) {
                     LOG(DEBUG) <<" SPS: not enough satellites: quit";
                     break;
                  }
                  else if(iret == -4) {
                     LOG(DEBUG) <<" SPS: no ephemeris: quit";
                     break;
                  }
               }

               // ----------------------------------------------------------------
               // print solution with diagnostic information
               LOG(DEBUG) << outputString(string("RPS"),iret);

               // do again for residuals
               // if memory exists, output residuals
               //if(hasMemory) LOG(DEBUG) << outputString(string("RAP"), -99,
                     //(Solution-memory.APSolution));

               // deal with the results of SimplePRSolution()
               // save 'best' solution for later
               if(BestRMS < 0.0 || RMSResidual < BestRMS) {
                  BestRMS = RMSResidual;
                  BestSol = Solution;
                  BestSats = SatelliteIDs;
                  BestSyss = SystemIDs;
                  BestSL = MaxSlope;
                  BestConv = Convergence;
                  BestNIter = NIterations;
                  BestCov = Covariance;
                  BestInvMCov = invMeasCov;
                  BestPartials = Partials;
                  BestPFR = PreFitResidual;
                  BestTropFlag = TropFlag;
                  BestIret = iret;
               }

               if(stage==0 && RMSResidual < RMSLimit)
                  break;

            } while(Combo.Next() != -1);  // get the next combinations and repeat

            // end of the stage
            if(BestRMS > 0.0 && BestRMS < RMSLimit) {          // success
               LOG(DEBUG) << " RAIM: Success in the RAIM loop";
               iret = 0;
               break;
            }

            // go to next stage
            stage++;

            // but not if too many are being rejected
            if(NSatsReject > -1 && stage > NSatsReject) {
               LOG(DEBUG) << " RAIM: break before stage " << stage
                  << " due to NSatsReject " << NSatsReject;
               break;
            }

            // already broke out of the combo loop...
            if(iret == -3 || iret == -4) {
               LOG(DEBUG) << " RAIM: break before stage " << stage
                  << "; " << (iret==-3 ? "too few sats" : "no ephemeris");
               break;
            }

            LOG(DEBUG) << " RAIM: go to stage " << stage;

         } while(1);    // end loop over stages

         // ----------------------------------------------------------------
         // copy out the best solution and return
         if(iret >= 0) {
            Sats = SatelliteIDs = BestSats;
            SystemIDs = BestSyss;
            Solution = BestSol;
            Covariance = BestCov;
            invMeasCov = BestInvMCov;
            Partials = BestPartials;
            PreFitResidual = BestPFR;
            Convergence = BestConv;
            NIterations = BestNIter;
            RMSResidual = BestRMS;
            MaxSlope = BestSL;
            TropFlag = BestTropFlag;
            iret = BestIret;

            if(iret==0) {
               DOPCompute();        // compute DOPs
               if(hasMemory)        // update memory
                  memory.add(Solution,Covariance,PreFitResidual,Partials,invMeasCov);
            }

            // must add zeros to state, covariance and partials if these don't match
            if(Syss.size() != BestSyss.size()) {
               N = 3+Syss.size();
               Solution = Vector<double>(N,0.0);
               Covariance = Matrix<double>(N,N,0.0);
               Partials = Matrix<double>(BestPartials.rows(),N,0.0);

               // build a little map of indexes; note systems are sorted alike
               vector<int> indexes;
               for(j=0,i=0; i<Syss.size(); i++) {
                  indexes.push_back(Syss[i] == BestSyss[j] ? j++ : -1);
               }

               // copy over position and position,clk parts
               for(i=0; i<3; i++) {
                  Solution(i) = BestSol(i);
                  for(j=0; j<Partials.rows(); j++) Partials(j,i) = BestPartials(j,i);
                  for(j=0; j<3; j++) Covariance(i,j) = BestCov(i,j);
                  for(j=0; j<indexes.size(); j++) {
                     Covariance(i,3+j)=(indexes[j]==-1 ? 0.:BestCov(i,3+indexes[j]));
                     Covariance(3+j,i)=(indexes[j]==-1 ? 0.:BestCov(3+indexes[j],i));
                  }
               }

               // copy the clock part, inserting zeros where needed
               for(j=0; j<indexes.size(); j++) {
                  int n(indexes[j]);
                  bool ok(n != -1);
                  Solution(3+j) = (ok ? BestSol(3+n) : 0.0);
                  for(i=0; i<Partials.rows(); i++)
                     Partials(i,3+j) = (ok ? BestPartials(i,3+n) : 0.0);
                  for(i=0; i<indexes.size(); i++) {
                     Covariance(3+i,3+j) = (ok ? BestCov(3+i,3+n) : 0.0);
                     Covariance(3+j,3+i) = (ok ? BestCov(3+n,3+i) : 0.0);
                  }
               }
            }

            // compute number of satellites actually used
            for(Nsvs=0,i=0; i<SatelliteIDs.size(); i++)
               if(SatelliteIDs[i].id > 0)
                  Nsvs++;
         }

         if(iret==0) {
            if(BestSL > SlopeLimit) { iret = 1; SlopeFlag = true; }
            if(BestSL > SlopeLimit/2.0 && Nsvs == 5) { iret = 1; SlopeFlag = true; }
            if(BestRMS >= RMSLimit) { iret = 1; RMSFlag = true; }
            if(TropFlag) iret = 1;
            Valid = true;
         }
         else if(iret == -1) Valid = false;

         LOG(DEBUG) << " RAIM exit with ret value " << iret
                     << " and Valid " << (Valid ? "T":"F");

         return iret;
      }
      catch(Exception& e) {
         GPSTK_RETHROW(e);
      }
   }  // end PRSolution::RAIMCompute()


   // -------------------------------------------------------------------------
   int PRSolution::DOPCompute(void) throw(Exception)
   {
      try {
         Matrix<double> PTP(transpose(Partials)*Partials);
         Matrix<double> Cov(inverseLUD(PTP));
         PDOP = SQRT(Cov(0,0)+Cov(1,1)+Cov(2,2));
         TDOP = 0.0;
         for(size_t i=3; i<Cov.rows(); i++) TDOP += Cov(i,i);
         TDOP = SQRT(TDOP);
         GDOP = RSS(PDOP,TDOP);
         return 0;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }
   }

   // -------------------------------------------------------------------------
   // conveniences for printing the solutions
   string PRSolution::outputValidString(int iret) throw()
   {
      ostringstream oss;
      if(iret != -99) {
         oss << " (" << iret << " " << errorCodeString(iret);
         if(iret==1) {
            oss << " due to";
            if(RMSFlag) oss << " large RMS residual";
            if(SlopeFlag) oss << " large slope";
            if(TropFlag) oss << " missed trop. corr.";
         }
         oss << ") " << (Valid ? "" : "N") << "V";
      }
      return oss.str();
   }  // end PRSolution::outputValidString

   string PRSolution::outputNAVString(string tag, int iret, const Vector<double>& Vec)
      throw()
   {
      ostringstream oss;

      // tag NAV timetag X Y Z clks endtag
      oss << tag << " NAV " << printTime(currTime,gpsfmt)
         << fixed << setprecision(6)
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(0) : Vec(0))
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(1) : Vec(1))
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(2) : Vec(2))
         << fixed << setprecision(3);
      for(size_t i=0; i<SystemIDs.size(); i++) {
         RinexSatID sat(1,SystemIDs[i]);
         oss << " " << sat.systemString3() << " " << setw(11) << Solution(3+i);
      }
      oss << outputValidString(iret);

      return oss.str();
   }  // end PRSolution::outputNAVString

   string PRSolution::outputPOSString(string tag, int iret, const Vector<double>& Vec)
      throw()
   {
      ostringstream oss;

      // tag POS timetag X Y Z endtag
      oss << tag << " POS " << printTime(currTime,gpsfmt)
         << fixed << setprecision(6)
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(0) : Vec(0))
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(1) : Vec(1))
         << " " << setw(16) << (&Vec==&PRSNullVector ? Solution(2) : Vec(2))
         << outputValidString(iret);

      return oss.str();
   }  // end PRSolution::outputPOSString

   string PRSolution::outputCLKString(string tag, int iret) throw()
   {
      ostringstream oss;

      // tag CLK timetag SYS clk [SYS clk SYS clk ...] endtag
      oss << tag << " CLK " << printTime(currTime,gpsfmt)
         << fixed << setprecision(3);
      for(size_t i=0; i<SystemIDs.size(); i++) {
         RinexSatID sat(1,SystemIDs[i]);
         oss << " " << sat.systemString3() << " " << setw(11) << Solution(3+i);
      }
      oss << outputValidString(iret);

      return oss.str();
   }  // end PRSolution::outputCLKString

   // NB must call DOPCompute() if SimplePRSol() only was called.
   string PRSolution::outputRMSString(string tag, int iret) throw()
   {
      ostringstream oss;

      // remove duplicates and minus from satellite list
      vector<RinexSatID> sats;
      for(size_t i=0; i<SatelliteIDs.size(); i++) {
         RinexSatID rs(::abs(SatelliteIDs[i].id), SatelliteIDs[i].system);
         if(find(sats.begin(),sats.end(),rs) == sats.end())
            sats.push_back(rs);
      }

      //# tag RMS week  sec.of.wk nsat rmsres    tdop    pdop    gdop slope it
      //                converg   sats... (return) [N]V"
      oss << tag << " RMS " << printTime(currTime,gpsfmt)
         //<< " " << setw(2) << SatelliteIDs.size()-Nsvs
         << " " << setw(2) << sats.size()
         << fixed << setprecision(3)
         << " " << setw(8) << RMSResidual
         << setprecision(2)
         << " " << setw(7) << TDOP
         << " " << setw(7) << PDOP
         << " " << setw(7) << GDOP
         << setprecision(1)
         << " " << setw(5) << MaxSlope
         << " " << setw(2) << NIterations
         << scientific << setprecision(2)
         << " " << setw(8) << Convergence;
      for(size_t i=0; i<sats.size(); i++) {
         oss << " " << sats[i];
      }
      oss << outputValidString(iret);

      return oss.str();
   }  // end PRSolution::outputRMSString

   string PRSolution::outputString(string tag, int iret, const Vector<double>& Vec)
      throw()
   {
      ostringstream oss;
      oss << outputNAVString(tag,iret,Vec) << endl;
      oss << outputRMSString(tag,iret);

      return oss.str();
   }  // end PRSolution::outputString

   string PRSolution::errorCodeString(int iret) throw()
   {
      string str("unknown");
      if(iret == 1) str = string("ok but perhaps degraded");
      else if(iret== 0) str = string("ok");
      else if(iret==-1) str = string("failed to converge");
      else if(iret==-2) str = string("singular solution");
      else if(iret==-3) str = string("not enough satellites");
      else if(iret==-4) str = string("not any ephemeris");
      return str;
   }

   string PRSolution::configString(string tag) throw()
   {
      ostringstream oss;
      oss << tag << " " << printTime(currTime,timfmt)
         << (Valid ? "":" not") << " valid,"
         << (Mixed ? "":" not") << " mixed"
         << "\n   iterations " << MaxNIterations
         << "\n   convergence " << scientific << setprecision(2) << ConvergenceLimit
         << "\n   RMS residual limit " << fixed << RMSLimit
         << "\n   RAIM slope limit " << fixed << SlopeLimit << " meters"
         << "\n   Maximum number of satellites to reject is " << NSatsReject
         << "\n   Memory information IS " << (hasMemory ? "":"NOT ") << "stored"
         ;

      // TD output memory information
      //if(APrioriSol.size() >= 4) oss
      //   << "\n   APriori is " << fixed << setprecision(3) << APrioriSol(0)
      //      << " " << APrioriSol(1) << " " << APrioriSol(2) << " " << APrioriSol(3);
      //else oss << "\n   APriori is undefined";

      return oss.str();
   }

   const Vector<double> PRSolution::PRSNullVector;

} // namespace gpstk

