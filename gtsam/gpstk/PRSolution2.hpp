#pragma ident "$Id$"

/**
 * @file PRSolution2.hpp
 * Autonomous pseudorange navigation solution, including RAIM algorithm
 */
 
#ifndef PRS_POSITION_SOLUTION_NEW_HPP
#define PRS_POSITION_SOLUTION_NEW_HPP

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

#include <vector>
#include <ostream>
#include "GNSSconstants.hpp"
#include "CommonTime.hpp"
#include "SatID.hpp"
#include "Matrix.hpp"
#include "RinexObsHeader.hpp"
#include "XvtStore.hpp"
#include "TropModel.hpp"

namespace gpstk
{
   /** @defgroup GPSsolutions GPS solution algorithms and Tropospheric models */
   //@{
 
   /** This class defines an interface to routines which compute a position
    * and time solution from pseudorange data, with a data editing algorithm
    * based on Receiver Autonomous Integrity Monitoring (RAIM) concepts.
    * RAIM ref. "A Baseline GPS RAIM Scheme and a Note on the Equivalence of
    * Three RAIM Methods," by R. Grover Brown, Journal of the Institute of
    * Navigation, Vol. 39, No. 3, Fall 1992, pg 301.
    */
   class PRSolution2
   {
   public:
         /// Constructor
      PRSolution2() throw() :
         RMSLimit(6.5), SlopeLimit(1000.), Algebraic(false),
         ResidualCriterion(true), ReturnAtOnce(false), NSatsReject(-1),
         Debug(false), pDebugStream(&std::cout), MaxNIterations(10),
         ConvergenceLimit(3.e-7), Valid(false)
      {};

      /** Compute a position/time solution, given satellite PRNs and pseudoranges
       *  using a RAIM algorithm.
       * @param Tr          Measured time of reception of the data.
       * @param Satellite   std::vector<SatID> of satellites; on successful
       *                    return, satellites that were excluded by the algorithm
       *                    are marked by a negative 'prn' member.
       * @param Pseudorange std::vector<double> of raw pseudoranges (parallel to
       *                    Satellite), in meters.
       * @param Eph         gpstk::EphemerisStore to be used in the algorithm.
       *
       * @return Return values:
       *  2  solution is found, but it is not good (RMS residual exceed limits)
       *  1  solution is found, but it is suspect (slope is large)
       *  0  ok
       * -1  algorithm failed to converge
       * -2  singular problem, no solution is possible
       * -3  not enough good data (> 4) to form a (RAIM) solution
       *     (the 4 satellite solution might be returned - check isValid())
       * -4  ephemeris is not found for one or more satellites
       */
      int RAIMCompute(const CommonTime& Tr,
                      std::vector<SatID>& Satellite,
                      const std::vector<double>& Pseudorange,
                      const XvtStore<SatID>& Eph,
                      TropModel *pTropModel)
         throw(Exception);

         /// Return the status of solution
      bool isValid()
         const throw() { return Valid; }

      // input:

      /// RMS limit - either residual of fit or distance (see ResidualCriterion).
      double RMSLimit;

      /// Slope limit.
      double SlopeLimit;

      /// Use an algebraic (if true) or linearized least squares (if false) algorithm.
      bool Algebraic;

      /** Use a rejection criterion based on RMS residual of fit (true)
       * or RMS distance from an a priori position. If false, member Vector Solution
       * must be defined as this a priori position when RAIMCompute() is called.
       */
      bool ResidualCriterion;

      /** Return as soon as a solution meeting the limit requirements is found
       * (this makes it a non-RAIM algorithm).
       */
      bool ReturnAtOnce;

      /** Maximum number of satellites that may be rejected in the RAIM algorithm;
       * if this = -1, as many as possible will be rejected (RAIM requires at least 5
       * satellites). A (single) non-RAIM solution can be obtained by setting this
       * to 0 before calling RAIMCompute().
       */
      int NSatsReject;

      /// If true, RAIMCompute() will output solution information to *pDebugStream.
      bool Debug;

      /// Pointer to an ostream, default &std::cout; if Debug is true, RAIMCompute()
      /// will print all preliminary solutions to this stream.
      std::ostream *pDebugStream;

      // TD optional: measurement covariance matrix

      /// Maximum number of iterations allowed in the linearized least squares
      /// algorithm.
      int MaxNIterations;

      /// Convergence limit (m): continue iteration loop while RSS change in
      /// solution exceeds this.
      double ConvergenceLimit;

      // output:

      /// flag: output content is valid.
      bool Valid;

      /** Vector<double> containing the computed position solution (ECEF, meter);
       * valid only when isValid() is true.
       */
      Vector<double> Solution;

      /** 4x4 Matrix<double> containing the computed solution covariance (meter);
       * valid only when isValid() is true.
       */
      Matrix<double> Covariance;

      /** Root mean square residual of fit (except when RMSDistanceFlag is set,
       * then RMS distance from apriori 4-position); in meters.
       */
      double RMSResidual;

      /** Slope computed in the RAIM algorithm (largest of all satellite values)
       * for the returned solution, dimensionless ??.
       */
      double MaxSlope;

      /// the actual number of iterations used (linearized least squares algorithm)
      int NIterations;

      /// the RSS change in solution at the end of iterations.
      double Convergence;

      /// the number of good satellites used in the final computation
      int Nsvs;

      /** Compute the satellite position / corrected range matrix (SVP) which is used
       * by AutonomousPRSolution(). SVP is output, dimensioned (N,4) where N is the
       * number of satellites and the length of both Satellite and Pseudorange.
       * Data is ignored whenever Sats[i].id is < 0. NB caller should verify that the
       * number of good entries (Satellite[.] > 0) is > 4 before proceeding.
       * @param Tr          Measured time of reception of the data.
       * @param Sats        std::vector<SatID> of satellites; satellites that are
       *                    to be excluded by the algorithm are marked by a
       *                    negative 'prn' member.
       * @param Pseudorange std::vector<double> of raw pseudoranges (parallel to
       *                    Satellite), in meters
       * @param Eph         gpstk::XvtStore<SatID> to be used in the algorithm.
       * @param SVP         gpstk::Matrix<double> of dimension (N,4), N is the number
       *                    of unmarked satellites in Sats[], on output this
       *                    contains the satellite positions at transmit time, and
       *                    the corrected pseudorange.
       * @param pDebug      pointer to an ostream for debug output, NULL (the default)
       *                    for no debug output.
       * @return Return values:
       *  0  ok
       * -4  ephemeris not found for all the satellites
       */
      static int PrepareAutonomousSolution(const CommonTime& Tr,
                                           std::vector<SatID>& Sats,
                                           const std::vector<double>& Pseudorange,
                                           const XvtStore<SatID>& Eph,
                                           Matrix<double>& SVP,
                                           std::ostream *pDebug=NULL)
         throw();

      /** Compute a single autonomous pseudorange solution.
       * Input only:
       * @param Tr          Measured time of reception of the data.
       * @param Use         std::vector<bool> of length N, the number of satellites;
       *                    false means do not include that sat. in the computation.
       * @param SVP         Matrix<double> of dimension (N,4). This Matrix must have
       *                    been computed by calling PrepareAutonomousPRSolution().
       * @param Algebraic   bool flag indicating algorithm to use : algebraic (true)
       *                    or linearized least squares (false).
       * @param pTropModel  pointer to a gpstk::TropModel for use within the algorithm
       *
       *   Weight matrix TD...
       *
       * Input and output (for least squares only; ignored if Algebraic==true):
       * @param n_iterate   integer limit on the number of iterations. On output,
       *                    it is the number of iterations actually used.
       * @param converge    double convergence criterion, = RSS change in solution,
       *                    in meters. On output, it is the the final value.
       *
       * Output:  (these will be resized within the function)
       * @param Sol         gpstk::Vector<double> solution (ECEF + time components;
       *                    all in meters)
       * @param Cov         gpstk::Matrix<double> 4X4 covariance matrix (meter*meter)
       * @param Resid       gpstk::Vector<double> post-fit range residuals for each
       *                    satellite (m), the length of this Vector is the number of
       *                    satellites actually used (see Use).
       * @param Slope       gpstk::Vector<double> slope value used in RAIM for each
       *                    good satellite, length N
       * @param pDebug      pointer to an ostream for debug output, NULL (the default)
       *                    for no debug output.
       * @return Return values:
       *  0  ok
       * -1  failed to converge
       * -2  singular problem
       * -3  not enough good data to form a solution (at least 4 satellites required)
       * -4  ephemeris not found for one or more satellites
       */
      static int AutonomousPRSolution(const CommonTime& Tr,
                                      const std::vector<bool>& Use,
                                      const Matrix<double> SVP,
                                      TropModel *pTropModel,
                                      const bool Algebraic,
                                      int& n_iterate,
                                      double& converge,
                                      Vector<double>& Sol,
                                      Matrix<double>& Cov,
                                      Vector<double>& Resid,
                                      Vector<double>& Slope,
                                      std::ostream *pDebug=NULL,
                                      Vector<int>* satSystems=NULL)
            throw(Exception);

   private:

      /** Matrix, dimensioned Nx4, where N data are input, containing satellite
       * positions at transmit time (0,1,2) and raw pseudorange+clk+relativity (3).
       */
      Matrix<double> SVP;

      /// Save the input solution (for use in rejection when ResidualCriterion is
      /// false).
      Vector<double> APrioriSolution;

      /** Function used internally to handle the details of the Algebraic solution */
      static int AlgebraicSolution(Matrix<double>& A,
                                               Vector<double>& Q,
                                               Vector<double>& X,
                                               Vector<double>& R);

   }; // end class PRSolution2

   //@}

} // namespace gpstk

#endif
