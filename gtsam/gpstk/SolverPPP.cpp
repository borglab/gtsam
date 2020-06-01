//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 3.0 of the License, or
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2008, 2009, 2011
//
//============================================================================

//============================================================================
//
//This software developed by Applied Research Laboratories at the University of
//Texas at Austin, under contract to an agency or agencies within the U.S. 
//Department of Defense. The U.S. Government retains all rights to use,
//duplicate, distribute, disclose, or release this software. 
//
//Pursuant to DoD Directive 523024 
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//=============================================================================

/**
 * @file SolverPPP.cpp
 * Class to compute the PPP Solution.
 */

#include <gtsam/gpstk/SolverPPP.hpp>
#include <gtsam/gpstk/MatrixFunctors.hpp>


namespace gpstk
{

      // Index initially assigned to this class
   int SolverPPP::classIndex = 9300000;

      // Returns an index identifying this object.
   int SolverPPP::getIndex() const
   { return index; }


      // Returns a string identifying this object.
   std::string SolverPPP::getClassName() const
   { return "SolverPPP"; }


      /* Common constructor.
       *
       * @param useNEU   If true, will compute dLat, dLon, dH coordinates;
       *                 if false (the default), will compute dx, dy, dz.
       */
   SolverPPP::SolverPPP(bool useNEU)
      : firstTime(true)
   {

         // Set the equation system structure
      setNEU(useNEU);

         // Call initializing method
      Init();

   }  // End of 'SolverPPP::SolverPPP()'



      // Initializing method.
   void SolverPPP::Init(void)
   {

         // Set qdot value for default random walk stochastic model
      rwalkModel.setQprime(3e-8);

         // Pointer to default stochastic model for troposphere (random walk)
      pTropoStoModel = &rwalkModel;

         // Set default coordinates stochastic model (constant)
      setCoordinatesModel( &constantModel );

      whitenoiseModelX.setSigma(100.0);
      whitenoiseModelY.setSigma(100.0);
      whitenoiseModelZ.setSigma(100.0);

         // Pointer to default receiver clock stochastic model (white noise)
      pClockStoModel = &whitenoiseModel;

         // Pointer to stochastic model for phase biases
      pBiasStoModel  = &biasModel;

         // Set default factor that multiplies phase weights
         // If code sigma is 1 m and phase sigma is 1 cm, the ratio is 100:1
      weightFactor = 10000.0;       // 100^2


   }  // End of method 'SolverPPP::Init()'



      /* Compute the solution of the given equations set.
       *
       * @param prefitResiduals   Vector of prefit residuals
       * @param designMatrix      Design matrix for the equation system
       * @param weightVector      Vector of weights assigned to each
       *                          satellite.
       *
       * \warning A typical Kalman filter works with the measurements noise
       * covariance matrix, instead of the vector of weights. Beware of this
       * detail, because this method uses the later.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int SolverPPP::Compute( const Vector<double>& prefitResiduals,
                           const Matrix<double>& designMatrix,
                           const Vector<double>& weightVector )
      throw(InvalidSolver)
   {

         // By default, results are invalid
      valid = false;

         // First, check that everyting has a proper size
      int wSize = static_cast<int>(weightVector.size());
      int pSize = static_cast<int>(prefitResiduals.size());
      if (!(wSize==pSize))
      {
         InvalidSolver e("prefitResiduals size does not match dimension \
of weightVector");
         GPSTK_THROW(e);
      }

      Matrix<double> wMatrix(wSize,wSize,0.0);  // Declare a weight matrix

         // Fill the weight matrix diagonal with the content of
         // the weights vector
      for( int i=0; i<wSize; i++ )
      {
         wMatrix(i,i) = weightVector(i);
      }

         // Call the more general SolverPPP::Compute() method
      return SolverPPP::Compute( prefitResiduals,
                                 designMatrix,
                                 wMatrix );

   }  // End of method 'SolverPPP::Compute()'



      // Compute the solution of the given equations set.
      //
      // @param prefitResiduals   Vector of prefit residuals
      // @param designMatrix      Design matrix for equation system
      // @param weightMatrix      Matrix of weights
      //
      // \warning A typical Kalman filter works with the measurements noise
      // covariance matrix, instead of the matrix of weights. Beware of this
      // detail, because this method uses the later.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SolverPPP::Compute( const Vector<double>& prefitResiduals,
                           const Matrix<double>& designMatrix,
                           const Matrix<double>& weightMatrix )
      throw(InvalidSolver)
   {

         // By default, results are invalid
      valid = false;

      if (!(weightMatrix.isSquare()))
      {
         InvalidSolver e("Weight matrix is not square");
         GPSTK_THROW(e);
      }

      int wRow = static_cast<int>(weightMatrix.rows());
      int pRow = static_cast<int>(prefitResiduals.size());
      if (!(wRow==pRow))
      {
         InvalidSolver e("prefitResiduals size does not match dimension of \
weightMatrix");
         GPSTK_THROW(e);
      }

      int gRow = static_cast<int>(designMatrix.rows());
      if (!(gRow==pRow))
      {
         InvalidSolver e("prefitResiduals size does not match dimension \
of designMatrix");
         GPSTK_THROW(e);
      }

      if (!(phiMatrix.isSquare()))
      {
         InvalidSolver e("phiMatrix is not square");
         GPSTK_THROW(e);
      }

      int phiRow = static_cast<int>(phiMatrix.rows());
      if (!(phiRow==numUnknowns))
      {
         InvalidSolver e("Number of unknowns does not match dimension \
of phiMatrix");
         GPSTK_THROW(e);
      }

      if (!(qMatrix.isSquare()))
      {
         InvalidSolver e("qMatrix is not square");
         GPSTK_THROW(e);
      }

      int qRow = static_cast<int>(qMatrix.rows());
      if (!(qRow==numUnknowns))
      {
         InvalidSolver e("Number of unknowns does not match dimension \
of qMatrix");
         GPSTK_THROW(e);
      }

         // After checking sizes, let's invert the matrix of weights in order
         // to get the measurements noise covariance matrix, which is what we
         // use in the "SimpleKalmanFilter" class
      Matrix<double> measNoiseMatrix;

      try
      {
         measNoiseMatrix = inverseChol(weightMatrix);
      }
      catch(...)
      {
         InvalidSolver e("Correct(): Unable to compute measurements noise \
covariance matrix.");
         GPSTK_THROW(e);
      }


      try
      {

            // Call the Kalman filter object.
         kFilter.Compute( phiMatrix,
                          qMatrix,
                          prefitResiduals,
                          designMatrix,
                          measNoiseMatrix );

      }
      catch(InvalidSolver& e)
      {
         GPSTK_RETHROW(e);
      }

         // Store the solution
      solution = kFilter.xhat;

         // Store the covariance matrix of the solution
      covMatrix = kFilter.P;

         // Compute the postfit residuals Vector
      postfitResiduals = prefitResiduals - (designMatrix * solution);

         // If everything is fine so far, then the results should be valid
      valid = true;

      return 0;

   }  // End of method 'SolverPPP::Compute()'



      /* Returns a reference to a gnnsSatTypeValue object after
       * solving the previously defined equation system.
       *
       * @param gData    Data object holding the data.
       */
   gnssSatTypeValue& SolverPPP::Process(gnssSatTypeValue& gData)
      throw(ProcessingException)
   {

      try
      {

            // Build a gnssRinex object and fill it with data
         gnssRinex g1;
         g1.header = gData.header;
         g1.body = gData.body;

            // Call the Process() method with the appropriate input object
         Process(g1);

            // Update the original gnssSatTypeValue object with the results
         gData.body = g1.body;

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'SolverPPP::Process()'



      /* Returns a reference to a gnnsRinex object after solving
       * the previously defined equation system.
       *
       * @param gData     Data object holding the data.
       */
   gnssRinex& SolverPPP::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

            // Please note that there are two different sets being defined:
            //
            // - "currSatSet" stores satellites currently in view, and it is
            //   related with the number of measurements.
            //
            // - "satSet" stores satellites being processed; this set is
            //   related with the number of unknowns.
            //


            // Get a set with all satellites present in this GDS
         SatIDSet currSatSet( gData.body.getSatID() );

            // Get the number of satellites currently visible
         int numCurrentSV( gData.numSats() );

            // Update set with satellites being processed so far
         satSet.insert( currSatSet.begin(), currSatSet.end() );

            // Get the number of satellites to be processed
         int numSV( satSet.size() );

            // Number of measurements is twice the number of visible satellites
         numMeas = 2 * numCurrentSV;

            // Number of 'core' variables: Coordinates, RX clock, troposphere
         numVar = defaultEqDef.body.size();

            // Total number of unknowns is defined as variables + processed SVs
         numUnknowns = numVar + numSV;

            // State Transition Matrix (PhiMatrix)
         phiMatrix.resize(numUnknowns, numUnknowns, 0.0);

            // Noise covariance matrix (QMatrix)
         qMatrix.resize(numUnknowns, numUnknowns, 0.0);


            // Build the vector of measurements (Prefit-residuals): Code + phase
         measVector.resize(numMeas, 0.0);

         Vector<double> prefitC(gData.getVectorOfTypeID(defaultEqDef.header));
         Vector<double> prefitL(gData.getVectorOfTypeID(TypeID::prefitL));
         for( int i=0; i<numCurrentSV; i++ )
         {
            measVector( i                ) = prefitC(i);
            measVector( numCurrentSV + i ) = prefitL(i);
         }


            // Weights matrix
         rMatrix.resize(numMeas, numMeas, 0.0);

            // Generate the appropriate weights matrix
            // Try to extract weights from GDS
         satTypeValueMap dummy(gData.body.extractTypeID(TypeID::weight));

            // Check if weights match
         if ( dummy.numSats() == (size_t)numCurrentSV )
         {

               // If we have weights information, let's load it
            Vector<double>
               weightsVector(gData.getVectorOfTypeID(TypeID::weight));

            for( int i=0; i<numCurrentSV; i++ )
            {

               rMatrix( i               , i         ) = weightsVector(i);
               rMatrix( i + numCurrentSV, i + numCurrentSV )
                                             = weightsVector(i) * weightFactor;

            }  // End of 'for( int i=0; i<numCurrentSV; i++ )'

         }
         else
         {

               // If weights don't match, assign generic weights
            for( int i=0; i<numCurrentSV; i++ )
            {
               rMatrix( i               , i         ) = 1.0;

                  // Phases weights are bigger
               rMatrix( i + numCurrentSV, i + numCurrentSV )
                                                         = 1.0 * weightFactor;

            }  // End of 'for( int i=0; i<numCurrentSV; i++ )'

         }  // End of 'if ( dummy.numSats() == numCurrentSV )'



            // Generate the corresponding geometry/design matrix
         hMatrix.resize(numMeas, numUnknowns, 0.0);

            // Get the values corresponding to 'core' variables
         Matrix<double> dMatrix(gData.body.getMatrixOfTypes(defaultEqDef.body));

            // Let's fill 'hMatrix'
         for( int i=0; i<numCurrentSV; i++ )
         {

               // First, fill the coefficients related to tropo, coord and clock
            for( int j=0; j<numVar; j++ )
            {

               hMatrix( i               , j ) = dMatrix(i,j);
               hMatrix( i + numCurrentSV, j ) = dMatrix(i,j);

            }

         }  // End of 'for( int i=0; i<numCurrentSV; i++ )'


            // Now, fill the coefficients related to phase biases
            // We must be careful because not all processed satellites
            // are currently visible
         int count1(0);
         for( SatIDSet::const_iterator itSat = currSatSet.begin();
              itSat != currSatSet.end();
              ++itSat )
         {

               // Find in which position of 'satSet' is the current '(*itSat)'
               // Please note that 'currSatSet' is a subset of 'satSet'
            int j(0);
            SatIDSet::const_iterator itSat2( satSet.begin() );
            while( (*itSat2) != (*itSat) )
            {
               ++j;
               ++itSat2;
            }


               // Put coefficient in the right place
            hMatrix( count1 + numCurrentSV, j + numVar ) = 1.0;

            ++count1;

         }  // End of 'for( itSat = satSet.begin(); ... )'



            // Now, let's fill the Phi and Q matrices
         SatID  dummySat;

            // First, the troposphere
         pTropoStoModel->Prepare( dummySat,
                                  gData );
         phiMatrix(0,0) = pTropoStoModel->getPhi();
         qMatrix(0,0)   = pTropoStoModel->getQ();


            // Second, the coordinates
         pCoordXStoModel->Prepare(dummySat, gData);
         phiMatrix(1,1) = pCoordXStoModel->getPhi();
         qMatrix(1,1)   = pCoordXStoModel->getQ();

         pCoordYStoModel->Prepare(dummySat, gData);
         phiMatrix(2,2) = pCoordYStoModel->getPhi();
         qMatrix(2,2)   = pCoordYStoModel->getQ();

         pCoordZStoModel->Prepare(dummySat, gData);
         phiMatrix(3,3) = pCoordZStoModel->getPhi();
         qMatrix(3,3)   = pCoordZStoModel->getQ();


            // Third, the receiver clock
         pClockStoModel->Prepare( dummySat,
                                  gData );
         phiMatrix(4,4) = pClockStoModel->getPhi();
         qMatrix(4,4)   = pClockStoModel->getQ();


            // Finally, the phase biases
         int count2(numVar);     // Note that for PPP, 'numVar' is always 5!!!
         for( SatIDSet::const_iterator itSat = satSet.begin();
              itSat != satSet.end();
              ++itSat )
         {

               // Prepare stochastic model
            pBiasStoModel->Prepare( *itSat,
                                    gData );

               // Get values into phi and q matrices
            phiMatrix(count2,count2) = pBiasStoModel->getPhi();
            qMatrix(count2,count2)   = pBiasStoModel->getQ();

            ++count2;
         }



            // Feed the filter with the correct state and covariance matrix
         if(firstTime)
         {

            Vector<double> initialState(numUnknowns, 0.0);
            Matrix<double> initialErrorCovariance( numUnknowns,
                                                   numUnknowns,
                                                   0.0 );


               // Fill the initialErrorCovariance matrix

               // First, the zenital wet tropospheric delay
            initialErrorCovariance(0,0) = 0.25;          // (0.5 m)**2

               // Second, the coordinates
            for( int i=1; i<4; i++ )
            {
               initialErrorCovariance(i,i) = 10000.0;    // (100 m)**2
            }

               // Third, the receiver clock
            initialErrorCovariance(4,4) = 9.0e10;        // (300 km)**2

               // Finally, the phase biases
            for( int i=5; i<numUnknowns; i++ )
            {
               initialErrorCovariance(i,i) = 4.0e14;     // (20000 km)**2
            }


               // Reset Kalman filter
            kFilter.Reset( initialState, initialErrorCovariance );

               // No longer first time
            firstTime = false;

         }
         else
         {

               // Adapt the size to the current number of unknowns
            Vector<double> currentState(numUnknowns, 0.0);
            Matrix<double> currentErrorCov(numUnknowns, numUnknowns, 0.0);


               // Set first part of current state vector and covariance matrix
            for( int i=0; i<numVar; i++ )
            {
               currentState(i) = solution(i);

                  // This fills the upper left quadrant of covariance matrix
               for( int j=0; j<numVar; j++ )
               {
                  currentErrorCov(i,j) =  covMatrix(i,j);
               }
            }


               // Fill in the rest of state vector and covariance matrix
               // These are values that depend on satellites being processed
            int c1(numVar);
            for( SatIDSet::const_iterator itSat = satSet.begin();
                 itSat != satSet.end();
                 ++itSat )
            {

                  // Put ambiguities into state vector
               currentState(c1) = KalmanData[*itSat].ambiguity;

                  // Put ambiguities covariance values into covariance matrix
                  // This fills the lower right quadrant of covariance matrix
               int c2(numVar);
               SatIDSet::const_iterator itSat2;
               for( itSat2 = satSet.begin(); itSat2 != satSet.end(); ++itSat2 )
               {

                  currentErrorCov(c1,c2) = KalmanData[*itSat].aCovMap[*itSat2];
                  currentErrorCov(c2,c1) = KalmanData[*itSat].aCovMap[*itSat2];

                  ++c2;
               }

                  // Put variables X ambiguities covariances into
                  // covariance matrix. This fills the lower left and upper
                  // right quadrants of covariance matrix
               int c3(0);
               TypeIDSet::const_iterator itType;
               for( itType  = defaultEqDef.body.begin();
                    itType != defaultEqDef.body.end();
                    ++itType )
               {

                  currentErrorCov(c1,c3) = KalmanData[*itSat].vCovMap[*itType];
                  currentErrorCov(c3,c1) = KalmanData[*itSat].vCovMap[*itType];

                  ++c3;
               }

               ++c1;
            }


               // Reset Kalman filter to current state and covariance matrix
            kFilter.Reset( currentState, currentErrorCov );

         }  // End of 'if(firstTime)'



            // Call the Compute() method with the defined equation model.
            // This equation model MUST HAS BEEN previously set, usually when
            // creating the SolverPPP object with the appropriate
            // constructor.
         Compute( measVector,
                  hMatrix,
                  rMatrix );



            // Store those values of current state and covariance matrix
            // that depend on satellites currently in view
         int c1(numVar);
         for( SatIDSet::const_iterator itSat = satSet.begin();
              itSat != satSet.end();
              ++itSat )
         {

               // Store ambiguities
            KalmanData[*itSat].ambiguity = solution(c1);

               // Store ambiguities covariance values
            int c2(numVar);
            SatIDSet::const_iterator itSat2;
            for( itSat2 = satSet.begin(); itSat2 != satSet.end(); ++itSat2 )
            {

               KalmanData[*itSat].aCovMap[*itSat2] = covMatrix(c1,c2);

               ++c2;
            }

               // Store variables X ambiguities covariances
            int c3(0);
            TypeIDSet::const_iterator itType;
            for( itType  = defaultEqDef.body.begin();
                 itType != defaultEqDef.body.end();
                 ++itType )
            {

               KalmanData[*itSat].vCovMap[*itType] = covMatrix(c1,c3);

               ++c3;
            }

            ++c1;

         }  // End of 'for( itSat = satSet.begin(); ...'


            // Now we have to add the new values to the data structure
         Vector<double> postfitCode(numCurrentSV,0.0);
         Vector<double> postfitPhase(numCurrentSV,0.0);
         for( int i=0; i<numCurrentSV; i++ )
         {
            postfitCode(i)  = postfitResiduals( i                );
            postfitPhase(i) = postfitResiduals( i + numCurrentSV );
         }

         gData.insertTypeIDVector(TypeID::postfitC, postfitCode);
         gData.insertTypeIDVector(TypeID::postfitL, postfitPhase);

            // Update set of satellites to be used in next epoch
         satSet = currSatSet;

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'SolverPPP::Process()'



      /* Sets if a NEU system will be used.
       *
       * @param useNEU  Boolean value indicating if a NEU system will
       *                be used
       *
       */
   SolverPPP& SolverPPP::setNEU( bool useNEU )
   {

         // First, let's define a set with the typical code-based unknowns
      TypeIDSet tempSet;
         // Watch out here: 'tempSet' is a 'std::set', and all sets order their
         // elements. According to 'TypeID' class, this is the proper order:
      tempSet.insert(TypeID::wetMap);  // BEWARE: The first is wetMap!!!

      if (useNEU)
      {
         tempSet.insert(TypeID::dLat); // #2
         tempSet.insert(TypeID::dLon); // #3
         tempSet.insert(TypeID::dH);   // #4
      }
      else
      {
         tempSet.insert(TypeID::dx);   // #2
         tempSet.insert(TypeID::dy);   // #3
         tempSet.insert(TypeID::dz);   // #4
      }
      tempSet.insert(TypeID::cdt);     // #5

         // Now, we build the basic equation definition
      defaultEqDef.header = TypeID::prefitC;
      defaultEqDef.body = tempSet;

      return (*this);

   }  // End of method 'SolverPPP::setNEU()'



      /* Set a single coordinates stochastic model to ALL coordinates.
       *
       * @param pModel      Pointer to StochasticModel associated with
       *                    coordinates.
       *
       * @warning Do NOT use this method to set the SAME state-aware
       * stochastic model (like RandomWalkModel, for instance) to ALL
       * coordinates, because the results will certainly be erroneous. Use
       * this method only with non-state-aware stochastic models like
       * 'StochasticModel' (constant coordinates) or 'WhiteNoiseModel'.
       */
   SolverPPP& SolverPPP::setCoordinatesModel( StochasticModel* pModel )
   {

         // All coordinates will have the same model
      pCoordXStoModel = pModel;
      pCoordYStoModel = pModel;
      pCoordZStoModel = pModel;

      return (*this);

   }  // End of method 'SolverPPP::setCoordinatesModel()'


       /** Set the positioning mode, kinematic or static.
        */
   SolverPPP& SolverPPP::setKinematic( bool kinematicMode,
                                       double sigmaX,
                                       double sigmaY,
                                       double sigmaZ )
   {
      if(kinematicMode)
      {
         whitenoiseModelX.setSigma(sigmaX);
         whitenoiseModelY.setSigma(sigmaY);
         whitenoiseModelZ.setSigma(sigmaZ);

         setXCoordinatesModel(&whitenoiseModelX);
         setYCoordinatesModel(&whitenoiseModelY);
         setZCoordinatesModel(&whitenoiseModelZ);
      }
      else
      {
         setCoordinatesModel(&constantModel);
      }

      return (*this);

   }  // End of method 'SolverPPP::setKinematic()'


}  // End of namespace gpstk
