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
 * @file CodeKalmanSolver.cpp
 * Class to compute the code-based solution using a simple Kalman solver.
 */

#include <gtsam/gpstk/CodeKalmanSolver.hpp>
#include <gtsam/gpstk/MatrixFunctors.hpp>


namespace gpstk
{

      // Returns a string identifying this object.
   std::string CodeKalmanSolver::getClassName() const
   { return "CodeKalmanSolver"; }


      // Default constructor.
   CodeKalmanSolver::CodeKalmanSolver()
   {

         // First, let's define a set with the typical code-based unknowns
      TypeIDSet tempSet;
      tempSet.insert(TypeID::dx);
      tempSet.insert(TypeID::dy);
      tempSet.insert(TypeID::dz);
      tempSet.insert(TypeID::cdt);

         // Now, we build the default definition for a common GNSS 
         // code-based equation
      defaultEqDef.header = TypeID::prefitC;
      defaultEqDef.body = tempSet;

         // Call the initializing method
      Init();

   }  // End of 'CodeKalmanSolver::CodeKalmanSolver()'



      // Initializing method.
   void CodeKalmanSolver::Init()
   {

      numUnknowns = defaultEqDef.body.size();

      Vector<double> initialState(numUnknowns, 0.0);
      Matrix<double> initialErrorCovariance(numUnknowns, numUnknowns, 0.0);
         // Fill the initialErrorCovariance matrix
         // First, the coordinates
      for (int i=0; i<3; i++)
      {
         initialErrorCovariance(i,i) = 100.0;
      }
         // Now, the receiver clock
      initialErrorCovariance(3,3) = 9.0e10;

      kFilter.Reset( initialState, initialErrorCovariance );

         // Set default coordinates stochastic model (constant)
      setCoordinatesModel(&constantModel);


         // Pointer to default receiver clock stochastic model (white noise)
      pClockStoModel = &whitenoiseModel;


      solution.resize(numUnknowns);

   }  // End of method 'CodeKalmanSolver::Init()'



      /* Explicit constructor. Sets the default equation definition
       * to be used when fed with GNSS data structures.
       *
       * @param eqDef     gnssEquationDefinition to be used
       */
   CodeKalmanSolver::CodeKalmanSolver(const gnssEquationDefinition& eqDef)
   {

      setDefaultEqDefinition(eqDef);

      Init();

   }  // End of 'CodeKalmanSolver::CodeKalmanSolver()'



      /* Compute the code-based Kalman solution of the given equations set.
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
       * \warning If you use this method, be sure you previously set
       * phiMatrix and qMatrix using the appropriate methods.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int CodeKalmanSolver::Compute( const Vector<double>& prefitResiduals,
                                  const Matrix<double>& designMatrix,
                                  const Vector<double>& weightVector )
      throw(InvalidSolver)
   {

         // By default, results are invalid
      valid = false;

         // Check that everyting has a proper size
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
         // the weight vector
      for (int i=0; i<wSize; i++)
      {
         wMatrix(i,i) = weightVector(i);
      }

         // Call the more general CodeKalmanSolver::Compute() method
      return CodeKalmanSolver::Compute( prefitResiduals,
                                        designMatrix,
                                        wMatrix );

   }  // End of method 'CodeKalmanSolver::Compute()'



      // Compute the code-based Kalman solution of the given equations set.
      //
      // @param prefitResiduals   Vector of prefit residuals
      // @param designMatrix      Design matrix for equation system
      // @param weightMatrix      Matrix of weights
      //
      // \warning A typical Kalman filter works with the measurements noise
      // covariance matrix, instead of the matrix of weights. Beware of this
      // detail, because this method uses the later.
      //
      // \warning If you use this method, be sure you previously set
      // phiMatrix and qMatrix using the appropriate methods.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int CodeKalmanSolver::Compute( const Vector<double>& prefitResiduals,
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
         InvalidSolver e("prefitResiduals size does not match dimension \
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
         InvalidSolver e("prefitResiduals size does not match dimension \
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
      postfitResiduals = prefitResiduals - designMatrix * solution;

         // If everything is fine so far, then the results should be valid
      valid = true;

      return 0;

   }  // End of method 'CodeKalmanSolver::Compute()'



      /* Returns a reference to a gnnsSatTypeValue object after
       * solving the previously defined equation system.
       *
       * @param gData    Data object holding the data.
       */
   gnssSatTypeValue& CodeKalmanSolver::Process(gnssSatTypeValue& gData)
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

   }  // End of method 'CodeKalmanSolver::Process()'



      /* Returns a reference to a gnnsRinex object after solving
       * the previously defined equation system.
       *
       * @param gData     Data object holding the data.
       */
   gnssRinex& CodeKalmanSolver::Process(gnssRinex& gData)
      throw(ProcessingException)
   {

      try
      {

            // Number of measurements equals the number of visible satellites
         numMeas = gData.numSats();

            // State Transition Matrix (PhiMatrix)
         phiMatrix.resize(numUnknowns, numUnknowns, 0.0);

            // Noise covariance matrix (QMatrix)
         qMatrix.resize(numUnknowns, numUnknowns, 0.0);

            // Geometry matrix
         hMatrix.resize(numMeas, numUnknowns, 0.0);

            // Weights matrix
         rMatrix.resize(numMeas, numMeas, 0.0);

            // Measurements vector (Prefit-residuals)
         measVector.resize(numMeas, 0.0);

            // Build the vector of measurements
         measVector = gData.getVectorOfTypeID(defaultEqDef.header);


            // Generate the appropriate weights matrix
            // Try to extract weights from GDS
         satTypeValueMap dummy(gData.body.extractTypeID(TypeID::weight));

            // Count the number of satellites with weights
         int nW(dummy.numSats());
         for (int i=0; i<numMeas; i++)
         {
            if (nW == numMeas)   // Check if weights match
            {
               Vector<double>
                  weightsVector(gData.getVectorOfTypeID(TypeID::weight));

               rMatrix(i,i) = weightsVector(i);
            }
            else
            {

                 // If weights don't match, assign generic weights
               rMatrix(i,i) = 1.0;

            }
         }


            // Generate the corresponding geometry/design matrix
         hMatrix = gData.body.getMatrixOfTypes((*this).defaultEqDef.body);

         SatID  dummySat;

            // Now, let's fill the Phi and Q matrices
            // First, the coordinates
         pCoordXStoModel->Prepare(dummySat, gData);
         phiMatrix(0,0) = pCoordXStoModel->getPhi();
         qMatrix(0,0)   = pCoordXStoModel->getQ();

         pCoordYStoModel->Prepare(dummySat, gData);
         phiMatrix(1,1) = pCoordYStoModel->getPhi();
         qMatrix(1,1)   = pCoordYStoModel->getQ();

         pCoordZStoModel->Prepare(dummySat, gData);
         phiMatrix(2,2) = pCoordZStoModel->getPhi();
         qMatrix(2,2)   = pCoordZStoModel->getQ();


            // Now, the receiver clock
         pClockStoModel->Prepare(dummySat, gData);
         phiMatrix(3,3) = pClockStoModel->getPhi();
         qMatrix(3,3)   = pClockStoModel->getQ();

             // Call the Compute() method with the defined equation model.
             // This equation model MUST HAS BEEN previously set, usually
             // when creating the CodeKalmanSolver object with the
             // appropriate constructor.
         Compute(measVector, hMatrix, rMatrix);


            // Now we have to add the new values to the data structure
         if ( defaultEqDef.header == TypeID::prefitC )
         {
            gData.insertTypeIDVector(TypeID::postfitC, postfitResiduals);
         }

         return gData;

      }
      catch(Exception& u)
      {
            // Throw an exception if something unexpected happens
         ProcessingException e( getClassName() + ":"
                                + u.what() );

         GPSTK_THROW(e);

      }

   }  // End of method 'CodeKalmanSolver::Process()'



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
   CodeKalmanSolver& CodeKalmanSolver::setCoordinatesModel(
                                                   StochasticModel* pModel)
   {

         // All coordinates will have the same model
      pCoordXStoModel = pModel;
      pCoordYStoModel = pModel;
      pCoordZStoModel = pModel;

      return (*this);

   }  // End of method 'CodeKalmanSolver::setCoordinatesModel()'


}  // End of namespace gpstk
