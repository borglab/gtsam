#pragma ident "$Id$"

/**
 * @file SimpleKalmanFilter.cpp
 * Class to compute the solution using a Kalman filter.
 */

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008
//
//============================================================================



#include "SimpleKalmanFilter.hpp"
#include "MatrixFunctors.hpp"


namespace gpstk
{

      /* Reset method.
       *
       * This method will reset the filter, setting new values for initial
       * system state vector and the a posteriori error covariance matrix.
       *
       * @param initialState      Vector setting the initial state of
       *                          the system.
       * @param initialErrorCovariance    Matrix setting the initial
       *                   values of the a posteriori error covariance.
       */
   void SimpleKalmanFilter::Reset( const Vector<double>& initialState,
                                const Matrix<double>& initialErrorCovariance )
   {

      xhat = initialState;
      P = initialErrorCovariance;
      xhatminus.resize(initialState.size(), 0.0);
      Pminus.resize( initialErrorCovariance.rows(),
                     initialErrorCovariance.cols(), 0.0);

   }  // End of method 'SimpleKalmanFilter::Reset()'



      /* Reset method.
       *
       * This method will reset the filter, setting new values for initial
       * system  state and the a posteriori error variance. Used for
       * one-dimensional systems.
       *
       * @param initialValue      Initial value of system state.
       * @param initialErrorVariance  Initial value of the a posteriori
       *                              error variance.
       */
   void SimpleKalmanFilter::Reset( const double& initialValue,
                                   const double& initialErrorVariance )
   {

      xhat.resize(1, initialValue);
      P.resize(1,1, initialErrorVariance);
      xhatminus.resize(1, 0.0);
      Pminus.resize(1, 1, 0.0);

   }  // End of method 'SimpleKalmanFilter::Reset()'



      // Compute the a posteriori estimate of the system state, as well as
      // the a posteriori estimate error covariance matrix.
      //
      // @param phiMatrix         State transition matrix.
      // @param controlMatrix     Control matrix.
      // @param controlInput      Control input vector.
      // @param processNoiseCovariance    Process noise covariance matrix.
      // @param measurements      Measurements vector.
      // @param measurementsMatrix    Measurements matrix. Called geometry
      //                              matrix in GNSS.
      // @param measurementsNoiseCovariance   Measurements noise covariance
      //                                      matrix.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Compute( const Matrix<double>& phiMatrix,
                                    const Matrix<double>& controlMatrix,
                                    const Vector<double>& controlInput,
                                 const Matrix<double>& processNoiseCovariance,
                                    const Vector<double>& measurements,
                                    const Matrix<double>& measurementsMatrix,
                            const Matrix<double>& measurementsNoiseCovariance )
      throw(InvalidSolver)
   {

      try
      {
         Predict( phiMatrix,
                  xhat,
                  controlMatrix,
                  controlInput,
                  processNoiseCovariance );

         Correct( measurements,
                  measurementsMatrix,
                  measurementsNoiseCovariance );
      }
      catch(InvalidSolver e)
      {
         GPSTK_THROW(e);
         return -1;
      }

      return 0;

   }  // End of method 'SimpleKalmanFilter::Compute()'



      // Compute the a posteriori estimate of the system state, as well as
      // the a posteriori estimate error covariance matrix. This version
      // assumes that no control inputs act on the system.
      //
      // @param phiMatrix         State transition matrix.
      // @param processNoiseCovariance    Process noise covariance matrix.
      // @param measurements      Measurements vector.
      // @param measurementsMatrix    Measurements matrix. Called geometry
      //                              matrix in GNSS.
      // @param measurementsNoiseCovariance   Measurements noise covariance
      //                                      matrix.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Compute( const Matrix<double>& phiMatrix,
                                 const Matrix<double>& processNoiseCovariance,
                                    const Vector<double>& measurements,
                                    const Matrix<double>& measurementsMatrix,
                            const Matrix<double>& measurementsNoiseCovariance )
      throw(InvalidSolver)
   {

      try
      {
         Predict( phiMatrix,
                  xhat,
                  processNoiseCovariance );

         Correct( measurements,
                  measurementsMatrix,
                  measurementsNoiseCovariance );
      }
      catch(InvalidSolver e)
      {
         GPSTK_THROW(e);
         return -1;
      }

      return 0;

   }  // End of method 'SimpleKalmanFilter::Compute()'



      // Compute the a posteriori estimate of the system state, as well as
      // the a posteriori estimate error variance. Version for
      // one-dimensional systems.
      //
      // @param phiValue          State transition gain.
      // @param controlGain       Control gain.
      // @param controlInput      Control input value.
      // @param processNoiseVariance    Process noise variance.
      // @param measurement       Measurement value.
      // @param measurementsGain  Measurements gain.
      // @param measurementsNoiseVariance   Measurements noise variance.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Compute( const double& phiValue,
                                    const double& controlGain,
                                    const double& controlInput,
                                    const double& processNoiseVariance,
                                    const double& measurement,
                                    const double& measurementsGain,
                                    const double& measurementsNoiseVariance )
      throw(InvalidSolver)
   {

      try
      {
         Predict( phiValue,
                  xhat(0),
                  controlGain,
                  controlInput,
                  processNoiseVariance );

         Correct( measurement,
                  measurementsGain,
                  measurementsNoiseVariance );
      }
      catch(InvalidSolver e)
      {
         GPSTK_THROW(e);
         return -1;
      }

      return 0;

   }  // End of method 'SimpleKalmanFilter::Compute()'



      // Compute the a posteriori estimate of the system state, as well as
      // the a posteriori estimate error variance. Version for
      // one-dimensional systems without control input on the system.
      //
      // @param phiValue          State transition gain.
      // @param processNoiseVariance    Process noise variance.
      // @param measurement       Measurement value.
      // @param measurementsGain  Measurements gain.
      // @param measurementsNoiseVariance   Measurements noise variance.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Compute( const double& phiValue,
                                    const double& processNoiseVariance,
                                    const double& measurement,
                                    const double& measurementsGain,
                                    const double& measurementsNoiseVariance )
      throw(InvalidSolver)
   {

      try
      {
         Predict( phiValue,
                  xhat(0),
                  processNoiseVariance );

         Correct( measurement,
                  measurementsGain,
                  measurementsNoiseVariance );
      }
      catch(InvalidSolver e)
      {
         GPSTK_THROW(e);
         return -1;
      }

      return 0;

   }  // End of method 'SimpleKalmanFilter::Compute()'



      /* Predicts (or "time updates") the a priori estimate of the
       * system state, as well as the a priori estimate error variance.
       * Version for one-dimensional systems.
       *
       * @param phiValue          State transition gain.
       * @param previousState     Previous system state. It is the last
       *                          computed xhat.
       * @param controlGain       Control gain.
       * @param controlInput      Control input value.
       * @param processNoiseVariance    Process noise variance.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int SimpleKalmanFilter::Predict( const double& phiValue,
                                    const double& previousState,
                                    const double& controlGain,
                                    const double& controlInput,
                                    const double& processNoiseVariance )
      throw(InvalidSolver)
   {

         // Create dummy matrices and vectors and call the full
         // Predict() method
      Matrix<double> dummyPhiMatrix(1,1,phiValue);
      Vector<double> dummyPreviousState(1,previousState);
      Matrix<double> dummyControMatrix(1,1,controlGain);
      Vector<double> dummyControlInput(1,controlInput);
      Matrix<double> dummyProcessNoiseCovariance(1,1,processNoiseVariance);

      return ( Predict( dummyPhiMatrix,
                        dummyPreviousState,
                        dummyControMatrix,
                        dummyControlInput,
                        dummyProcessNoiseCovariance ) );

   }  // End of method 'SimpleKalmanFilter::Predict()'



      /* Predicts (or "time updates") the a priori estimate of the
       * system state, as well as the a priori estimate error
       * covariance matrix.
       * This version assumes that no control inputs act on the system.
       *
       * @param phiMatrix         State transition matrix.
       * @param previousState     Previous system state vector. It is
       *                          the last computed xhat.
       * @param processNoiseCovariance    Process noise covariance matrix.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int SimpleKalmanFilter::Predict( const Matrix<double>& phiMatrix,
                                    const Vector<double>& previousState,
                                const Matrix<double>& processNoiseCovariance )
      throw(InvalidSolver)
   {

         // Create dummy matrices and vectors and call the full
         // Predict() method
      int stateRow(previousState.size());

      Matrix<double> dummyControMatrix(stateRow,1,0.0);
      Vector<double> dummyControlInput(1,0.0);

      return ( Predict( phiMatrix,
                        previousState,
                        dummyControMatrix,
                        dummyControlInput,
                        processNoiseCovariance ) );

   }  // End of method 'SimpleKalmanFilter::Predict()'



      /* Predicts (or "time updates") the a priori estimate of the
       * system state, as well as the a priori estimate error variance.
       * Version for one-dimensional systems and no control input acting
       * on the system.
       *
       * @param phiValue          State transition gain.
       * @param previousState     Previous system state. It is the last
       *                          computed xhat.
       * @param processNoiseVariance    Process noise variance.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int SimpleKalmanFilter::Predict( const double& phiValue,
                                    const double& previousState,
                                    const double& processNoiseVariance )
      throw(InvalidSolver)
   {

         // Create dummy matrices and vectors and call the full
         // Predict() method
      Matrix<double> dummyPhiMatrix(1,1,phiValue);
      Vector<double> dummyPreviousState(1,previousState);
      Matrix<double> dummyControMatrix(1,1,0.0);
      Vector<double> dummyControlInput(1,0.0);
      Matrix<double> dummyProcessNoiseCovariance(1,1,processNoiseVariance);

      return ( Predict( dummyPhiMatrix,
                        dummyPreviousState,
                        dummyControMatrix,
                        dummyControlInput,
                        dummyProcessNoiseCovariance ) );

   }  // End of method 'SimpleKalmanFilter::Predict()'



      // Predicts (or "time updates") the a priori estimate of the system
      // state, as well as the a priori estimate error covariance matrix.
      //
      // @param phiMatrix         State transition matrix.
      // @param previousState     Previous system state vector. It is the
      //                          last computed xhat.
      // @param controlMatrix     Control matrix.
      // @param controlInput      Control input vector.
      // @param processNoiseCovariance    Process noise covariance matrix.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Predict( const Matrix<double>& phiMatrix,
                                    const Vector<double>& previousState,
                                    const Matrix<double>& controlMatrix,
                                    const Vector<double>& controlInput,
                                 const Matrix<double>& processNoiseCovariance )
      throw(InvalidSolver)
   {
         // Let's check sizes before start
      int aposterioriStateRow(xhat.size());
      int controlInputRow(controlInput.size());

      int phiCol(phiMatrix.cols());
      int phiRow(phiMatrix.rows());

      int controlCol(controlMatrix.cols());
      int controlRow(controlMatrix.rows());

      int processNoiseRow(processNoiseCovariance.rows());

      if ( phiCol != phiRow )
      {
         InvalidSolver e("Predict(): State transition matrix is not square, \
and it must be.");
         GPSTK_THROW(e);
      }

      if ( phiCol != aposterioriStateRow )
      {
         InvalidSolver e("Predict(): Sizes of state transition matrix and \
a posteriori state estimation vector do not match.");
         GPSTK_THROW(e);
      }

      if ( controlCol != controlInputRow )
      {
         InvalidSolver e("Predict(): Sizes of control matrix and a control \
input vector do not match.");
         GPSTK_THROW(e);
      }

      if ( aposterioriStateRow != controlRow )
      {
         InvalidSolver e("Predict(): Sizes of control matrix and a \
posteriori state estimation vector do not match.");
         GPSTK_THROW(e);
      }

      if ( phiRow != processNoiseRow )
      {
         InvalidSolver e("Predict(): Sizes of state transition matrix and \
process noise covariance matrix do not match.");
         GPSTK_THROW(e);
      }

         // After checking sizes, lets' do the real prediction work
      try
      {
            // Compute the a priori state vector
         xhatminus = phiMatrix*xhat + controlMatrix * controlInput;

         Matrix<double> phiT(transpose(phiMatrix));

            // Compute the a priori estimate error covariance matrix
         Pminus = phiMatrix*P*phiT + processNoiseCovariance;
      }
      catch(...)
      {
         InvalidSolver e("Predict(): Unable to predict next state.");
         GPSTK_THROW(e);
         return -1;
      }

      return 0;

   }  // End of method 'SimpleKalmanFilter::Predict()'



      // Corrects (or "measurement updates") the a posteriori estimate of
      // the system state vector, as well as the a posteriori estimate error
      // covariance matrix, using as input the predicted a priori state vector
      // and error covariance matrix, plus measurements and associated
      // matrices.
      //
      // @param measurements      Measurements vector.
      // @param measurementsMatrix    Measurements matrix. Called geometry
      //                              matrix in GNSS.
      // @param measurementsNoiseCovariance   Measurements noise covariance
      //                                      matrix.
      //
      // @return
      //  0 if OK
      //  -1 if problems arose
      //
   int SimpleKalmanFilter::Correct( const Vector<double>& measurements,
                                    const Matrix<double>& measurementsMatrix,
                           const Matrix<double>& measurementsNoiseCovariance )
      throw(InvalidSolver)
   {
         // Let's check sizes before start
      int measRow(measurements.size());
      int aprioriStateRow(xhatminus.size());

      int mMRow(measurementsMatrix.rows());

      int mNCCol(measurementsNoiseCovariance.cols());
      int mNCRow(measurementsNoiseCovariance.rows());

      int pMCol(Pminus.cols());
      int pMRow(Pminus.rows());

      if ( ( mNCCol != mNCRow ) || 
           ( pMCol != pMRow )      )
      {
         InvalidSolver e("Correct(): Either Pminus or measurement covariance \
matrices are not square, and therefore not invertible.");
         GPSTK_THROW(e);
      }

      if ( mMRow != mNCRow )
      {
         InvalidSolver e("Correct(): Sizes of measurements matrix and \
measurements noise covariance matrix do not match.");
         GPSTK_THROW(e);
      }

      if ( mNCCol != measRow )
      {
         InvalidSolver e("Correct(): Sizes of measurements matrix and \
measurements vector do not match.");
         GPSTK_THROW(e);
      }

      if ( pMCol != aprioriStateRow )
      {
         InvalidSolver e("Correct(): Sizes of a priori error covariance \
matrix and a priori state estimation vector do not match.");
         GPSTK_THROW(e);
      }

         // After checking sizes, let's do the real correction work
      Matrix<double> invR;
      Matrix<double> invPMinus;
      Matrix<double> measMatrixT( transpose(measurementsMatrix) );

      try
      {

         invR = inverseChol(measurementsNoiseCovariance);

      }
      catch(...)
      {
         InvalidSolver e("Correct(): Unable to compute invR matrix.");
         GPSTK_THROW(e);
         return -1;
      }

      try
      {

         invPMinus = inverseChol(Pminus);

      }
      catch(...)
      {
         InvalidSolver e("Correct(): Unable to compute invPMinus matrix.");
         GPSTK_THROW(e);
         return -1;
      }

      try
      {

         Matrix<double> invTemp( measMatrixT*invR*measurementsMatrix +
                                 invPMinus );

            // Compute the a posteriori error covariance matrix
         P = inverseChol( invTemp );

      }
      catch(...)
      {
         InvalidSolver e("Correct(): Unable to compute P matrix.");
         GPSTK_THROW(e);
         return -1;
      }

      try
      {

            // Compute the a posteriori state estimation
         xhat = P * ( (measMatrixT * invR * measurements) + 
                      (invPMinus * xhatminus) );

      }
      catch(Exception e)
      {
         InvalidSolver eis("Correct(): Unable to compute xhat.");
         GPSTK_THROW(eis);
         return -1;
      }

      xhatminus = xhat;
      Pminus = P;

      return 0;

   }  // End of method 'SimpleKalmanFilter::Correct()'



      /* Corrects (or "measurement updates") the a posteriori estimate
       * of the system state value, as well as the a posteriori estimate
       * error variance, using as input the predicted a priori state and
       * error variance values, plus measurement and associated gain and
       * variance. Version for one-dimensional systems.
       *
       * @param measurement       Measurement value.
       * @param measurementsGain  Measurements gain.
       * @param measurementsNoiseVariance   Measurements noise variance.
       *
       * @return
       *  0 if OK
       *  -1 if problems arose
       */
   int SimpleKalmanFilter::Correct( const double& measurement,
                                    const double& measurementsGain,
                                    const double& measurementsNoiseVariance )
      throw(InvalidSolver)
   {

         // Create dummy matrices and vectors and call the full
         // Correct() method
      Vector<double> dummyMeasurements(1,measurement);
      Matrix<double> dummyMeasurementsMatrix(1,1,measurementsGain);
      Matrix<double> dummyMeasurementsNoise(1,1,measurementsNoiseVariance);

      return ( Correct( dummyMeasurements,
                        dummyMeasurementsMatrix,
                        dummyMeasurementsNoise ) );

   }  // End of method 'SimpleKalmanFilter::Correct()'




}  // End of namespace gpstk
