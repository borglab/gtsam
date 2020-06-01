#pragma ident "$Id$"

/**
 * @file SimpleKalmanFilter.hpp
 * Class to compute the solution using a Kalman filter.
 */

#ifndef SIMPLEKALMANFILTER_HPP
#define SIMPLEKALMANFILTER_HPP

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



#include "Exception.hpp"
#include "Matrix.hpp"
#include "Vector.hpp"
#include "SolverBase.hpp"


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      /// @ingroup math

      //@{

      /** This class computes the solution using a Kalman filter.
       *
       * A typical way to use this class follows:
       *
       * @code
       *    // Declarations and initializations here...
       *
       *    SimpleKalmanFilter kalman(xhat0, pmatrix);
       *
       *    while(cin >> x >> y)
       *    {
       *
       *       try
       *       {
       *
       *          meas(0) = x;
       *          meas(1) = y;
       *
       *          kalman.Compute(phimatrix, qmatrix, meas, hmatrix, rmatrix);
       *
       *          cout << kalman.xhat(0) << " " << kalman.xhat(1) << endl;
       *       }
       *       catch (Exception e)
       *       {
       *          cout << e;
       *       }
       *    }
       * @endcode
       *
       * There is another way to use this class:
       * @code
       *       // Declarations and initializations here...
       *    SimpleKalmanFilter kalman(xhat0, pmatrix);
       *
       *       // Call this method to reset the filter
       *    kalman.Reset(initialValue,initialErrorVariance);
       *    
       *       // One time time update
       *    kalman.TimeUpdate(phiMatrix, processNoiseCovariance );
       *
       *       // Following several times measurement update
       *    kalman.MeasUpdate( measurements1, measurementsMatrix1, 
       *                                               measurementsCovariance1);
       *
       *    kalman.MeasUpdate( measurements2, measurementsMatrix2, 
       *                                               measurementsCovariance2);
       *      // Get the final solution
       *    Vector<double> x = kalman.xwhat;
       *    Matrix<double> p = kalman.P;
       *
       * @endcode
       *
       * More information about the Kalman filter may be found in the
       * excellent and easy introduction by Welch, G. and G. Bishop.
       * "An Introduction to the Kalman Filter", at:
       * http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html.
       *
       * However, be aware that the algorithm used here is the modified
       * version presented in G. J. Bierman. "Factorization Methods for
       * Discrete Sequential Estimation". Mathematics in Science and
       * Engineering, Vol. 128. Academic Press, New York, 1977. This version
       * enjoys better numerical stability.
       *
       */
   class SimpleKalmanFilter
   {
   public:

         /// Default constructor.
      SimpleKalmanFilter()
         : xhat(1,0.0), P(1,1,0.0), xhatminus(1,0.0), Pminus(1,1,0.0) {};


         /** Common constructor.
          *
          * @param initialState     Vector setting the initial state of
          *                         the system.
          * @param initialErrorCovariance    Matrix setting the initial
          *                values of the a posteriori error covariance.
          */
      SimpleKalmanFilter( const Vector<double>& initialState,
                          const Matrix<double>& initialErrorCovariance )
         : xhat(initialState), P(initialErrorCovariance),
           xhatminus(initialState.size(), 0.0),
           Pminus( initialErrorCovariance.rows(),
                   initialErrorCovariance.cols(), 0.0) {};


         /** Common constructor. This is meant to be used with one-dimensional
          * systems.
          *
          * @param initialValue      Initial value of system state.
          * @param initialErrorVariance  Initial value of the a posteriori
          *                              error variance.
          */
      SimpleKalmanFilter( const double& initialValue,
                          const double& initialErrorVariance )
         : xhat(1,initialValue),
           P(1,1,initialErrorVariance), xhatminus(1,0.0),
           Pminus(1,1,0.0) {};


         /** Reset method.
          *
          * This method will reset the filter, setting new values for initial
          * system state vector and the a posteriori error covariance matrix.
          *
          * @param initialState      Vector setting the initial state of
          *                          the system.
          * @param initialErrorCovariance    Matrix setting the initial
          *                   values of the a posteriori error covariance.
          */
      virtual void Reset( const Vector<double>& initialState,
                          const Matrix<double>& initialErrorCovariance );


         /** Reset method.
          *
          * This method will reset the filter, setting new values for initial
          * system  state and the a posteriori error variance. Used for
          * one-dimensional systems.
          *
          * @param initialValue      Initial value of system state.
          * @param initialErrorVariance  Initial value of the a posteriori
          *                              error variance.
          */
      virtual void Reset( const double& initialValue,
                          const double& initialErrorVariance );


         /** Compute the a posteriori estimate of the system state, as well
          *  as the a posteriori estimate error covariance matrix.
          *
          * @param phiMatrix         State transition matrix.
          * @param controlMatrix     Control matrix.
          * @param controlInput      Control input vector.
          * @param processNoiseCovariance    Process noise covariance matrix.
          * @param measurements      Measurements vector.
          * @param measurementsMatrix    Measurements matrix. Called geometry
          *                              matrix in GNSS.
          * @param measurementsNoiseCovariance   Measurements noise covariance
          *                                      matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Compute( const Matrix<double>& phiMatrix,
                           const Matrix<double>& controlMatrix,
                           const Vector<double>& controlInput,
                           const Matrix<double>& processNoiseCovariance,
                           const Vector<double>& measurements,
                           const Matrix<double>& measurementsMatrix,
                           const Matrix<double>& measurementsNoiseCovariance )
         throw(InvalidSolver);


         /** Compute the a posteriori estimate of the system state, as well
          *  as the a posteriori estimate error covariance matrix. This
          *  version assumes that no control inputs act on the system.
          *
          * @param phiMatrix         State transition matrix.
          * @param processNoiseCovariance    Process noise covariance matrix.
          * @param measurements      Measurements vector.
          * @param measurementsMatrix    Measurements matrix. Called geometry
          *                              matrix in GNSS.
          * @param measurementsNoiseCovariance   Measurements noise covariance
          *                                      matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Compute( const Matrix<double>& phiMatrix,
                           const Matrix<double>& processNoiseCovariance,
                           const Vector<double>& measurements,
                           const Matrix<double>& measurementsMatrix,
                           const Matrix<double>& measurementsNoiseCovariance )
         throw(InvalidSolver);


         /** Compute the a posteriori estimate of the system state, as well
          *  as the a posteriori estimate error variance. Version for
          *  one-dimensional systems.
          *
          * @param phiValue          State transition gain.
          * @param controlGain       Control gain.
          * @param controlInput      Control input value.
          * @param processNoiseVariance    Process noise variance.
          * @param measurement       Measurement value.
          * @param measurementsGain  Measurements gain.
          * @param measurementsNoiseVariance   Measurements noise variance.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Compute( const double& phiValue,
                           const double& controlGain,
                           const double& controlInput,
                           const double& processNoiseVariance,
                           const double& measurement,
                           const double& measurementsGain,
                           const double& measurementsNoiseVariance )
         throw(InvalidSolver);


         /** Compute the a posteriori estimate of the system state, as well
          *  as the a posteriori estimate error variance. Version for
          *  one-dimensional systems without control input on the system.
          *
          * @param phiValue          State transition gain.
          * @param processNoiseVariance    Process noise variance.
          * @param measurement       Measurement value.
          * @param measurementsGain  Measurements gain.
          * @param measurementsNoiseVariance   Measurements noise variance.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Compute( const double& phiValue,
                           const double& processNoiseVariance,
                           const double& measurement,
                           const double& measurementsGain,
                           const double& measurementsNoiseVariance )
         throw(InvalidSolver);


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error
          *  covariance matrix.
          *  This version assumes that the phiMatrix is indedify matrix, 
          *  previousState is the last state of the filter, no process
          *  noise, and no control inputs act on the system.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int TimeUpdate()
      { xhatminus=xhat;Pminus=P; return 0;}


      /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error
          *  covariance matrix.
          *  This version assumes that no control inputs act on the system.
          *
          * @param phiMatrix         State transition matrix.
          * @param processNoiseCovariance    Process noise covariance matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int TimeUpdate( const Matrix<double>& phiMatrix,
                              const Matrix<double>& processNoiseCovariance )
         throw(InvalidSolver)
      { return Predict(phiMatrix,xhat,processNoiseCovariance); }


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error
          *  covariance matrix.
          *  This version assumes that no control inputs act on the system.
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
      virtual int TimeUpdate( const Matrix<double>& phiMatrix,
                              const Vector<double>& previousState,
                              const Matrix<double>& processNoiseCovariance )
         throw(InvalidSolver)
      { return Predict(phiMatrix,previousState,processNoiseCovariance); }



         /** Corrects (or "measurement updates") the a posteriori estimate
          *  of the system state vector, as well as the a posteriori estimate
          *  error covariance matrix, using as input the predicted a priori
          *  state vector and error covariance matrix, plus measurements and
          *  associated matrices.
          *
          * @param measurements      Measurements vector.
          * @param measurementsMatrix    Measurements matrix. Called geometry
          *                              matrix in GNSS.
          * @param measurementsNoiseCovariance   Measurements noise covariance
          *                                      matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int MeasUpdate( const Vector<double>& measurements,
                              const Matrix<double>& measurementsMatrix,
                              const Matrix<double>& measurementsNoiseCovariance)
         throw(InvalidSolver)
      { 
         return Correct(measurements,
                        measurementsMatrix,
                        measurementsNoiseCovariance); 
      }


         /// Destructor.
      virtual ~SimpleKalmanFilter() {};


         /// A posteriori state estimation. This is usually your target.
      Vector<double> xhat;


         /// A posteriori error covariance.
      Matrix<double> P;


         /// A priori state estimation.
      Vector<double> xhatminus;


         /// A priori error covariance.
      Matrix<double> Pminus;


   private:


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error covariance
          *  matrix.
          *
          * @param phiMatrix         State transition matrix.
          * @param previousState     Previous system state vector. It is the
          *                          last computed xhat.
          * @param controlMatrix     Control matrix.
          * @param controlInput      Control input vector.
          * @param processNoiseCovariance    Process noise covariance matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Predict( const Matrix<double>& phiMatrix,
                           const Vector<double>& previousState,
                           const Matrix<double>& controlMatrix,
                           const Vector<double>& controlInput,
                           const Matrix<double>& processNoiseCovariance )
         throw(InvalidSolver);


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error variance.
          *  Version for one-dimensional systems.
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
      virtual int Predict( const double& phiValue,
                           const double& previousState,
                           const double& controlGain,
                           const double& controlInput,
                           const double& processNoiseVariance )
         throw(InvalidSolver);


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error
          *  covariance matrix.
          *  This version assumes that no control inputs act on the system.
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
      virtual int Predict( const Matrix<double>& phiMatrix,
                           const Vector<double>& previousState,
                           const Matrix<double>& processNoiseCovariance )
         throw(InvalidSolver);


         /** Predicts (or "time updates") the a priori estimate of the
          *  system state, as well as the a priori estimate error variance.
          *  Version for one-dimensional systems and no control input acting
          *  on the system.
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
      virtual int Predict( const double& phiValue,
                           const double& previousState,
                           const double& processNoiseVariance )
         throw(InvalidSolver);


         /** Corrects (or "measurement updates") the a posteriori estimate
          *  of the system state vector, as well as the a posteriori estimate
          *  error covariance matrix, using as input the predicted a priori
          *  state vector and error covariance matrix, plus measurements and
          *  associated matrices.
          *
          * @param measurements      Measurements vector.
          * @param measurementsMatrix    Measurements matrix. Called geometry
          *                              matrix in GNSS.
          * @param measurementsNoiseCovariance   Measurements noise covariance
          *                                      matrix.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Correct( const Vector<double>& measurements,
                           const Matrix<double>& measurementsMatrix,
                           const Matrix<double>& measurementsNoiseCovariance )
         throw(InvalidSolver);


         /** Corrects (or "measurement updates") the a posteriori estimate
          *  of the system state value, as well as the a posteriori estimate
          *  error variance, using as input the predicted a priori state and
          *  error variance values, plus measurement and associated gain and
          *  variance. Version for one-dimensional systems.
          *
          * @param measurement       Measurement value.
          * @param measurementsGain  Measurements gain.
          * @param measurementsNoiseVariance   Measurements noise variance.
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Correct( const double& measurement,
                           const double& measurementsGain,
                           const double& measurementsNoiseVariance )
         throw(InvalidSolver);


   }; // End of class 'SimpleKalmanFilter'

      //@}

}  // End of namespace gpstk
#endif // SIMPLEKALMANFILTER_HPP
