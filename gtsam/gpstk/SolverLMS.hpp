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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2006, 2007, 2008, 2011
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
 * @file SolverLMS.hpp
 * Class to compute the Least Mean Squares Solution
 */

#ifndef GPSTK_SOLVERLMS_HPP
#define GPSTK_SOLVERLMS_HPP

#include <gtsam/gpstk/SolverBase.hpp>
#include <gtsam/gpstk/TypeID.hpp>
#include <gtsam/gpstk/ProcessingClass.hpp>


namespace gpstk
{

      /** @addtogroup GPSsolutions */
      /// @ingroup math
      //@{

      /** This class computes the Least Mean Squares Solution of a given
       *  equations set.
       *
       * This class may be used either in a Vector- and Matrix-oriented way,
       * or with GNSS data structure objects from "DataStructures" class.
       *
       * A typical way to use this class with GNSS data structures follows:
       *
       * @code
       *      // Data stream
       *   RinexObsStream rin("ebre0300.02o");
       *
       *      // More declarations here: Ionospheric and tropospheric models,
       *      // ephemeris, etc.
       *
       *     // Declare a modeler object, setting all the parameters in
       *     // one pass
       *   ModelObs model( ionoStore,
       *                   mopsTM,
       *                   bceStore,
       *                   TypeID::C1 );
       *
       *      // Set initial position (Bancroft method)
       *   model.Prepare();
       *
       *      // Declare a SolverLMS object
       *   SolverLMS solver;
       *
       *      // GDS object
       *   gnssRinex gRin;
       *
       *   while(rin >> gRin)
       *   {
       *      gRin >> model >> solver;
       *   }
       * @endcode
       *
       * The "SolverLMS" object will extract all the data it needs from the
       * GNSS data structure that is "gRin" and will try to solve the system
       * of equations using the Least-Mean-Squares method. It will also insert
       * back postfit residual data into "gRin" if it successfully solves the
       * equation system.
       *
       * By default, it will build the geometry matrix from the values of
       * coefficients dx, dy, dz and cdt, and the independent vector will be
       * composed of the code prefit residuals (TypeID::prefitC) values.
       *
       * You may change the former by redefining the default equation
       * definition to be used. For instance:
       *
       * @code
       *   TypeIDSet unknownsSet;
       *   unknownsSet.insert(TypeID::dLat);
       *   unknownsSet.insert(TypeID::dLon);
       *   unknownsSet.insert(TypeID::dH);
       *   unknownsSet.insert(TypeID::cdt);
       *
       *      // Create a new equation definition
       *      // newEq(independent value, set of unknowns)
       *   gnssEquationDefinition newEq(TypeID::prefitC, unknownsSet);
       *
       *      // Reconfigure solver
       *   solver.setDefaultEqDefinition(newEq);
       * @endcode
       *
       * @sa SolverBase.hpp for base class.
       *
       */
   class SolverLMS : public SolverBase, public ProcessingClass
   {
   public:


         /** Default constructor. When fed with GNSS data structures, the
          *  default the equation definition to be used is the common GNSS
          *  code equation.
          */
      SolverLMS();


         /** Explicit constructor. Sets the default equation definition
          *  to be used when fed with GNSS data structures.
          *
          * @param eqDef     gnssEquationDefinition to be used
          */
      SolverLMS(const gnssEquationDefinition& eqDef)
         : defaultEqDef(eqDef)
      { };


         /** Compute the Least Mean Squares Solution of the given
          *  equations set.
          *
          * @param prefitResiduals   Vector of prefit residuals
          * @param designMatrix      Design matrix for the equation system
          *
          * @return
          *  0 if OK
          *  -1 if problems arose
          */
      virtual int Compute( const Vector<double>& prefitResiduals,
                           const Matrix<double>& designMatrix )
         throw(InvalidSolver);


         /** Returns a reference to a satTypeValueMap object after
          *  solving the previously defined equation system.
          *
          * @param gData     Data object holding the data.
          */
      virtual satTypeValueMap& Process(satTypeValueMap& gData)
         throw(ProcessingException);


         /** Returns a reference to a gnnsSatTypeValue object after
          *  solving the previously defined equation system.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssSatTypeValue& Process(gnssSatTypeValue& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /** Returns a reference to a gnnsRinex object after solving
          *  the previously defined equation system.
          *
          * @param gData    Data object holding the data.
          */
      virtual gnssRinex& Process(gnssRinex& gData)
         throw(ProcessingException)
      { Process(gData.body); return gData; };


         /** Returns the solution associated to a given TypeID.
          *
          * @param type    TypeID of the solution we are looking for.
          */
      virtual double getSolution(const TypeID& type) const
         throw(InvalidRequest);


         /** Returns the variance associated to a given TypeID.
          *
          * @param type    TypeID of the variance we are looking for.
          */
      virtual double getVariance(const TypeID& type) const
         throw(InvalidRequest);


         /** Method to set the default equation definition to be used
          *  when fed with GNSS data structures.
          *
          * @param eqDef     gnssEquationDefinition to be used by default.
          */
      virtual SolverLMS& setDefaultEqDefinition(
                                       const gnssEquationDefinition& eqDef )
      { defaultEqDef = eqDef; return (*this); };


         /** Method to get the default equation definition being used
          *  with GNSS data structures.
          */
      virtual gnssEquationDefinition getDefaultEqDefinition() const
      { return defaultEqDef; };


         /// Returns a string identifying this object.
      virtual std::string getClassName(void) const;


         /// Destructor.
      virtual ~SolverLMS() {};


   protected:


         /** Default equation definition to be used when fed with
          *  GNSS data structures.
          */
      gnssEquationDefinition defaultEqDef;


   }; // End of class 'SolverLMS'

      //@}

}  // End of namespace gpstk

#endif   // GPSTK_SOLVERLMS_HPP
