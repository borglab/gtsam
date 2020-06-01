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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007, 2008, 2009, 2011
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
 * @file DataStructures.hpp
 * Set of several data structures to be used by other GPSTk classes.
 */

#ifndef GPSTK_DATASTRUCTURES_HPP
#define GPSTK_DATASTRUCTURES_HPP

#include <utility>
#include <vector>
#include <set>
#include <map>
#include <string>

#include <gtsam/gpstk/DataHeaders.hpp>
#include <gtsam/gpstk/FFData.hpp>
#include <gtsam/gpstk/RinexObsStream.hpp>
#include <gtsam/gpstk/RinexObsData.hpp>
#include <gtsam/gpstk/Rinex3ObsStream.hpp>
#include <gtsam/gpstk/Rinex3ObsData.hpp>
#include <gtsam/gpstk/StringUtils.hpp>
#include <gtsam/gpstk/Vector.hpp>
#include <gtsam/gpstk/Matrix.hpp>
#include <gtsam/gpstk/CivilTime.hpp>
#include <gtsam/gpstk/YDSTime.hpp>
#include <gtsam/gpstk/GNSSconstants.hpp>



namespace gpstk
{

      /** @defgroup DataStructures GPSTk data structures 
       *
       * This is a set of several data structures to be used by other
       * GPSTk classes.
       *
       * Each data structure is composed of a header and a body. The header
       * contains the information that is common to all the data stored in
       * the structure, and the body contains the data themselves along with
       * the information (indexes) necessary to access them.
       *
       * In this regard, four basic indexes are considered enough to
       * completely identify any GNSS value:
       *
       *  \li Receiver/Source (SourceID)
       *  \li Epoch (CommonTime)
       *  \li Satellite (SatID)
       *  \li Type of value (TypeID)
       *
       * Moreover, all the GNSS data structures have two main parts:
       *
       *  \li Header: Containing the indexes that are common to all the values
       *              (sometimes with some extra information).
       *
       *  \li Body: Containing the GNSS values themselves, organized in
       *            std::maps.
       *
       * The general idea is to use the GNSS data structures like WHITE BOXES
       * that are able to carry all the important data around in an easy way,
       * in order to do something like the following to process GNSS data:
       *
       * @code
       *   RinexObsStream rin("bahr1620.04o"); // Create the input file stream
       *   gnssRinex gRin;                     // Declare a gnssRinex object
       *
       *   ModeledPR modelPR;          // Declare a ModeledReferencePR object
       *   SolverLMS solver;           // Declare an object to apply LMS method
       *
       *
       *   // ... other inicialization code here ...
       *
       *
       *   // Feed the gRin data structure
       *   while(rin >> gRin)
       *   {
       *
       *      gRin.keepOnlyTypeID(TypeID::C1) >> modelPR >> solver;
       *
       *      // Print the results for this epoch
       *      cout << gRin.header.epoch.DOYsecond() << "  ";   // Epoch
       *      cout << solver.solution[0] << "  ";              // dx
       *      cout << solver.solution[1] << "  ";              // dy
       *      cout << solver.solution[2] << "  ";              // dz
       *
       *   }
       * @endcode
       *
       */

      //@{


      // First, we must declare some important exception objects


      /// Thrown when attempting to access a value and the corresponding TypeID
      /// does not exist in the map.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(TypeIDNotFound, gpstk::Exception);


      /// Thrown when attempting to access a value and the corresponding SatID
      /// does not exist in the map.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SatIDNotFound, gpstk::Exception);


      /// Thrown when attempting to access a value and the corresponding
      /// source (SourceID) does not exist in the map.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(SourceIDNotFound, gpstk::Exception);


      /// Thrown when attempting to access a value and the corresponding
      /// epoch (CommonTime) does not exist in the map.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(CommonTimeNotFound, gpstk::Exception);


      /// Thrown when attempting to access a value and any of the corresponding
      /// indexes (SourceID, SatID or TypeID) does not exist in the map.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(ValueNotFound, gpstk::Exception);


      /// Thrown when the number of data values and the number of
      /// corresponding types does not match.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(NumberOfTypesMismatch, gpstk::Exception);


      /// Thrown when the number of data values and the number of
      /// corresponding satellites does not match.
      /// @ingroup exceptiongroup
   NEW_EXCEPTION_CLASS(NumberOfSatsMismatch, gpstk::Exception);


      // Now, some useful type definitions

      /// Set containing TypeID objects.
   typedef std::set<TypeID> TypeIDSet;

      /// Set containing SatID objects.
   typedef std::set<SatID> SatIDSet;

      /// Set containing SourceID objects.
   typedef std::set<SourceID> SourceIDSet;


      /// Map holding TypeID with corresponding numeric value.
   struct typeValueMap : std::map<TypeID, double>
   {

         /// Returns the number of different types available.
      inline size_t numTypes() const
      { return (*this).size(); }


         /// Returns a TypeIDSet with all the data types present in
         /// this object.
      TypeIDSet getTypeID() const;


         /// Returns a typeValueMap with only this type of data.
         /// @param type Type of value to be extracted.
      typeValueMap extractTypeID(const TypeID& type) const;


         /// Returns a typeValueMap with only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data to
         ///                be extracted.
      typeValueMap extractTypeID(const TypeIDSet& typeSet) const;


         /// Modifies this object, keeping only this type of data.
         /// @param type Type of value to be kept.
      typeValueMap& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      typeValueMap& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, removing this type of data.
         /// @param type Type of value to be removed.
      typeValueMap& removeTypeID(const TypeID& type)
      { (*this).erase(type); return (*this); }


         /// Modifies this object, removing these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      typeValueMap& removeTypeID(const TypeIDSet& typeSet);


         /** Returns the data value (double) corresponding to provided type.
          *
          * @param type       Type of value to be looked for.
          */
      double getValue(const TypeID& type) const
         throw(TypeIDNotFound);


         /// Returns a reference to the data value (double) with
         /// corresponding type.
         /// @param type Type of value to be looked for.
      double& operator()(const TypeID& type)
         throw(TypeIDNotFound);


         /// Destructor.
      virtual ~typeValueMap() {};

    };  // End typeValueMap



      /// Map holding SatID with corresponding numeric value.
   struct satValueMap : std::map<SatID, double>
   {

         /// Returns the number of satellites available.
      size_t numSats() const
      { return (*this).size(); }


         /// Returns a SatIDSet with all the satellites present in this object.
      SatIDSet getSatID() const;


         /// Returns a Vector with all the satellites present in this object.
      Vector<SatID> getVectorOfSatID() const;


         /// Returns a satValueMap with only this satellite.
         /// @param satellite Satellite to be extracted.
      satValueMap extractSatID(const SatID& satellite) const;


         /// Returns a satValueMap with only one satellite, identified by
         /// the given parameters.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      satValueMap extractSatID( const int& p,
                                const SatID::SatelliteSystem& s ) const;


         /// Returns a satValueMap with only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to
         ///               be extracted.
      satValueMap extractSatID(const SatIDSet& satSet) const;


         /// Modifies this object, keeping only this satellite.
         /// @param satellite Satellite to be kept.
      satValueMap& keepOnlySatID(const SatID& satellite);


         /// Modifies this object, keeping only this satellite.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      satValueMap& keepOnlySatID( const int& p,
                                  const SatID::SatelliteSystem& s );


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to be kept.
      satValueMap& keepOnlySatID(const SatIDSet& satSet);


         /// Modifies this object, removing this satellite.
         /// @param satellite Satellite to be removed.
      satValueMap& removeSatID(const SatID& satellite)
      { (*this).erase(satellite); return (*this); }


         /// Modifies this object, removing the given satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to
         ///               be removed.
      satValueMap& removeSatID(const SatIDSet& satSet);


         /** Returns the data value (double) corresponding to provided SatID.
          *
          * @param satellite     Satellite to be looked for.
          */
      double getValue(const SatID& satellite) const
         throw(SatIDNotFound);


         /// Returns a reference to the data value (double) with
         /// corresponding SatID.
         /// @param satellite Satellite to be looked for.
      double& operator()(const SatID& satellite)
         throw(SatIDNotFound);


         /// Destructor.
      virtual ~satValueMap() {};

   };  // End of 'satValueMap'



      /// Map holding SatID with corresponding typeValueMap.
   struct satTypeValueMap : std::map<SatID, typeValueMap>
   {

         /// Returns the number of available satellites.
      size_t numSats() const
      { return (*this).size(); }


         /** Returns the total number of data elements in the map.
          * This method DOES NOT suppose that all the satellites have
          * the same number of type values.
          */
      size_t numElements() const;


         /// Returns a SatIDSet with all the satellites present in this object.
      SatIDSet getSatID() const;


         /// Returns a Vector with all the satellites present in this object.
      Vector<SatID> getVectorOfSatID() const;


         /// Returns a TypeIDSet with all the data types present in
         /// this object.  This does not imply that all satellites have
         /// these types.
      TypeIDSet getTypeID() const;


         /// Returns a satTypeValueMap with only this satellite.
         /// @param satellite Satellite to be extracted.
      satTypeValueMap extractSatID(const SatID& satellite) const;


         /// Returns a satTypeValueMap with only one satellite, identified
         /// by the given parameters.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      satTypeValueMap extractSatID( const int& p,
                                    const SatID::SatelliteSystem& s) const;


         /// Returns a satTypeValueMap with only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to
         ///               be extracted.
      satTypeValueMap extractSatID(const SatIDSet& satSet) const;


         /// Modifies this object, keeping only this satellite.
         /// @param satellite Satellite to be kept.
      satTypeValueMap& keepOnlySatID(const SatID& satellite);


         /// Modifies this object, keeping only this satellite.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      satTypeValueMap& keepOnlySatID( const int& p,
                                      const SatID::SatelliteSystem& s );


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to be kept.
      satTypeValueMap& keepOnlySatID(const SatIDSet& satSet);


         /// Returns a satTypeValueMap with only this type of value.
         /// @param type Type of value to be extracted.
      satTypeValueMap extractTypeID(const TypeID& type) const;


         /// Returns a satTypeValueMap with only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be extracted.
      satTypeValueMap extractTypeID(const TypeIDSet& typeSet) const;


         /// Modifies this object, keeping only this type of data.
         /// @param type Type of value to be kept.
      satTypeValueMap& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      satTypeValueMap& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, removing this satellite.
         /// @param satellite Satellite to be removed.
      satTypeValueMap& removeSatID(const SatID& satellite)
      { (*this).erase(satellite); return (*this); }


         /// Modifies this object, removing these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be removed.
      satTypeValueMap& removeSatID(const SatIDSet& satSet);


         /// Modifies this object, removing this type of data.
         /// @param type Type of value to be removed.
      satTypeValueMap& removeTypeID(const TypeID& type);


         /// Modifies this object, removing these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      satTypeValueMap& removeTypeID(const TypeIDSet& typeSet);


         /// Returns a GPSTk::Vector containing the data values with this type.
         /// @param type Type of value to be returned.
         /// This method returns zero if a given satellite does not have
         /// this type.
      Vector<double> getVectorOfTypeID(const TypeID& type) const;


         /// Returns a GPSTk::Matrix containing the data values in this set.
         /// @param typeSet  TypeIDSet of values to be returned.
      Matrix<double> getMatrixOfTypes(const TypeIDSet& typeSet) const;


         /** Modifies this object, adding one vector of data with this type,
          *  one value per satellite.
          *
          * If type already exists, data is overwritten. If the number of
          * values does not match with the number of satellites, a
          * NumberOfSatsMismatch exception is thrown.
          *
          * Given that dataVector does not store information about the
          * satellites the values correspond to, the user is held responsible
          * for having the data values stored in dataVector in the proper
          * order regarding the SatIDs in this object.
          *
          * @param type          Type of data to be added.
          * @param dataVector    GPSTk Vector containing the data to be added.
          */
      satTypeValueMap& insertTypeIDVector( const TypeID& type,
                                           const Vector<double> dataVector )
         throw(NumberOfSatsMismatch);


         /** Modifies this object, adding a matrix of data, one vector
          *  per satellite.
          *
          * If types already exists, data is overwritten. If the number of
          * rows in matrix does not match with the number of satellites, a
          * NumberOfSatsMismatch exception is thrown. If the number of columns
          * in matrix does not match with the number of types in typeSet, a
          * NumberOfTypesMismatch exception is thrown.
          *
          * Given that dataMatrix does not store information about the
          * satellites and types the values correspond to, the user is held
          * responsible for having those data values stored in dataMatrix in
          * the proper order regarding the SatIDs in this object and the
          * provided typeSet.
          *
          * @param typeSet       Set (TypeIDSet) containing the types of data
          *                      to be added.
          * @param dataMatrix    GPSTk Matrix containing the data to be added.
          */
      satTypeValueMap& insertMatrix( const TypeIDSet& typeSet,
                                     const Matrix<double> dataMatrix )
         throw(NumberOfSatsMismatch, NumberOfTypesMismatch);


         /** Returns the data value (double) corresponding to provided SatID
          *  and TypeID.
          *
          * @param satellite     Satellite to be looked for.
          * @param type          Type to be looked for.
          */
      double getValue( const SatID& satellite,
                       const TypeID& type ) const
         throw( SatIDNotFound, TypeIDNotFound );


         /// Returns a reference to the typeValueMap with corresponding SatID.
         /// @param type Type of value to be look for.
      typeValueMap& operator()(const SatID& satellite)
         throw(SatIDNotFound);


         /// Convenience output method
      virtual std::ostream& dump( std::ostream& s,
                                  int mode = 0) const;


         /// Destructor.
      virtual ~satTypeValueMap() {};

   };  // End of 'satTypeValueMap'



      /// stream output for satTypeValueMap
   std::ostream& operator<<( std::ostream& s,
                             const satTypeValueMap& stvMap);



      /// Map holding epoch with corresponding satTypeValueMap.
   typedef std::map<CommonTime, satTypeValueMap>  epochSatTypeValueMap;

      /// Map holding epoch with corresponding satValueMap.
   typedef std::map<CommonTime, satValueMap>      epochSatValueMap;

      /// Map holding epoch with corresponding typeValueMap.
   typedef std::map<CommonTime, typeValueMap>     epochTypeValueMap;




      /// Basic gnssData structure.
   template <class HEADER_CLASS, class BODY_CLASS>
   struct gnssData
   {

         /// Header.
      HEADER_CLASS header;


         /// Body.
      BODY_CLASS   body;



         /// Default constructor.
      gnssData() {}


         /// Common constructor.
      gnssData( const HEADER_CLASS& h,
                const BODY_CLASS& b )
      {
         header = h;
         body = b;
      }


         /// Copy constructor.
      template<class H, class B>
      gnssData(const gnssData<H,B>& g)
      {
         header = g.header;
         body = g.body;
      }


         /// Destructor.
      virtual ~gnssData() {};

   };  // End of 'gnssData'




      // Further type definitions

      /// GNSS data structure with source, epoch and data type as header
      /// (common indexes) and satValueMap as body.
   struct  gnssSatValue : gnssData<sourceEpochTypeHeader, satValueMap>
   {


         /// Returns the number of satellites available in the body,
         /// which is a satValueMap.
      size_t numSats() const
      { return body.numSats(); };


         /// Returns a SatIDSet with all the satellites present in this object.
      SatIDSet getSatID() const
      { return (*this).body.getSatID(); }


         /// Returns a Vector with all the satellites present in this object.
      Vector<SatID> getVectorOfSatID() const
      { return body.getVectorOfSatID(); }


         /// Returns a gnssSatValue with only this satellite.
         /// @param satellite Satellite to be extracted.
      gnssSatValue extractSatID(const SatID& satellite) const;


         /// Returns a gnssSatValue with only one satellite, identified by
         /// the given parameters.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssSatValue extractSatID( const int& p,
                                 const SatID::SatelliteSystem& s ) const;


         /// Returns a gnssSatValue with only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be extracted.
      gnssSatValue extractSatID(const SatIDSet& satSet) const;


         /// Modifies this object, keeping only this satellite.
         /// @param satellite Satellite to be kept.
      gnssSatValue& keepOnlySatID(const SatID& satellite);


         /// Modifies this object, keeping only this satellite.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssSatValue& keepOnlySatID( const int& p,
                                   const SatID::SatelliteSystem& s );


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to be kept.
      gnssSatValue& keepOnlySatID(const SatIDSet& satSet);


         /// Modifies this object, removing this satellite.
         /// @param satellite Satellite to be removed.
      gnssSatValue& removeSatID(const SatID& satellite)
      { (*this).body.erase(satellite); return (*this); }


         /// Modifies this object, removing these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be removed.
      gnssSatValue& removeSatID(const SatIDSet& satSet);


         /** Returns the data value (double) corresponding to provided SatID.
          *
          * @param satellite     Satellite to be looked for.
          */
      double getValue(const SatID& satellite) const
         throw(SatIDNotFound)
      { return (*this).body.getValue(satellite); }


         /// Returns a reference to the value (double) with corresponding
         /// satellite.
         /// @param satellite Satellite to be looked for.
      double& operator()(const SatID& satellite)
         throw(SatIDNotFound)
      { return (*this).body(satellite); }


         /// Destructor.
      virtual ~gnssSatValue() {};


   };  // End of 'gnssSatValue'




      /// GNSS data structure with source, epoch and satellite as header
      /// (common indexes) and typeValueMap as body.
   struct  gnssTypeValue : gnssData<sourceEpochSatHeader, typeValueMap>
   {

         /// Returns the number of types available in the body,
         /// which is a typeValueMap.
      size_t numTypes() const
      { return body.numTypes(); };


         /// Returns a TypeIDSet with all the data types present
         /// in this object.
      TypeIDSet getTypeID() const
      { return (*this).body.getTypeID(); }


         /// Returns a gnssTypeValue with only this type of data.
         /// @param type Type of value to be extracted.
      gnssTypeValue extractTypeID(const TypeID& type) const;


         /// Returns a gnssTypeValue with only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be extracted.
      gnssTypeValue extractTypeID(const TypeIDSet& typeSet) const;


         /// Modifies this object, keeping only this type of data.
         /// @param type Type of value to be kept.
      gnssTypeValue& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      gnssTypeValue& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, removing this type of data.
         /// @param type Type of value to be removed.
      gnssTypeValue& removeTypeID(const TypeID& type)
      { (*this).body.erase(type); return (*this); }


         /// Modifies this object, removing these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      gnssTypeValue& removeTypeID(const TypeIDSet& typeSet);


         /** Returns the data value (double) corresponding to provided TypeID.
          *
          * @param type    Type to be looked for.
          */
      double getValue(const TypeID& type) const
         throw(TypeIDNotFound)
      { return (*this).body.getValue(type); }


         /// Returns a reference to the value (double) with corresponding type.
         /// @param type TypeID to be looked for.
      double& operator()(const TypeID& type)
         throw(TypeIDNotFound)
      { return (*this).body(type); }


         /// Destructor.
      virtual ~gnssTypeValue() {};


   };  // End of 'gnssTypeValue'



      /// GNSS data structure with source and epoch as header
      /// (common indexes) and satTypeValueMap as body.
   struct  gnssSatTypeValue : gnssData<sourceEpochHeader, satTypeValueMap>
   {

         /// Returns the number of satellites available in the body,
         /// which is a satTypeValueMap.
      size_t numSats() const
      { return body.numSats(); };


         /// Returns a TypeIDSet with all the data types present in
         /// this object.
      TypeIDSet getTypeID() const
      { return (*this).body.getTypeID(); }


         /// Returns a SatIDSet with all the satellites present in this object.
      SatIDSet getSatID() const
      { return (*this).body.getSatID(); }


         /// Returns a Vector with all the satellites present in this object.
      Vector<SatID> getVectorOfSatID() const
      { return (*this).body.getVectorOfSatID(); }


         /** Returns the total number of data elements in the body.
          * This method DOES NOT suppose that all the satellites have
          * the same number of type values.
          */
      size_t numElements() const
      { return body.numElements(); };


         /// Returns a gnssSatTypeValue with only this satellite.
         /// @param satellite Satellite to be extracted.
      gnssSatTypeValue extractSatID(const SatID& satellite) const;


         /// Returns a gnssSatTypeValue with only one satellite, identified
         /// by the given parameters.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssSatTypeValue extractSatID( const int& p,
                                     const SatID::SatelliteSystem& s ) const;


         /// Returns a gnssSatTypeValue with only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be extracted.
      gnssSatTypeValue extractSatID(const SatIDSet& satSet) const;


         /// Modifies this object, keeping only this satellite.
         /// @param satellite Satellite to be kept.
      gnssSatTypeValue& keepOnlySatID(const SatID& satellite);


         /// Modifies this object, keeping only this satellite.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssSatTypeValue& keepOnlySatID( const int& p,
                                       const SatID::SatelliteSystem& s );


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to be kept.
      gnssSatTypeValue& keepOnlySatID(const SatIDSet& satSet);


         /// Returns a gnssSatTypeValue with only this type of data.
         /// @param type Type of value to be extracted.
      gnssSatTypeValue extractTypeID(const TypeID& type) const;


         /// Returns a gnssSatTypeValue with only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be extracted.
      gnssSatTypeValue extractTypeID(const TypeIDSet& typeSet) const;


         /// Modifies this object, keeping only this type of data.
         /// @param type Type of value to be kept.
      gnssSatTypeValue& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      gnssSatTypeValue& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, removing this satellite.
         /// @param satellite Satellite to be removed.
      gnssSatTypeValue& removeSatID(const SatID& satellite)
      { (*this).body.erase(satellite); return (*this); }


         /// Modifies this object, removing these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be removed.
      gnssSatTypeValue& removeSatID(const SatIDSet& satSet);


         /// Modifies this object, removing this type of data.
         /// @param type Type of value to be kept.
      gnssSatTypeValue& removeTypeID(const TypeID& type)
      { (*this).body.removeTypeID(type); return (*this); }


         /// Modifies this object, removing these types of data
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      gnssSatTypeValue& removeTypeID(const TypeIDSet& typeSet);


         /// Returns a GPSTk::Vector containing the data values with this type.
         /// @param type Type of value to be returned.
      Vector<double> getVectorOfTypeID(const TypeID& type) const
      { return ( (*this).body.getVectorOfTypeID(type) ); }


         /** Modifies this object, adding one vector of data with this type,
          *  one value per satellite.
          *
          * If type already exists, data is overwritten. If the number of
          * values does not match with the number of satellites, a
          * NumberOfSatsMismatch exception is thrown.
          *
          * Given that dataVector does not store information about the
          * satellites the values correspond to, the user is held responsible
          * for having the data values stored in dataVector in the proper order
          * regarding the SatIDs in this object.
          *
          * @param type          Type of data to be added.
          * @param dataVector    GPSTk Vector containing the data to be added.
          */
      gnssSatTypeValue& insertTypeIDVector( const TypeID& type,
                                            const Vector<double> dataVector )
         throw(NumberOfSatsMismatch)
      { (*this).body.insertTypeIDVector(type, dataVector); return (*this); }


         /** Modifies this object, adding a matrix of data, one vector
          *  per satellite.
          *
          * If types already exists, data is overwritten. If the number of
          * rows in matrix does not match with the number of satellites, a
          * NumberOfSatsMismatch exception is thrown. If the number of columns
          * in matrix does not match with the number of types in typeSet, a
          * NumberOfTypesMismatch exception is thrown.
          *
          * Given that dataMatrix does not store information about the
          * satellites and types the values correspond to, the user is held
          * responsible for having those data values stored in dataMatrix in
          * the proper order regarding the SatIDs in this object and the
          * provided typeSet.
          *
          * @param typeSet    Set (TypeIDSet) containing the types of data
          *                   to be added.
          * @param dataMatrix GPSTk Matrix containing the data to be added.
          */
      gnssSatTypeValue& insertMatrix( const TypeIDSet& typeSet,
                                      const Matrix<double> dataMatrix )
         throw(NumberOfSatsMismatch, NumberOfTypesMismatch)
      { (*this).body.insertMatrix(typeSet, dataMatrix); return (*this); }


         /** Returns the data value (double) corresponding to provided SatID
          *  and TypeID.
          *
          * @param satellite     Satellite to be looked for.
          * @param type          Type to be looked for.
          */
      double getValue( const SatID& satellite,
                       const TypeID& type ) const
         throw( SatIDNotFound, TypeIDNotFound )
      { return (*this).body.getValue( satellite, type ); }


         /** Returns a reference to the typeValueMap with corresponding
          *  satellite.
          *
          * This operator allows direct access to data values when chained
          * with the typeValueMap::operator(), like this:
          *
          *    gRin(sat21)(TypeID::C1).
          *
          * Example:
          *
          * @code
          *   // Create the input file stream
          *   RinexObsStream rin("bahr1620.04o");
          *
          *   // Declare a gnssRinex object
          *   gnssRinex gRin;
          *
          *   // Create a satellite object
          *   SatID sat21(21,SatID::systemGPS);
          *
          *   // Feed the gRin data structure
          *   while(rin >> gRin)
          *   {
          *      try
          *      {
          *          if (gRin(sat21)(TypeID::C1) == 0.0)
          *          {
          *             gRin(sat21)(TypeID::C1) = 123.456;
          *          }
          *
          *          cout << "C1 value for satellite G21: "
          *               << gRin(sat21)(TypeID::C1) << endl;
          *      }
          *      catch (SatIDNotFound& e)
          *      {
          *          cout << endl << "Satellite G21 not found." << endl;
          *      };
          *   }
          * @endcode
          *
          * @param satellite Satellite to be looked for.
          *
          * @warning Please be aware that this operator doesn't mantain the
          * 'constness' of the data structure, allowing direct access to all
          * data (including editing). If this is not what you want, use method
          * 'getValue()' instead.
          */
      typeValueMap& operator()(const SatID& satellite)
         throw(SatIDNotFound)
      { return (*this).body(satellite); }


         /// Destructor.
      virtual ~gnssSatTypeValue() {};


   };  // End of 'gnssSatTypeValue'



      /// GNSS data structure with source, epoch and extra Rinex data as
      /// header (common indexes) and satTypeValueMap as body.
   struct gnssRinex : gnssSatTypeValue
   {


         /// Header.
      sourceEpochRinexHeader header;


         /// Default constructor.
      gnssRinex() {};


         /** Explicit constructor from parent class
          *
          * @param gds      gnssSatTypeValue to build this gnssRinex from.
          */
      gnssRinex(const gnssSatTypeValue& gds)
      { header = gds.header; body = gds.body; };


         /// Returns a gnssRinex with only this satellite.
         /// @param satellite Satellite to be extracted.
      gnssRinex extractSatID(const SatID& satellite) const;


         /// Returns a gnssRinex with only one satellite, identified by
         /// the given parameters.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssRinex extractSatID( const int& p,
                              const SatID::SatelliteSystem& s ) const;


         /// Returns a gnssRinex with only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites
         ///               to be extracted.
      gnssRinex extractSatID(const SatIDSet& satSet) const;


         /// Modifies this object, keeping only this satellite.
         /// @param satellite Satellite to be kept.
      gnssRinex& keepOnlySatID(const SatID& satellite);


         /// Modifies this object, keeping only this satellite.
         /// @param p Satellite PRN number.
         /// @param p System the satellite belongs to.
      gnssRinex& keepOnlySatID( const int& p,
                                const SatID::SatelliteSystem& s );


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set (SatIDSet) containing the satellites to be kept.
      gnssRinex& keepOnlySatID(const SatIDSet& satSet);


         /// Returns a gnssRinex with only this type of data.
         /// @param type Type of value to be extracted.
      gnssRinex extractTypeID(const TypeID& type) const;


         /// Returns a gnssRinex with only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be extracted.
      gnssRinex extractTypeID(const TypeIDSet& typeSet) const;


         /// Modifies this object, keeping only this type of data.
         /// @param type Type of value to be kept.
      gnssRinex& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types of data.
         /// @param typeSet Set (TypeIDSet) containing the types of data
         ///                to be kept.
      gnssRinex& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Returns a gnssRinex with only these types of data.
         /// @param satSys Satellite System value to be kept. 
      gnssRinex& keepOnlySatSystem(const SatID::SatelliteSystem satSys);



         /// Destructor.
      virtual ~gnssRinex() {};


   };  // End of 'gnssRinex'



      //// Some other handy data structures


      /// GNSS data structure consisting in a map with SourceID as keys, and
      /// satTypeValueMap as elements.
   struct sourceDataMap : std::map<SourceID, satTypeValueMap>
   {

         /// Default constructor
      sourceDataMap() {};


         /** Returns the data value (double) corresponding to provided SourceID,
          *  SatID and TypeID.
          *
          * @param source        Source to be looked for.
          * @param satellite     Satellite to be looked for.
          * @param type          Type to be looked for.
          */
      double getValue( const SourceID& source,
                       const SatID& satellite,
                       const TypeID& type ) const
         throw( SourceIDNotFound, SatIDNotFound, TypeIDNotFound );


         /** Get a set with all the SourceID's in this data structure.
          *
          * @warning If current 'sourceDataMap' is big, this could be a very
          * costly operation.
          */
      SourceIDSet getSourceIDSet( void ) const;


         /** Get a set with all the SatID's in this data structure.
          *
          * @warning If current 'sourceDataMap' is big, this could be a very
          * costly operation.
          */
      SatIDSet getSatIDSet( void ) const;


         /// Destructor.
      virtual ~sourceDataMap() {};


   };    // End of 'sourceDataMap'



      /// GNSS data structure consisting in a map with CommonTime as keys, and
      /// sourceDataMap as elements.
   struct  gnssDataMap : std::multimap<CommonTime, sourceDataMap>
   {

         /// Default constructor
      gnssDataMap()
         : tolerance(0.1) {};


         /** Common constructor.
          *
          * @param tol     Tolerance to be applied to epochs (in seconds).
          */
      gnssDataMap( double tol )
         : tolerance(tol) {};


         /** Adds 'gnssSatTypeValue' object data to this structure.
          *
          * @param gds     gnssSatTypeValue object containing data to be added.
          */
      gnssDataMap& addGnssSatTypeValue( const gnssSatTypeValue& gds );


         /** Adds 'gnssRinex' object data to this structure.
          *
          * @param gds     gnssRinex object containing data to be added.
          */
      gnssDataMap& addGnssRinex( const gnssRinex& gds );


         /** Returns a 'gnssRinex' object corresponding to given SourceID.
          *
          * @param source     SourceID object.
          *
          * @warning Returned data will correspond to first matching SourceID,
          * if it exists.
          */
      gnssRinex getGnssRinex( const SourceID& source ) const;


         /** Adds 'gnssDataMap' object data to this structure.
          *
          * @param gds     gnssDataMap object containing data to be added.
          */
      gnssDataMap& addGnssDataMap( const gnssDataMap& gds );


         /// Returns a copy of the first element in the map.
      gnssDataMap front( void ) const;


         /// Removes the first element in the map.
      void pop_front( void );


         /// Returns the data corresponding to the first epoch in the map,
         /// taking into account the 'tolerance' value.
      gnssDataMap frontEpoch( void ) const;


         /// Removes the first epoch in the map. Be aware that this method
         /// takes into account 'tolerance', so more than just one epoch may be
         /// removed if they are within 'tolerance' margin.
      void pop_front_epoch( void );


         /// Returns a copy of the last element in the map.
      gnssDataMap back( void ) const;


         /// Removes the last element in the map.
      void pop_back( void );


         /// Returns the data corresponding to the last epoch in the map,
         /// taking into account the 'tolerance' value.
      gnssDataMap backEpoch( void ) const;


         /// Removes the last epoch in the map. Be aware that this method
         /// takes into account 'tolerance', so more than just one epoch may be
         /// removed if they are within 'tolerance' margin.
      void pop_back_epoch( void );


         /** Returns a 'gnssDataMap' with the data corresponding to provided
          *  CommonTime, taking into account 'tolerance'.
          *
          * @param epoch         Epoch to be looked for.
          */
      gnssDataMap getDataFromEpoch( const CommonTime& epoch ) const
         throw( CommonTimeNotFound );


         /** Returns the data value (double) corresponding to provided CommonTime,
          *  SourceID, SatID and TypeID.
          *
          * @param epoch         Epoch to be looked for.
          * @param source        Source to be looked for.
          * @param satellite     Satellite to be looked for.
          * @param type          Type to be looked for.
          *
          * @warning If within (epoch +/- tolerance) more than one match exists,
          * then only the first one is returned.
          */
      double getValue( const CommonTime& epoch,
                       const SourceID& source,
                       const SatID& satellite,
                       const TypeID& type ) const
         throw( CommonTimeNotFound, ValueNotFound );


         /** Returns the data value (double) corresponding to the first epoch
          *  in the data structure, given SourceID, SatID and TypeID.
          *
          * @param source        Source to be looked for.
          * @param satellite     Satellite to be looked for.
          * @param type          Type to be looked for.
          *
          * @warning If within first epoch (epoch +/- tolerance) more than one
          * match exists, then only the first one is returned.
          */
      double getValue( const SourceID& source,
                       const SatID& satellite,
                       const TypeID& type ) const
         throw( ValueNotFound );


         /** Inserts a data value (double) at the provided CommonTime, SourceID,
          *  SatID and TypeID, taking into account 'tolerance'.
          *
          * @param epoch         Epoch to be looked for.
          * @param source        Source to be looked for.
          * @param satellite     Satellite to be looked for.
          * @param type          Type of the new data.
          * @param value         Value to be inserted.
          */
      gnssDataMap& insertValue( const CommonTime& epoch,
                                const SourceID& source,
                                const SatID& satellite,
                                const TypeID& type,
                                double value )
         throw( CommonTimeNotFound, ValueNotFound );


         /** Inserts a data value (double) in the first epoch of the data
          *  structure with the given SourceID, SatID and TypeID.
          *
          * @param source        Source to be looked for.
          * @param satellite     Satellite to be looked for.
          * @param type          Type of the new data.
          * @param value         Value to be inserted.
          */
      gnssDataMap& insertValue( const SourceID& source,
                                const SatID& satellite,
                                const TypeID& type,
                                double value )
         throw( ValueNotFound );


         /** Get a set with all the SourceID's in this data structure.
          *
          * @warning If current 'gnssDataMap' is big, this could be a very
          * costly operation.
          */
      SourceIDSet getSourceIDSet( void ) const;


         /** Get a set with all the SatID's in this data structure.
          *
          * @warning If current 'gnssDataMap' is big, this could be a very
          * costly operation.
          */
      SatIDSet getSatIDSet( void ) const;


         /// Get tolerance
      double getTolerance( void ) const
      { return tolerance; };


         /** Set tolerance.
          *
          * @param tol     Tolerance, in seconds.
          */
      gnssDataMap& setTolerance( double tol )
      { tolerance = std::abs(tol); return (*this); };


         /// Convenience output method
      virtual std::ostream& dump( std::ostream& s,
                                  int mode = 0) const;

         
         /// Returns a gnssDataMap with only this source.
         /// @param source Source to be extracted.
      gnssDataMap extractSourceID(const SourceID& source);
         

         /// Returns a gnssDataMap with only these sources.
         /// @param sourceSet Set(SourceIDSet) containing the sources 
         ///                  to be extracted.
      gnssDataMap extractSourceID(const SourceIDSet& sourceSet);


         /// Modifies this object, keeping only this source.
         /// @param source Source to be extracted.
      gnssDataMap& keepOnlySourceID(const SourceID& source);


         /// Modifies this object, keeping only these sources.
         /// @param sourceSet Set(SourceIDSet) containing the sources 
         ///                  to be extracted.
      gnssDataMap& keepOnlySourceID(const SourceIDSet& sourceSet);


         /// Modifies this object, removing this source.
         /// @param source Source to be removed.
      gnssDataMap& removeSourceID(const SourceID& source);


         /// Modifies this object, keeping only these sources.
         /// @param sourceSet Set(SourceIDSet) containing the sources 
         ///                  to be removed.
      gnssDataMap& removeSourceID(const SourceIDSet& sourceSet);


         /// Returns a gnssDataMap with only this satellite.
         /// @param sat Satellite to be extracted.
      gnssDataMap extractSatID(const SatID& sat);


         /// Returns a gnssDataMap with only these satellites.
         /// @param satSet Set(SatIDSet) containing the satellite 
         ///               to be extracted.
      gnssDataMap extractSatID(const SatIDSet& satSet);


         /// Modifies this object, keeping only this satellite.
         /// @param sat Satellite to be extracted.
      gnssDataMap& keepOnlySatID(const SatID& sat);


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set(SatIDSet) containing the satellite 
         ///                  to be extracted.
      gnssDataMap& keepOnlySatID(const SatIDSet& satSet);


         /// Modifies this object, removing this satellite.
         /// @param sat Satellite to be removed.
      gnssDataMap& removeSatID(const SatID& sat);


         /// Modifies this object, keeping only these satellites.
         /// @param satSet Set(SatIDSet) containing the satellites 
         ///               to be removed.
      gnssDataMap& removeSatID(const SatIDSet& satSet);


         /// Returns a gnssDataMap with only this type.
         /// @param type Type to be extracted.
      gnssDataMap extractTypeID(const TypeID& type);


         /// Returns a gnssDataMap with only these satellites.
         /// @param typeSet Set(TypeIDSet) containing the types 
         ///               to be extracted.
      gnssDataMap extractTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, keeping only this type.
         /// @param type Type to be extracted.
      gnssDataMap& keepOnlyTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types.
         /// @param typeSet Set(TypeIDSet) containing the type 
         ///                  to be extracted.
      gnssDataMap& keepOnlyTypeID(const TypeIDSet& typeSet);


         /// Modifies this object, removing this type.
         /// @param type Type to be removed.
      gnssDataMap& removeTypeID(const TypeID& type);


         /// Modifies this object, keeping only these types.
         /// @param typeSet Set(TypeIDSet) containing the types 
         ///               to be removed.
      gnssDataMap& removeTypeID(const TypeIDSet& typeSet);


         /** Edit the dataset, removing data outside the indicated time
          *  interval.
          *
          * @param[in] tmin defines the beginning of the time interval
          * @param[in] tmax defines the end of the time interval
          */
      virtual gnssDataMap& edit(CommonTime tmin, 
                                CommonTime tmax = CommonTime::END_OF_TIME);

         /// Load data from a rinex observation file
      void loadObsFile(std::string obsFile);


         /// Tolerance set to get data from a given epoch
      double tolerance;


         /// Destructor.
      virtual ~gnssDataMap() {};


   };    // End of 'gnssDataMap'



      /// stream output for gnssDataMap
   std::ostream& operator<<( std::ostream& s,
                             const gnssDataMap& gdsMap);



      /// Object defining the structure of a GNSS equation. The header is the
      /// prefit and the body is a TypeIDSet containing the unknowns.
   struct  gnssEquationDefinition : gnssData<TypeID, TypeIDSet>
   {

         /// Default constructor.
      gnssEquationDefinition() {};


         /// Common constructor.
      gnssEquationDefinition( const TypeID& h,
                              const TypeIDSet& b )
      {
         header = h;
         body   = b;
      }


         /// Destructor.
      virtual ~gnssEquationDefinition() {};


   };  // End of 'gnssEquationDefinition'



      /// Object defining the structure of a GNSS linear combination. The
      /// header is the result type and the body is a typeValueMap containing
      /// the GNSS data types to be combined plus corresponding coefficients.
   struct  gnssLinearCombination : gnssData<TypeID, typeValueMap>
   {

         /// Default constructor.
      gnssLinearCombination() {};


         /// Common constructor.
      gnssLinearCombination( const TypeID& h,
                             const typeValueMap& b )
      {
         header = h;
         body   = b;
      }


         /// Destructor.
      virtual ~gnssLinearCombination() {};


   };  // End of 'gnssLinearCombination'


      /// List containing gnssLinearCombination objects.
   typedef std::list<gnssLinearCombination> LinearCombList;


      /// Convenience function to convert from SatID system to SourceID type.
      /// @param sid Satellite ID.
   SourceID::SourceType SatIDsystem2SourceIDtype(const SatID& sid);



      /// Convenience function to fill a satTypeValueMap with data
      /// from RinexObsData.
      /// @param roh RinexObsHeader holding the data
      /// @param rod RinexObsData holding the data.
   satTypeValueMap satTypeValueMapFromRinexObsData(
                           const RinexObsHeader& roh, const RinexObsData& rod );


      /// Convenience function to fill a satTypeValueMap with data
      /// from Rinex3ObsData.
      /// @param roh Rinex3ObsHeader holding the data
      /// @param rod Rinex3ObsData holding the data.
   satTypeValueMap satTypeValueMapFromRinex3ObsData(
                         const Rinex3ObsHeader& roh, const Rinex3ObsData& rod );


      /** Stream input for gnssRinex.
       *
       * This handy operator allows to fed a gnssRinex data structure
       * directly from an input stream such a RinexObsStream object.
       *
       * For example:
       *
       * @code
       *   // Create the input file stream
       *   RinexObsStream rin("bahr1620.04o");
       *
       *   // Declare a gnssRinex object
       *   gnssRinex gRin;
       *
       *   // Feed the gRin data structure
       *   while( rin >> gRin )
       *   {
       *       // Lots of stuff in here...
       *   }
       * @endcode
       */
   std::istream& operator>>( std::istream& i, gnssRinex& f );


	   /** Stream output for gnssRinex.
       *
       * This handy operator allows to output a gnssRinex data structure
       * directly to an output stream such a RinexObsStream object.
       * 
       * The RinexObsHeader object of output stream should be initialized
       * correctly before any output operation.
       *
       * For example:
       *
       * @code
       *   // Create the input file stream
       *   RinexObsStream rin("bahr1620.04o");
       *
       *   //Create the output file stream
       *   RinexObsStream rout("bahr1620.04o.new", ios::out|ios::trunc);
       *
       *   // Read the RINEX header
       *   RinexObsHeader head; //RINEX header object
       *   rin >> head;
       *   rout.header = rin.header;
       *
       *   rout << rout.header;
       *
       *   gnssRinex gRin;
       *   while( rin >> gRin )
       *   {
       *       rout << gRin
       *   }
       * @endcode
       */
   std::ostream& operator<<( std::ostream& s,
                             gnssRinex& f )
      throw(FFStreamError, gpstk::StringUtils::StringException);


      //@}

}  // End of namespace gpstk

#endif // GPSTK_DATASTRUCTURES_HPP
