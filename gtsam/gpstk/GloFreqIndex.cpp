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

//============================================================================
//
// This software developed by Applied Research Laboratories at the University
// of Texas at Austin, under contract to an agency or agencies within the U.S. 
// Department of Defense. The U.S. Government retains all rights to use,
// duplicate, distribute, disclose, or release this software.
//
// Pursuant to DoD Directive 523024
//
// DISTRIBUTION STATEMENT A: This software has been approved for public 
//                           release, distribution is unlimited.
//
//============================================================================

/**
 * @file GloFreqIndex.cpp
 * Calculate GLONASS SV frequency index from range & phase data and store it.
 * See .hpp for full details.
 */

#include <cmath>

#include "CivilTime.hpp"
#include "GloFreqIndex.hpp"
#include "GNSSconstants.hpp"
//#include "icd_glo_freqindex.hpp"
#include "Rinex3NavData.hpp"
#include "Rinex3NavHeader.hpp"
#include "Rinex3NavStream.hpp"
#include "Stats.hpp"

using namespace gpstk::StringUtils;
using namespace std;

namespace gpstk
{
   const double GloFreqIndex::maxDist = 8000.0;
   const int GloFreqIndex::numSats = 24;
   const double GloFreqIndex::maxRPshift = 1.0;

   /// Fills map with known SV info as of January 2010.
   /// This method is for testing purposes only.

   void GloFreqIndex::knownIndex()
      throw()
   {
      // dummy map for testing purposes
      freqIndexTruth[RinexSatID("R01")] =  1;
      freqIndexTruth[RinexSatID("R02")] = -4;
      freqIndexTruth[RinexSatID("R03")] =  5;
      freqIndexTruth[RinexSatID("R04")] =  6;
      freqIndexTruth[RinexSatID("R05")] =  1;
      freqIndexTruth[RinexSatID("R06")] = -4;
      freqIndexTruth[RinexSatID("R07")] =  5;
      freqIndexTruth[RinexSatID("R08")] =  6;
      freqIndexTruth[RinexSatID("R09")] = -2;
      freqIndexTruth[RinexSatID("R10")] = -7;
      freqIndexTruth[RinexSatID("R11")] =  0;
//      freqIndexTruth[RinexSatID("R12")] = ; // not currently in orbit
      freqIndexTruth[RinexSatID("R13")] = -2;
      freqIndexTruth[RinexSatID("R14")] = -7;
      freqIndexTruth[RinexSatID("R15")] =  0;
//      freqIndexTruth[RinexSatID("R16")] = ; // not currently in orbit
      freqIndexTruth[RinexSatID("R17")] =  4;
      freqIndexTruth[RinexSatID("R18")] = -3;
      freqIndexTruth[RinexSatID("R19")] =  3;
      freqIndexTruth[RinexSatID("R20")] =  2;
      freqIndexTruth[RinexSatID("R21")] =  4;
      freqIndexTruth[RinexSatID("R22")] = -3;
      freqIndexTruth[RinexSatID("R23")] =  3;
      freqIndexTruth[RinexSatID("R24")] =  2;
   }


   /// Fills map with frequency index data from a RINEX 3 Nav data file.

   int GloFreqIndex::getFromRinex3Nav( const std::string& filename )
      throw()
   {
      Rinex3NavHeader header;
      Rinex3NavData data;

      Rinex3NavStream navfile(filename.c_str(), ios::in);
      navfile.exceptions(ifstream::failbit);

      navfile >> header;      // Read the header to get past it.

      // Read all entries and add them to the map.
      // If an existing entry disagrees with the new data,
      // return an error to the user.  Error codes:
      //   0 = no error
      //   1 = entry disagrees with new data
      while (navfile >> data)
      { 
         RinexSatID id(data.sat);
         int freqNum = static_cast<int>(data.freqNum);
         std::map< RinexSatID, int >::const_iterator iter;
         iter = freqIndex.find(id);
         if (iter != freqIndex.end())
         {
            if (iter->second != freqNum) return 1; // entry disagrees w/ new data
         }
         freqIndex[id] = freqNum;
      }

      return 0; // successful, no errors
   }


   /// Fills map with weighted-average results from accumulated data.
   /// Execute this method after all SatPass data has been added.

   void GloFreqIndex::calcIndex( const int& verbose )
      throw()
   {
      freqIndex.clear(); // Reset the map by clearing it.

      for (int i = 1; i <= numSats; i++) // Loop over GLONASS SVs by ID.
      {
         std::map<RinexSatID, passData>::const_iterator iter;
         RinexSatID id(i,SatID::systemGlonass);
         iter = dataMap.find(id);
         if ( iter != dataMap.end() )
         {
            if (verbose != 0)
               cout << endl << "Found SV " << id.toString() << endl;
            int length = iter->second.size();
            if (verbose != 0)
               cout << "There are " << length << " passes recorded." << endl;
            double navg      = 0.0;
            double totweight = 0.0;
            for (size_t j = 0; j < iter->second.size(); j++)
            {
               if (verbose != 0)
               {
                  cout << "   navg add'n: "
                       << (iter->second[j].nG1) << "  "
                       << (iter->second[j].dG1) << endl;
               }
               if (iter->second[j].dG1 == 0.0) continue; // don't divide by zero
               navg += (iter->second[j].nG1)/(iter->second[j].dG1); // weighted add'n
               totweight += 1.0/iter->second[j].dG1;
            }
            navg /= totweight; // average by sum of weights
            if (verbose != 0)
            {
               cout << "   navg,totwt = " << navg
                    << "   " << totweight << endl;
            }
            int index = static_cast<int>(navg + (navg<0 ? -0.5 : 0.5));
            freqIndex[id] = index;
         }
         else continue;
      }
   }


   // Calculates a GLONASS SV's frequency channel index from Obs data.
   // CommonTime tt should be the start time of the pass.
   /*
     STEPS:
     1. Scrub out large phase changes, e.g. > 8 km (in distance).
     2. Compute y(i) = R(i) - lambda0*phi(i) of the remaining data.
     3. Compute the first differences del-y and del-phi.\
     4. Scrub out large shifts in del-y, e.g. log[dy - median(dy)] > 1.
     5. Find slope of del-y v. lambda0*delphi (degree one, a "straight" line).
     6. Compute a double precision index and round to integer.
     7. Store results in the struct, including standard error from the fit.
     8. Propagate slope error to del-n.
     9. Compute overall final result & fill int map.
   */

   int GloFreqIndex::addPass( const RinexSatID& id, const CommonTime& tt,
                              const std::vector<double>& r1, const std::vector<double>& p1,
                              const std::vector<double>& r2, const std::vector<double>& p2,
                              const int& verbose                                           )
      throw()
   {
      // Vectors to hold data as it gets filtered.
      vector<double> dy1, dy2, dp1, dp2;

      // TwoSampleStats (vector pairs) of del-y v. lambda0*del-phi.
      TwoSampleStats<double> line1, line2;

      if (r1.size() != p1.size()) return 1; // Error: G1 range & phase data sizes not equal.
      if (r2.size() != p2.size()) return 2; // Error: G2 range & phase data sizes not equal.

      // Scrub out large phase shifts, > maxDist; don't keep those points.

      // Keep good del-phi and del-y = del-R - lambda0*del-phi for G1.
      for (size_t i = 1; i < r1.size(); i++)
      {
         double dp = L1_WAVELENGTH_GLO*(p1[i] - p1[i-1]);
         if (fabs(dp) < maxDist) // keep the point
         {
            dp1.push_back(dp);
            dy1.push_back(r1[i]-r1[i-1] - dp); // dy in writeup
         }
      }

      // Keep good del-phi and del-y = del-R - lambda0*del-phi for G2.
      for (size_t i = 1; i < r2.size(); i++)
      {
         double dp = L2_WAVELENGTH_GLO*(p2[i] - p2[i-1]);
         if (fabs(dp) < maxDist) // keep the point
         {
            dp2.push_back(dp);
            dy2.push_back(r2[i]-r2[i-1] - dp); // dy in writeup
         }
      }

      // Scrub out large Range-Phase shifts:
      // If log10[del-median(del)] > maxRPshift, reject it.

      double med1 = median<double>(dy1);

      for (size_t i = 0; i < dy1.size(); i++)
      {
         double spread = ::log10(fabs(dy1[i]-med1));
         if (spread < maxRPshift)
         {
            line1.Add(dp1[i],dy1[i]);
         }
      }

      double med2 = median<double>(dy2);

      for (size_t i = 0; i < dy2.size(); i++)
      {
         double spread = ::log10(fabs(dy2[i]-med2));
         if (spread < maxRPshift)
         {
            line2.Add(dp2[i],dy2[i]);
         }
      }

      // Compute best-fit slopes of lines and their uncertainties.

      if (line1.N() < 2 || line2.N() < 2) // no data, or only 1 point
      {
         if (verbose != 0)
         {
            cout << "No data (# points < 2) for " << id.toString() << endl;
         }
         return -3;
      }

      double  m1 = line1.Slope();
      double  m2 = line2.Slope();
      double dm1 = line1.SigmaSlope();
      double dm2 = line2.SigmaSlope();
      double  a1 = line1.Intercept();
      double  a2 = line2.Intercept();
      double sx1 = line1.StdDevX();
      double sx2 = line2.StdDevX();
      double sy1 = line1.StdDevY();
      double sy2 = line2.StdDevY();
#pragma unused(a1,a2,sx1,sx2,sy1,sy2)
      // Compute float values of index from slopes.

      if (1.0-m1 == 0.0) // don't divide by zero (G1)
         return -1;
      if (1.0-m2 == 0.0) // don't divide by zero (G2)
         return -2;

      double n1 = -(L1_FREQ_GLO/L1_FREQ_STEP_GLO)*m1/(1.0-m1);
      double n2 = -(L2_FREQ_GLO/L2_FREQ_STEP_GLO)*m2/(1.0-m2);

      // Compute uncertainties on the float index values.

      if (1.0+m1 == 0.0) // don't divide by zero (G1)
         return -1;
      if (1.0+m2 == 0.0) // don't divide by zero (G2)
         return -2;

      double dn1 = (L1_FREQ_GLO/L1_FREQ_STEP_GLO)*dm1/std::pow(m1+1.0,2);
      double dn2 = (L2_FREQ_GLO/L2_FREQ_STEP_GLO)*dm2/std::pow(m2+1.0,2);

      // Cast float index results to nearest integer.

      int index1 = static_cast<int>(n1 + (n1<0 ? -0.5 : 0.5));
      int index2 = static_cast<int>(n2 + (n2<0 ? -0.5 : 0.5));

      // Added data to struct, append to vector in map by SatID.

      if (verbose != 0)
      {
         cout << "Results for " << id.toString() << ":   n1,dn1, n2,dn2 =    "
              << n1 << " +/-" << dn1 << "    "
              << n2 << " +/-" << dn2 << endl;
      }

      if ( index1 != index2 ) // Error: G1 & G2 results disagree.
      {
         if (verbose != 0)
            cout << "G1 & G2 results disagree, result thrown away." << endl;
         return 3;
      }
      if ( dn1 > 0.5 )        // Error: nG1 uncertainty too large.
      {
         if (verbose != 0)
            cout << "G1 uncertainty too large, result thrown away." << endl;
         return 4;
      }
      if ( dn2 > 0.5 )        // Error: nG2 uncertainty too large.
      {
         if (verbose != 0)
            cout << "G2 uncertainty too large, result thrown away." << endl;
         return 5;
      }

      IndexData tempData;
      tempData.tt  = tt;
      tempData.pG1 = dy1.size();
      tempData.pG2 = dy2.size();
      tempData.fG1 = n1;
      tempData.fG2 = n2;
      tempData.dG1 = dn1;
      tempData.dG2 = dn2;
      tempData.nG1 = index1;
      tempData.nG2 = index2;

      if (verbose != 0)
      {
         cout << "Results (G1,G2) for " << id.toString()
              << ":  " << index1 << "   " << index2 << endl;
      }

      dataMap[id].push_back(tempData);

      return 0;
   }


   // Method to calculate frequency index for a single satellite from
   // range & phase data in a single pass for the G1 band only.  This
   // method provides empty G2 vectors to the above method.  Note that
   // if one provides range & phase data for only one band, it is
   // assumed to be G1!
   // CommonTime tt should be the start time of the pass.
   // Method currently implemented, but all solutions will be rejected.

   int GloFreqIndex::addPass( const RinexSatID& id, const CommonTime& tt,
                              const std::vector<double>& r1, const std::vector<double>& p1,
                              const int& verbose                                           )
      throw()
   {
      std::vector<double> r2, p2;
      r2.clear();
      p2.clear();
      return addPass( id, tt, r1, p1, r2, p2, verbose );
   }


   // Method to return the frequency index truth value for a given SV ID.

   int GloFreqIndex::getIndexTruth( const RinexSatID& id )
      throw()
   {
      std::map< RinexSatID, int >::const_iterator iter;
      iter = freqIndexTruth.find(id);

      if (iter != freqIndex.end())
         return iter->second;
      else
         return -100; // No entry for the given SatID.
   }


   // Method to return the frequency index value from data for a given SV ID.

   int GloFreqIndex::getIndex( const RinexSatID& id )
      throw()
   {
      std::map< RinexSatID, int >::const_iterator iter;
      iter = freqIndex.find(id);

      if (iter != freqIndex.end())
         return iter->second;
      else
         return -100; // No entry for the given SatID.
   }


   // Method to return the frequency from icd_glo_constants for a given SV ID,
   // using the truth index table.

   double GloFreqIndex::getFreqTruth( const RinexSatID& id, const int& band, int& error )
      throw()
   {
      int index = getIndexTruth(id);

      if (index < -10)
      {
         error = 1; // No entry for the given SatID.
         return 0.;
      }
      else
      {
         error = 0;

         if(band == 1 || band == 2)
            return getWavelength(id,band,index);
         //// Singleton reference.
         //GloFreq *inst;
         //inst = inst->instance();

         //if (band == 1)
         //   return inst->getL1(index);
         //else if (band == 2)
         //   return inst->getL2(index);
         else
         {
            error = 2; // Invalid frequency band.
            return 0.;
         }
      }
   }


   // Method to return the frequency from icd_glo_constants for a given SV ID,
   // using index values derived from Obs data.

   double GloFreqIndex::getFreq( const RinexSatID& id, const int& band, int& error )
      throw()
   {
      int index = getIndex(id);

      if (index < -10)
      {
         error = 1; // No entry for the given SatID.
         return 0.;
      }
      else
      {
         error = 0;

         // Singleton reference.
         //GloFreq *inst;
         //inst = inst->instance();

         //if (band == 1)
         //   return inst->getL1(index);
         //else if (band == 2)
         //   return inst->getL2(index);

         if(band == 1)
            return (L1_FREQ_GLO + index * L1_FREQ_STEP_GLO);
         else if(band == 2)
            return (L2_FREQ_GLO + index * L2_FREQ_STEP_GLO);
         else
         {
            error = 2; // Invalid frequency band.
            return 0.;
         }
      }
   }


   // Dumps an individual dataMap entry (passed) to cout in a nice format.

   void GloFreqIndex::dumpPass( const RinexSatID& id, const IndexData& data ) const
   {
      cout << endl << "Dump of dataMap entry:" << endl << endl;

      cout << "SV ID: " << id << endl;
      cout << "Start time of pass: " << CivilTime(data.tt) << endl;

      cout << "  # points:    " << setw(4) << setfill(' ') << data.pG1
           << "     "           << setw(4) << setfill(' ') << data.pG2 << endl;
      cout << "  flt soln:    " << setw(4) << setprecision(4) << data.fG1 << "   " << data.fG2 << endl;
      cout << "  uncert  :    " << setw(4) << setprecision(4) << data.dG1 << "   " << data.dG2 << endl;
      cout << "  int soln:    " << setw(4) << setfill(' ') << data.nG1
           << "     "           << setw(4) << setfill(' ') << data.nG2 << endl;

   } // end of dump()


   // Dumps all the stored data in a nice format.

   void GloFreqIndex::dump(ostream& s) const
   {
      s << "Dump of all dataMap entries:" << endl;
      s << "ID   Start Time               # points    float solns         uncertainties     int solns" << endl;
      s << setiosflags(ios::fixed);

      std::map< RinexSatID, passData >::const_iterator iter = dataMap.begin();

      for (; iter != dataMap.end(); iter++)
      {
         int length = (iter->second).size();
         for (int i = 0; i < length; i++)
         {
            IndexData data = (iter->second)[i];
            RinexSatID id = iter->first;

            s << id << "  "
              << CivilTime(data.tt).printf("%02m/%02d/%04Y %02H:%02M:%02S %P")
              << "  ";

            s << setw(4) << setfill(' ') << data.pG1 << " "
              << setw(4) << setfill(' ') << data.pG2 << "  ";
            s << setw(8) << setprecision(4) << data.fG1 << " "
              << setw(8) << setprecision(4) << data.fG2 << "  ";
            s << setw(8) << setprecision(4) << data.dG1 << " "
              << setw(8) << setprecision(4) << data.dG2 << "  ";
            s << setw(4) << setfill(' ') << data.nG1 << " "
              << setw(4) << setfill(' ') << data.nG2 << " ";
            s << setw(4) << setfill(' ') << freqIndexTruth.find(id)->second << " " << endl;
         }
      }

   } // end of dump()


} // namespace
