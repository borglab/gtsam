/// @file ClockSatStore.cpp
/// Store a tabular list of clock data (bias, drift, accel) for several satellites,
/// and compute values at any timetag from this table. Inherits TabularSatStore.

#include "ClockSatStore.hpp"
#include "MiscMath.hpp"

using namespace std;

namespace gpstk
{
   // Output stream operator is used by dump() in TabularSatStore
   ostream& operator<<(ostream& os, const ClockRecord& rec) throw()
   {
      os << scientific << setprecision(12) << setw(19) << rec.bias
         << " " << setw(19) << rec.sig_bias
         << " " << setw(19) << rec.drift
         << " " << setw(19) << rec.sig_drift
         //<< " " << setw(19) << rec.accel
         //<< " " << setw(19) << rec.sig_accel
         ;
      return os;
   }

   // Return value for the given satellite at the given time (usually via
   // interpolation of the data table). This interface from TabularSatStore.
   // @param[in] sat the SatID of the satellite of interest
   // @param[in] ttag the time (CommonTime) of interest
   // @return object of type ClockRecord containing the data value(s).
   // @throw InvalidRequest if data value cannot be computed, for example because
   //  a) the time t does not lie within the time limits of the data table
   //  b) checkDataGap is true and there is a data gap
   //  c) checkInterval is true and the interval is larger than maxInterval
   ClockRecord ClockSatStore::getValue(const SatID& sat, const CommonTime& ttag)
      const throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         bool isExact;
         ClockRecord rec;
         DataTableIterator it1, it2, kt;        // cf. TabularSatStore.hpp

         isExact = getTableInterval(sat, ttag, Nhalf, it1, it2, haveClockDrift);
         if(isExact && haveClockDrift) {
            rec = it1->second;
            return rec;
         }

         // pull data out of the data table
         int n,Nlow(Nhalf-1),Nhi(Nhalf),Nmatch(Nhalf);
         CommonTime ttag0(it1->first);
         vector<double> times,biases,drifts,accels,sig_biases,sig_drifts,sig_accels;

         kt=it1; n=0;
         while(1) {
            // find index of matching time tag
            if(isExact && ABS(kt->first-ttag) < 1.e-8) Nmatch = n;
            times.push_back(kt->first - ttag0);    // sec
            biases.push_back(kt->second.bias);     // sec
            drifts.push_back(kt->second.drift);    // sec/sec
            accels.push_back(kt->second.accel);    // sec/sec^2
            sig_biases.push_back(kt->second.sig_bias);     // sec
            sig_drifts.push_back(kt->second.sig_drift);    // sec/sec
            sig_accels.push_back(kt->second.sig_accel);    // sec/sec^2
            if(kt == it2) break;
            ++kt;
            ++n;
         };

         if(isExact && Nmatch==Nhalf-1) { Nlow++; Nhi++; }

         // interpolate
         rec.accel = rec.sig_accel = 0.0;              // defaults
         double dt(ttag-ttag0), err, slope;
         if(haveClockDrift) {
            if(interpType == 2) {
               // Lagrange interpolation
               rec.bias = LagrangeInterpolation(times,biases,dt,err);      // sec
               rec.drift = LagrangeInterpolation(times,drifts,dt,err);     // sec/sec
            }
            else {
               // linear interpolation
               slope = (biases[Nhi]-biases[Nlow]) /
                                   (times[Nhi]-times[Nlow]);               // sec/sec
               rec.bias = biases[Nlow] + slope*(dt-times[Nlow]);           // sec
               slope = (drifts[Nhi]-drifts[Nlow])/(times[Nhi]-times[Nlow]);
               rec.drift = drifts[Nlow] + slope*(dt-times[Nlow]);          // sec/sec
            }

            // sigmas
            if(isExact)
               rec.sig_bias = sig_biases[Nmatch];
            else
               rec.sig_bias = RSS(sig_biases[Nhi],sig_biases[Nlow]);
            rec.sig_drift = RSS(sig_drifts[Nhi],sig_drifts[Nlow]);
         }
         else {                              // must interpolate biases to get drift
            if(interpType == 2) {
               // Lagrange interpolation
               LagrangeInterpolation(times,biases,dt,rec.bias,rec.drift);
            }
            else {
               // linear interpolation
               rec.drift = (biases[Nhi]-biases[Nlow]) /
                                   (times[Nhi]-times[Nlow]);            // sec/sec^2
               rec.bias = biases[Nlow] + (dt-times[Nlow])*rec.drift;    // sec/sec
            }

            // sigmas
            if(isExact)
               rec.sig_bias = sig_biases[Nmatch];
            else
               rec.sig_bias = RSS(sig_biases[Nhi],sig_biases[Nlow]);
            // TD ?
            rec.sig_drift = rec.sig_bias/(times[Nhi]-times[Nlow]);
         }

         if(haveClockAccel) {
            if(interpType == 2) {
               // Lagrange interpolation
               rec.accel = LagrangeInterpolation(times,accels,dt,err);  // sec/sec^2
            }
            else {
               // linear interpolation
               slope = (drifts[Nhi]-drifts[Nlow]) /
                                   (times[Nhi]-times[Nlow]);            // sec/sec^2
               rec.accel = accels[Nlow] + slope*(dt-times[Nlow]);       // sec/sec^2
            }

            // sigma
            if(isExact)
               rec.sig_accel = sig_accels[Nmatch];
            else
               rec.sig_accel = RSS(sig_accels[Nhi],sig_accels[Nlow]);
         }
         else if(haveClockDrift) {              // must interpolate drift to get accel
            if(interpType == 2) {
               // Lagrange interpolation  (err is a dummy here)
               LagrangeInterpolation(times,drifts,dt,err,rec.accel);
            }
            else {
               // linear interpolation                                  // sec/sec^2
               rec.accel = (drifts[Nhi]-drifts[Nlow]) / (times[Nhi]-times[Nlow]);
            }

            // sigmas  TD is there a better way?
            rec.sig_accel = rec.sig_drift/(times[Nhi]-times[Nlow]);
         }
         // else zero

         return rec;
      }
      catch(InvalidRequest& e) { GPSTK_RETHROW(e); }
   }

   // Return the clock bias for the given satellite at the given time
   // @param[in] sat the SatID of the satellite of interest
   // @param[in] ttag the time (CommonTime) of interest
   // @return double the clock bias
   // @throw InvalidRequest if bias cannot be computed, for example because
   //  a) the time t does not lie within the time limits of the data table
   //  b) checkDataGap is true and there is a data gap
   //  c) checkInterval is true and the interval is larger than maxInterval
   double ClockSatStore::getClockBias(const SatID& sat, const CommonTime& ttag)
      const throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         DataTableIterator it1, it2, kt;
         if(getTableInterval(sat, ttag, Nhalf, it1, it2, true)) {
            // exact match
            ClockRecord rec;
            rec = it1->second;
            return rec.bias;
         }

         // pull data out of the data table
         vector<double> times,biases;
         CommonTime ttag0(it1->first);
         kt = it1;
         while(1) {
            times.push_back(kt->first - ttag0);    // sec
            biases.push_back(kt->second.bias);     // sec
            if(kt == it2) break;
            ++kt;
         };

         // interpolate
         double bias, dt(ttag-ttag0), err, slope;
         if(interpType == 2) {                     // Lagrange interpolation
            bias = LagrangeInterpolation(times,biases,dt,err);    // sec
         }
         else {                                    // linear interpolation
            slope = (biases[Nhalf]-biases[Nhalf-1])/(times[Nhalf]-times[Nhalf-1]);
            bias = biases[Nhalf-1] + slope*(dt-times[Nhalf-1]);   // sec
         }

         return bias;
      }
      catch(InvalidRequest& e) { GPSTK_RETHROW(e); }
   }

   // Return the clock drift for the given satellite at the given time
   // @param[in] sat the SatID of the satellite of interest
   // @param[in] ttag the time (CommonTime) of interest
   // @return double the clock drift
   // @throw InvalidRequest if drift cannot be computed, for example because
   //  a) the time t does not lie within the time limits of the data table
   //  b) checkDataGap is true and there is a data gap
   //  c) checkInterval is true and the interval is larger than maxInterval
   //  d) there is no drift data in the store
   double ClockSatStore::getClockDrift(const SatID& sat, const CommonTime& ttag)
      const throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         DataTableIterator it1, it2, kt;
         bool isExact(getTableInterval(sat, ttag, Nhalf, it1, it2, haveClockDrift));
         if(isExact && haveClockDrift) {
            ClockRecord rec;
            rec = it1->second;
            return rec.drift;
         }

         // pull data out of the data table
         int n,Nhi(Nhalf);
         CommonTime ttag0(it1->first);
         vector<double> times,biases,drifts;

         n = 0;
         kt = it1;
         while(1) {
            if(isExact && ABS(kt->first-ttag) < 1.e-8) Nhi=n;
            times.push_back(kt->first - ttag0);    // sec
            if(haveClockDrift)
               drifts.push_back(kt->second.bias);  // sec/sec
            else
               biases.push_back(kt->second.bias);  // sec
            if(kt == it2) break;
            ++kt;
            ++n;
         };

         if(isExact && Nhi==Nhalf-1) Nhi++;

         // interpolate
         double drift, dt(ttag-ttag0), err, slope;
         if(haveClockDrift) {
            if(interpType == 2) {
               // Lagrange interpolation
               drift = LagrangeInterpolation(times,drifts,dt,err);         // sec/sec
            }
            else {
               // linear interpolation
               slope = (drifts[Nhi]-drifts[Nhi-1]) / (times[Nhi]-times[Nhi-1]);
               drift = drifts[Nhi-1] + slope*(dt-times[Nhi-1]);            // sec/sec
            }
         }
         else {
            if(interpType == 2) {
               // Lagrange interpolation // slope is dummy
               LagrangeInterpolation(times,biases,dt,slope,drift);
            }
            else {
               // linear interpolation
               drift = (biases[Nhi]-biases[Nhi-1])/(times[Nhi]-times[Nhi-1]);//sec/sec
            }
         }

         return drift;
      }
      catch(InvalidRequest& e) { GPSTK_RETHROW(e); }
   }

   // Add a ClockRecord to the store.
   void ClockSatStore::addClockRecord(const SatID& sat, const CommonTime& ttag,
                                      const ClockRecord& rec)
      throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         if(rec.drift != 0.0) haveClockDrift = true;
         if(rec.accel != 0.0) haveClockAccel = true;

         if(tables.find(sat) != tables.end() &&
            tables[sat].find(ttag) != tables[sat].end()) {
               // record already exists in the table
            ClockRecord& oldrec(tables[sat][ttag]);
            oldrec.bias = rec.bias;
            oldrec.sig_bias = rec.sig_bias;
            if(haveClockDrift) {
               oldrec.drift = rec.drift;
               oldrec.sig_drift = rec.sig_drift;
            }
            if(haveClockAccel) {
               oldrec.accel = rec.accel;
               oldrec.sig_accel = rec.sig_accel;
            }
         }
         else  // create a new entry in the table
            tables[sat][ttag] = rec;
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Add clock bias (only) data to the store
   void ClockSatStore::addClockBias(const SatID& sat, const CommonTime& ttag,
                                    const double& bias, const double& sig)
      throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         if(tables.find(sat) != tables.end() &&
            tables[sat].find(ttag) != tables[sat].end()) {
                  // record already exists in the table
            ClockRecord& oldrec(tables[sat][ttag]);
            oldrec.bias = bias;
            oldrec.sig_bias = sig;
         }
         else {   // create a new entry in the table
            ClockRecord rec;
            rec.bias = bias;
            rec.sig_bias = sig;
            rec.drift = rec.sig_drift = 0.0;
            rec.accel = rec.sig_accel = 0.0;

            tables[sat][ttag] = rec;
         }
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Add clock drift (only) data to the store
   void ClockSatStore::addClockDrift(const SatID& sat, const CommonTime& ttag,
                                    const double& drift, const double& sig)
      throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         haveClockDrift = true;

         if(tables.find(sat) != tables.end() &&
            tables[sat].find(ttag) != tables[sat].end()) {
                  // record already exists in the table
            ClockRecord& oldrec(tables[sat][ttag]);
            oldrec.drift = drift;
            oldrec.sig_drift = sig;
         }
         else {   // create a new entry in the table
            ClockRecord rec;
            rec.drift = drift;
            rec.sig_drift = sig;
            rec.bias = rec.sig_bias = 0.0;
            rec.accel = rec.sig_accel = 0.0;

            tables[sat][ttag] = rec;
         }
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

   // Add clock acceleration (only) data to the store
   void ClockSatStore::addClockAcceleration(const SatID& sat, const CommonTime& ttag,
                                    const double& accel, const double& sig)
      throw(InvalidRequest)
   {
      try {
         checkTimeSystem(ttag.getTimeSystem());

         haveClockAccel = true;

         if(tables.find(sat) != tables.end() &&
            tables[sat].find(ttag) != tables[sat].end()) {
                  // record already exists in the table
            ClockRecord& oldrec(tables[sat][ttag]);
            oldrec.accel = accel;
            oldrec.sig_accel = sig;
         }
         else {   // create a new entry in the table
            ClockRecord rec;
            rec.accel = accel;
            rec.sig_accel = sig;
            rec.drift = rec.sig_drift = 0.0;
            rec.bias = rec.sig_bias = 0.0;

            tables[sat][ttag] = rec;
         }
      }
      catch(InvalidRequest& ir) { GPSTK_RETHROW(ir); }
   }

}  // End of namespace gpstk
