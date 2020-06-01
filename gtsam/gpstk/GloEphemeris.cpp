/// @file GloEphemeris.cpp
/// Ephemeris data for GLONASS.

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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2011
//
//============================================================================

#include <iomanip>
#include "GloEphemeris.hpp"
#include "TimeString.hpp"

namespace gpstk
{


      /* Compute satellite position & velocity at the given time
       * using this ephemeris.
       *
       *  @throw InvalidRequest if required data has not been stored.
       */
   Xvt GloEphemeris::svXvt(const CommonTime& epoch) const
      throw( gpstk::InvalidRequest )
   {

         // Check that the given epoch is within the available time limits.
         // We have to add a margin of 15 minutes (900 seconds).
      if ( epoch <  (ephTime - 900.0) ||
           epoch >= (ephTime + 900.0)   )
      {
         InvalidRequest e( "Requested time is out of ephemeris data" );
         GPSTK_THROW(e);
      }

         // Values to be returned will be stored here
      Xvt sv;

         // If the exact epoch is found, let's return the values
      if ( epoch == ephTime )       // exact match for epoch
      {

         sv.x[0] = x[0]*1.e3;   // m
         sv.x[1] = x[1]*1.e3;   // m
         sv.x[2] = x[2]*1.e3;   // m
         sv.v[0] = v[0]*1.e3;  // m/sec
         sv.v[1] = v[1]*1.e3;  // m/sec
         sv.v[2] = v[2]*1.e3;  // m/sec

            // In the GLONASS system, 'clkbias' already includes the
            // relativistic correction, therefore we must substract the late
            // from the former.
         sv.relcorr = sv.computeRelativityCorrection();
         sv.clkbias = clkbias + clkdrift * (epoch - ephTime) - sv.relcorr;
         sv.clkdrift = clkdrift;
         sv.frame = ReferenceFrame::PZ90;

            // We are done, let's return
         return sv;

      }

         // Get the data out of the GloRecord structure
      double px( x[0] );   // X coordinate (km)
      double vx( v[0] );   // X velocity   (km/s)
      double ax( a[0] );   // X acceleration (km/s^2)
      double py( x[1] );   // Y coordinate
      double vy( v[1] );   // Y velocity
      double ay( a[1] );   // Y acceleration
      double pz( x[2] );   // Z coordinate
      double vz( v[2] );   // Z velocity
      double az( a[2] );   // Z acceleration

         // We will need some PZ-90 ellipsoid parameters
      PZ90Ellipsoid pz90;
      double we( pz90.angVelocity() );

         // Get sidereal time at Greenwich at 0 hours UT
      double gst( getSidTime( ephTime ) );
      double s0( gst*PI/12.0 );
      YDSTime ytime( ephTime );
      double numSeconds( ytime.sod );
      double s( s0 + we*numSeconds );
      double cs( std::cos(s) );
      double ss( std::sin(s) );

         // Initial state matrix
      Vector<double> initialState(6), accel(3), dxt1(6), dxt2(6), dxt3(6),
                     dxt4(6), tempRes(6);

         // Get the reference state out of GloEphemeris object data. Values
         // must be rotated from PZ-90 to an absolute coordinate system
         // Initial x coordinate (m)
      initialState(0)  = (px*cs - py*ss);
         // Initial y coordinate
      initialState(2)  = (px*ss + py*cs);
         // Initial z coordinate
      initialState(4)  = pz;

         // Initial x velocity   (m/s)
      initialState(1)  = (vx*cs - vy*ss - we*initialState(2) );
         // Initial y velocity
      initialState(3)  = (vx*ss + vy*cs + we*initialState(0) );
         // Initial z velocity
      initialState(5)  = vz;


         // Integrate satellite state to desired epoch using the given step
      double rkStep( step );

      if ( (epoch - ephTime) < 0.0 ) rkStep = step*(-1.0);
      CommonTime workEpoch( ephTime );

      double tolerance( 1e-9 );
      bool done( false );
      while (!done)
      {

            // If we are about to overstep, change the stepsize appropriately
            // to hit our target final time.
         if( rkStep > 0.0 )
         {
            if( (workEpoch + rkStep) > epoch )
               rkStep = (epoch - workEpoch);
         }
         else
         {
            if ( (workEpoch + rkStep) < epoch )
               rkStep = (epoch - workEpoch);
         }

         numSeconds += rkStep;
         s = s0 + we*( numSeconds );
         cs = std::cos(s);
         ss = std::sin(s);

            // Accelerations are computed once per iteration
         accel(0) = ax*cs - ay*ss;
         accel(1) = ax*ss + ay*cs;
         accel(2) = az;

         dxt1 = derivative( initialState, accel );
         for( int j = 0; j < 6; ++j )
            tempRes(j) = initialState(j) + rkStep*dxt1(j)/2.0;

         dxt2 = derivative( tempRes, accel );
         for( int j = 0; j < 6; ++j )
            tempRes(j) = initialState(j) + rkStep*dxt2(j)/2.0;

         dxt3 = derivative( tempRes, accel );
         for( int j = 0; j < 6; ++j )
            tempRes(j) = initialState(j) + rkStep*dxt3(j);

         dxt4 = derivative( tempRes, accel );
         for( int j = 0; j < 6; ++j )
            initialState(j) = initialState(j) + rkStep * ( dxt1(j)
                            + 2.0 * ( dxt2(j) + dxt3(j) ) + dxt4(j) ) / 6.0;


            // If we are within tolerance of the target time, we are done.
         workEpoch += rkStep;
         if ( std::fabs(epoch - workEpoch ) < tolerance )
            done = true;

      }  // End of 'while (!done)...'


      px = initialState(0);
      py = initialState(2);
      pz = initialState(4);
      vx = initialState(1);
      vy = initialState(3);
      vz = initialState(5);

      sv.x[0] = 1000.0*( px*cs + py*ss );         // X coordinate
      sv.x[1] = 1000.0*(-px*ss + py*cs);          // Y coordinate
      sv.x[2] = 1000.0*pz;                        // Z coordinate
      sv.v[0] = 1000.0*( vx*cs + vy*ss + we*(sv.x[1]/1000.0) ); // X velocity
      sv.v[1] = 1000.0*(-vx*ss + vy*cs - we*(sv.x[0]/1000.0) ); // Y velocity
      sv.v[2] = 1000.0*vz;                        // Z velocity

         // In the GLONASS system, 'clkbias' already includes the relativistic
         // correction, therefore we must substract the late from the former.
      sv.relcorr = sv.computeRelativityCorrection();
      sv.clkbias = clkbias + clkdrift * (epoch - ephTime) - sv.relcorr;
      sv.clkdrift = clkdrift;
      sv.frame = ReferenceFrame::PZ90;

         // We are done, let's return
      return sv;


   }  // End of method 'GloEphemeris::svXvt(const CommonTime& t)'


      // Get the epoch time for this ephemeris
   CommonTime GloEphemeris::getEphemerisEpoch() const
      throw( InvalidRequest )
   {

         // First, let's check if there is valid data
      if(!valid)
      {
         InvalidRequest exc("getEphemerisEpoch(): No valid data stored.");
         GPSTK_THROW(exc);
      }

      return ephTime;

   }  // End of method 'GloEphemeris::getEphemerisEpoch()'


      // This function returns the PRN ID of the SV.
   short GloEphemeris::getPRNID() const
      throw( InvalidRequest )
   {

         // First, let's check if there is valid data
      if(!valid)
      {
         InvalidRequest exc("getPRNID(): No valid data stored.");
         GPSTK_THROW(exc);
      }

      return PRNID;

   }  // End of method 'GloEphemeris::getPRNID()'


      /* Compute the satellite clock bias (sec) at the given time
       *
       * @param epoch   Epoch to compute satellite clock bias.
       *
       * @throw InvalidRequest if required data has not been stored.
       */
   double GloEphemeris::svClockBias(const CommonTime& epoch) const
      throw( gpstk::InvalidRequest )
   {

         // First, let's check if there is valid data
      if(!valid)
      {
         InvalidRequest exc("svClockBias(): No valid data stored.");
         GPSTK_THROW(exc);
      }

         // Auxiliar object
      Xvt sv;
      sv.x = x;
      sv.v = v;

         // In the GLONASS system, 'clkbias' already includes the relativistic
         // correction, therefore we must substract the late from the former.
      sv.relcorr = sv.computeRelativityCorrection();
      sv.clkbias = clkbias + clkdrift * (epoch - ephTime) - sv.relcorr;

      return sv.clkbias;

   }  // End of method 'GloEphemeris::svClockBias(const CommonTime& epoch)'


      /* Compute the satellite clock drift (sec/sec) at the given time
       *
       * @param epoch   Epoch to compute satellite clock drift.
       *
       * @throw InvalidRequest if required data has not been stored.
       */
   double GloEphemeris::svClockDrift(const CommonTime& epoch) const
      throw( gpstk::InvalidRequest )
   {

         // First, let's check if there is valid data
      if(!valid)
      {
         InvalidRequest exc("svClockDrift(): No valid data stored.");
         GPSTK_THROW(exc);
      }

      return clkdrift;

   }  // End of method 'GloEphemeris::svClockDrift(const CommonTime& epoch)'




      // Output the contents of this ephemeris to the given stream as a single line.
   void GloEphemeris::dump(std::ostream& s) const
      throw()
   {

      s << "Sys:" << satSys << ", PRN:" << PRNID
        << ", Epoch:" << ephTime << ", pos:" << x
        << ", vel:" << v << ", acc:" << a
        << ", TauN:" << clkbias << ", GammaN:" << clkdrift
         << ", MFTime:" << MFtime<< ", health:" << health
        << ", freqNum:" << freqNum << ", ageOfInfo:" << ageOfInfo;

   }  // End of method 'GloEphemeris::dump(std::ostream& s)'

   void GloEphemeris::prettyDump(std::ostream& s) const
   {
      s << "**********************************************" << std::endl;
      s << "Slot ID     " << std::setw(12) << PRNID << std::endl;
      s << "Epoch Time  " << printTime(ephTime,"%03j, %02m/%02d/%02y %02H:%02M:%02S")
                          << std::endl;
      s << "MFTime      " << std::setw(12) << MFtime << " sec of Week" << std::endl;
      s << "Health      " << std::setw(12) << health << std::endl;
      s << "Freq. Offset" << std::setw(12) << freqNum << std::endl;
      s << "Age of Info " << std::setw(12) << ageOfInfo << " days" << std::endl;
      
      s << "Position    " << std::setw(12) << x << "m" << std::endl;
      s << "Velocity    " << std::setw(12) << v << "m/sec" << std::endl;
      s << "Acceleration" << std::setw(12) << a << "m/sec**2" << std::endl;
      s << "TauN        " << std::setw(12) << clkbias << "units" << std::endl;
      s << "GammaN      " << std::setw(12) << clkdrift << "units" << std::endl;
   }

   void GloEphemeris::terseDump(std::ostream& s) const
   {
      s << " " << std::setw(2) << PRNID << " ";
      s << printTime(ephTime,"%03j, %02m/%02d/%02y %02H:%02M:%02S") << "  ";
      s << std::setw(6) << MFtime << "  ";
      s << std::setw(5) << health << "    ";
      s << std::setw(2) << freqNum << std::endl;
   }

   void GloEphemeris::terseHeader(std::ostream& s) const
   {
      s << "          Epoch time        Msg Time          Freq." << std::endl;
      s << "Slot DOY mm/dd/yy HH:MM:SS    (SOW)   Health Offset" << std::endl;
   }

      // Set the parameters for this ephemeris object.
   GloEphemeris& GloEphemeris::setRecord( std::string svSys,
                                          short prn,
                                          const CommonTime& epoch,
                                          Triple pos,
                                          Triple vel,
                                          Triple acc,
                                          double tau,
                                          double gamma,
                                          long mftime,
                                          short h,
                                          short freqnum,
                                          double age,
                                          double rkStep )
   {
      satSys   = svSys;
      PRNID    = prn;
      ephTime  = epoch;
      x        = pos;
      v        = vel;
      a        = acc;
      clkbias  = tau;
      clkdrift = gamma;
      MFtime   = mftime;
      health   = h;
      freqNum  = freqnum;
      ageOfInfo = age;

      step = rkStep;

         // Set this object as valid
      valid = true;

      return *this;

   }  // End of method 'GloEphemeris::setRecord()'


      // Compute true sidereal time  (in hours) at Greenwich at 0 hours UT.
   double GloEphemeris::getSidTime( const CommonTime& time ) const
   {

         // The following algorithm is based on the paper:
         // Aoki, S., Guinot,B., Kaplan, G. H., Kinoshita, H., McCarthy, D. D.
         //    and P.K. Seidelmann. 'The New Definition of Universal Time'.
         //    Astronomy and Astrophysics, 105, 359-361, 1982.

         // Get the Julian Day at 0 hours UT (jd)
      YDSTime ytime( time );
      double year( ytime.year );
      int doy( ytime.doy );
      int temp( floor(365.25 * (year - 1.0)) + doy );

      double jd( static_cast<double>(temp)+ 1721409.5 );

         // Compute the Julian centuries (36525 days)
      double jc( (jd - 2451545.0)/36525.0 );

         // Compute the sidereal time, in seconds
      double sid( 24110.54841 + jc * ( 8640184.812866
                              + jc * ( 0.093104 - jc * 0.0000062 ) ) );

      sid = sid / 3600.0;
      sid = fmod(sid, 24.0);
      if( sid < 0.0 ) sid = sid + 24.0;

      return sid;

   }; // End of method 'GloEphemeris::getSidTime()'


      // Function implementing the derivative of GLONASS orbital model.
   Vector<double> GloEphemeris::derivative( const Vector<double>& inState,
                                            const Vector<double>& accel )
      const
   {

         // We will need some important PZ90 ellipsoid values
      PZ90Ellipsoid pz90;
      const double j20( pz90.j20() );
      const double mu( pz90.gm_km() );
      const double ae( pz90.a_km() );

         // Let's start getting the current satellite position and velocity
      double  x( inState(0) );          // X coordinate
      double vx( inState(1) );          // X velocity
      double  y( inState(2) );          // Y coordinate
      double vy( inState(3) );          // Y velocity
      double  z( inState(4) );          // Z coordinate
      double vz( inState(5) );          // Z velocity
#pragma unused(vx,vy,vz)

      double r2( x*x + y*y + z*z );
      double r( std::sqrt(r2) );
      double xmu( mu/r2 );
      double rho( ae/r );
      double xr( x/r );
      double yr( y/r );
      double zr( z/r );
      double zr2( zr*zr );
      double k1(j20*xmu*1.5*rho*rho);
      double  cm( k1*(1.0-5.0*zr2) );
      double cmz( k1*(3.0-5.0*zr2) );
      double k2(cm-xmu);

      double gloAx( k2*xr + accel(0) );
      double gloAy( k2*yr + accel(1) );
      double gloAz( (cmz-xmu)*zr + accel(2) );

      Vector<double> dxt(6, 0.0);

         // Let's insert data related to X coordinates
      dxt(0) = inState(1);       // Set X'  = Vx
      dxt(1) = gloAx;            // Set Vx' = gloAx

         // Let's insert data related to Y coordinates
      dxt(2) = inState(3);       // Set Y'  = Vy
      dxt(3) = gloAy;            // Set Vy' = gloAy

         // Let's insert data related to Z coordinates
      dxt(4) = inState(5);       // Set Z'  = Vz
      dxt(5) = gloAz;            // Set Vz' = gloAz

      return dxt;

   }  // End of method 'GloEphemeris::derivative()'


      // Output the contents of this ephemeris to the given stream.
   std::ostream& operator<<( std::ostream& s, const GloEphemeris& glo )
   {

      glo.dump(s);
      return s;

   }  // End of 'std::ostream& operator<<()'


}  // End of namespace gpstk

