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
 * @file TropModel.cpp
 * Base class for tropospheric models, plus implementations of several
 * published models
 */

#include "TropModel.hpp"
#include "EphemerisRange.hpp"             // for Elevation()
#include "MathBase.hpp"                   // SQRT
#include "geometry.hpp"                   // DEG_TO_RAD
#include "GPSEllipsoid.hpp"               // ell.a() = R earth
#include "GNSSconstants.hpp"          // TWO_PI
#include "YDSTime.hpp"

namespace gpstk
{
      // for temperature conversion from Celcius to Kelvin
   static const double CELSIUS_TO_KELVIN = 273.15;

      // Compute and return the full tropospheric delay. Typically call
      // setWeather(T,P,H) before making this call.
      // @param elevation Elevation of satellite as seen at receiver, in degrees
   double TropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      if(elevation < 0.0)
         return 0.0;

      return (dry_zenith_delay() * dry_mapping_function(elevation)
            + wet_zenith_delay() * wet_mapping_function(elevation));

   }  // end TropModel::correction(elevation)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag may
      // vary; it computes the elevation (and other receiver location information)
      // and passes them to appropriate set...() routines and the
      // correction(elevation) routine.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time tag of the signal
   double TropModel::correction(const Position& RX,
                                const Position& SV,
                                const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      double c;
      try
      {
         c = correction(RX.elevation(SV));
      }
      catch(InvalidTropModel& e)
      {
         GPSTK_RETHROW(e);
      }
      return c;
   }  // end TropModel::correction(RX,SV,TT)

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   void TropModel::setWeather(const double& T,
                              const double& P,
                              const double& H)
      throw(InvalidParameter)
   {
      temp = T + CELSIUS_TO_KELVIN;
      press = P;
      humid = H;
      if (temp < 0.0)
      {
         valid = false;
         GPSTK_THROW(InvalidParameter("Invalid temperature parameter."));
      }
      if (press < 0.0)
      {
         valid = false;
         GPSTK_THROW(InvalidParameter("Invalid pressure parameter."));
      }
      if (humid < 0.0 || humid > 100.0)
      {
         valid = false;
         GPSTK_THROW(InvalidParameter("Invalid humidity parameter."));
      }
   }  // end TropModel::setWeather(T,P,H)

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void TropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      if (wx.isAllValid())
      {
         try
         {
            setWeather(wx.temperature, wx.pressure, wx.humidity);
            valid = true;
         }
         catch(InvalidParameter& e)
         {
            valid = false;
            GPSTK_RETHROW(e);
         }
      }
      else
      {
         valid = false;
         GPSTK_THROW(InvalidParameter("Invalid weather data"));
      }
   }
      /// get weather data by a standard atmosphere model
      /// reference to white paper of Bernese 5.0, P243
      /// @param ht     Height of the receiver in meters.
      /// @param T      temperature in degrees Celsius
      /// @param P      atmospheric pressure in millibars
      /// @param H      relative humidity in percent
   void TropModel::weatherByStandardAtmosphereModel(const double& ht, double& T, double& P, double& H)
   {

         // reference height and it's relate weather(T P H)
      const double h0  = 0.0;			   // meter
      const double Tr  = +18.0;	      // Celsius
      const double pr  = 1013.25;		// millibarc
      const double Hr  = 50;			   // humidity

      T = Tr-0.0065*(ht-h0);
      P = pr * std::pow((1 - 0.0000226 * (ht - h0)), 5.225);
      H = Hr * std::exp(-0.0006396 * (ht - h0));

   }

   // -----------------------------------------------------------------------
   // Simple Black model. This has been used as the 'default' for many years.

      // Default constructor
   SimpleTropModel::SimpleTropModel(void)
   {
      setWeather(20.0, 980.0, 50.0);
      Cwetdelay = 0.122382715318184;
      Cdrydelay = 2.235486646978727;
      Cwetmap = 1.000282213715744;
      Cdrymap = 1.001012704615527;
      valid = true;
   }

      // Creates a trop model from a weather observation
      // @param wx the weather to use for this correction.
   SimpleTropModel::SimpleTropModel(const WxObservation& wx)
      throw(InvalidParameter)
   {
      setWeather(wx);
      valid = true;
   }

      // Create a tropospheric model from explicit weather data
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   SimpleTropModel::SimpleTropModel(const double& T,
                                    const double& P,
                                    const double& H)
      throw(InvalidParameter)
   {
      setWeather(T,P,H);
      valid = true;
   }

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   void SimpleTropModel::setWeather(const double& T,
                                    const double& P,
                                    const double& H)
      throw(InvalidParameter)
   {
      TropModel::setWeather(T,P,H);
      GPSEllipsoid ell;
      Cdrydelay = 2.343*(press/1013.25)*(temp-3.96)/temp;
      double tks = temp * temp;
      Cwetdelay = 8.952/tks*humid*std::exp(-37.2465+0.213166*temp-(0.256908e-3)*tks);
      Cdrymap =1.0+(0.15)*148.98*(temp-3.96)/ell.a();
      Cwetmap =1.0+(0.15)*12000.0/ell.a();
      valid = true;
   }  // end SimpleTropModel::setWeather(T,P,H)

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void SimpleTropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      TropModel::setWeather(wx);
   }

      // Compute and return the zenith delay for dry component of the troposphere
   double SimpleTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      return Cdrydelay;
   }

      // Compute and return the zenith delay for wet component of the troposphere
   double SimpleTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      return Cwetdelay;
   }

      // Compute and return the mapping function for dry component
      // of the troposphere
      // @param elevation is the Elevation of satellite as seen at receiver,
      //                  in degrees
   double SimpleTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      if(elevation < 0.0) return 0.0;

      double d = std::cos(elevation*DEG_TO_RAD);
      d /= Cdrymap;
      return (1.0/SQRT(1.0-d*d));
   }

      // Compute and return the mapping function for wet component
      // of the troposphere
      // @param elevation is the Elevation of satellite as seen at receiver,
      //                  in degrees
   double SimpleTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      if(elevation < 0.0) return 0.0;

      double d = std::cos(elevation*DEG_TO_RAD);
      d /= Cwetmap;
      return (1.0/SQRT(1.0-d*d));
   }

   // ------------------------------------------------------------------------
   // Tropospheric model based on Goad and Goodman(1974),
   // "A Modified Hopfield Tropospheric Refraction Correction Model," Paper
   // presented at the Fall Annual Meeting of the American Geophysical Union,
   // San Francisco, December 1974.
   // See Leick, "GPS Satellite Surveying," Wiley, NY, 1990, Chapter 9,
   // particularly Table 9.1.
   // ------------------------------------------------------------------------

   static const double GGdryscale = 8594.777388436570600;
   static const double GGwetscale = 2540.042008403690900;

      // Default constructor
   GGTropModel::GGTropModel(void)
   {
      TropModel::setWeather(20.0, 980.0, 50.0);
      Cdrydelay = 2.59629761092150147e-4;    // zenith delay, dry
      Cwetdelay = 4.9982784999977412e-5;     // zenith delay, wet
      Cdrymap = 42973.886942182834900;       // height for mapping, dry
      Cwetmap = 12700.210042018454260;       // height for mapping, wet
      valid = true;
   }  // end GGTropModel::GGTropModel()

      // Creates a trop model from a weather observation
      // @param wx the weather to use for this correction.
   GGTropModel::GGTropModel(const WxObservation& wx)
      throw(InvalidParameter)
   {
      setWeather(wx);
      valid = true;
   }  // end GGTropModel::GGTropModel(weather)

      // Create a tropospheric model from explicit weather data
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   GGTropModel::GGTropModel(const double& T,
                            const double& P,
                            const double& H)
      throw(InvalidParameter)
   {
      setWeather(T,P,H);
      valid = true;
   } // end GGTropModel::GGTropModel()

   double GGTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      return (Cdrydelay * GGdryscale);
   }  // end GGTropModel::dry_zenith_delay()

   double GGTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      return (Cwetdelay * GGwetscale);
   }  // end GGTropModel::wet_zenith_delay()

   double GGTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      if(elevation < 0.0) return 0.0;

      GPSEllipsoid ell;
      double ce=std::cos(elevation*DEG_TO_RAD), se=std::sin(elevation*DEG_TO_RAD);
      double ad = -se/Cdrymap;
      double bd = -ce*ce/(2.0*ell.a()*Cdrymap);
      double Rd = SQRT((ell.a()+Cdrymap)*(ell.a()+Cdrymap)
                - ell.a()*ell.a()*ce*ce) - ell.a()*se;

      double Ad[9], ad2=ad*ad, bd2=bd*bd;
      Ad[0] = 1.0;
      Ad[1] = 4.0*ad;
      Ad[2] = 6.0*ad2 + 4.0*bd;
      Ad[3] = 4.0*ad*(ad2+3.0*bd);
      Ad[4] = ad2*ad2 + 12.0*ad2*bd + 6.0*bd2;
      Ad[5] = 4.0*ad*bd*(ad2+3.0*bd);
      Ad[6] = bd2*(6.0*ad2+4.0*bd);
      Ad[7] = 4.0*ad*bd*bd2;
      Ad[8] = bd2*bd2;

         // compute dry component of the mapping function
      double sumd=0.0;
      for(int j=9; j>=1; j--) {
         sumd += Ad[j-1]/double(j);
         sumd *= Rd;
      }
      return sumd/GGdryscale;

   }  // end GGTropModel::dry_mapping_function(elev)

      // compute wet component of the mapping function
   double GGTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
         GPSTK_THROW(InvalidTropModel("Invalid model"));

      if(elevation < 0.0) return 0.0;

      GPSEllipsoid ell;
      double ce = std::cos(elevation*DEG_TO_RAD), se = std::sin(elevation*DEG_TO_RAD);
      double aw = -se/Cwetmap;
      double bw = -ce*ce/(2.0*ell.a()*Cwetmap);
      double Rw = SQRT((ell.a()+Cwetmap)*(ell.a()+Cwetmap)
                - ell.a()*ell.a()*ce*ce) - ell.a()*se;

      double Aw[9], aw2=aw*aw, bw2=bw*bw;
      Aw[0] = 1.0;
      Aw[1] = 4.0*aw;
      Aw[2] = 6.0*aw2 + 4.0*bw;
      Aw[3] = 4.0*aw*(aw2+3.0*bw);
      Aw[4] = aw2*aw2 + 12.0*aw2*bw + 6.0*bw2;
      Aw[5] = 4.0*aw*bw*(aw2+3.0*bw);
      Aw[6] = bw2*(6.0*aw2+4.0*bw);
      Aw[7] = 4.0*aw*bw*bw2;
      Aw[8] = bw2*bw2;

      double sumw=0.0;
      for(int j=9; j>=1; j--) {
         sumw += Aw[j-1]/double(j);
         sumw *= Rw;
      }
      return sumw/GGwetscale;

   }  // end GGTropModel::wet_mapping_function(elev)

   void GGTropModel::setWeather(const double& T,
                                const double& P,
                                const double& H)
      throw(InvalidParameter)
   {
      TropModel::setWeather(T,P,H);
      double th=300./temp;
         // water vapor partial pressure (mb)
         // this comes from Leick and is not good.
         // double wvpp=6.108*(RHum*0.01)*exp((17.15*Tk-4684.0)/(Tk-38.45));
      double wvpp=2.409e9*humid*th*th*th*th*std::exp(-22.64*th);
      Cdrydelay = 7.7624e-5*press/temp;
      Cwetdelay = 1.0e-6*(-12.92+3.719e+05/temp)*(wvpp/temp);
      Cdrymap = (5.0*0.002277*press)/Cdrydelay;
      Cwetmap = (5.0*0.002277/Cwetdelay)*(1255.0/temp+0.5)*wvpp;
      valid = true;
   }  // end GGTropModel::setWeather(T,P,H)

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void GGTropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      TropModel::setWeather(wx);
   }

   // ------------------------------------------------------------------------
   // Tropospheric model with heights based on Goad and Goodman(1974),
   // "A Modified Hopfield Tropospheric Refraction Correction Model," Paper
   // presented at the Fall Annual Meeting of the American Geophysical Union,
   // San Francisco, December 1974.
   // (Not the same as GGTropModel because this has height dependence,
   // and the computation of this model does not break cleanly into
   // wet and dry components.)

      // Default constructor
   GGHeightTropModel::GGHeightTropModel(void)
   {
      validWeather = false; //setWeather(20.0,980.0,50.0);
      validHeights = false; //setHeights(0.0,0.0,0.0);
      validRxHeight = false;
   }

      // Creates a trop model from a weather observation
      // @param wx the weather to use for this correction.
   GGHeightTropModel::GGHeightTropModel(const WxObservation& wx)
      throw(InvalidParameter)
   {
      valid = validRxHeight = validHeights = false;
      setWeather(wx);
   }

      // Create a tropospheric model from explicit weather data
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   GGHeightTropModel::GGHeightTropModel(const double& T,
                                        const double& P,
                                        const double& H)
      throw(InvalidParameter)
   {
      validRxHeight = validHeights = false;
      setWeather(T,P,H);
   }

      // Create a valid model from explicit input.
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
      // @param hT height at which temperature applies in meters.
      // @param hP height at which atmospheric pressure applies in meters.
      // @param hH height at which relative humidity applies in meters.
   GGHeightTropModel::GGHeightTropModel(const double& T,
                                        const double& P,
                                        const double& H,
                                        const double hT,
                                        const double hP,
                                        const double hH)
      throw(InvalidParameter)
   {
      validRxHeight = false;
      setWeather(T,P,H);
      setHeights(hT,hP,hH);
   }

      // re-define this to get the throws
   double GGHeightTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
      {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }
      if(elevation < 0.0) return 0.0;

      return (dry_zenith_delay() * dry_mapping_function(elevation)
            + wet_zenith_delay() * wet_mapping_function(elevation));

   }  // end GGHeightTropModel::correction(elevation)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag
      // may vary; it computes the elevation (and other receiver location
      // information) and passes them to appropriate set...() routines and
      // the correction(elevation) routine.
      // @param RX  Receiver position
      // @param SV  Satellite position
      // @param tt  Time tag of the signal
   double GGHeightTropModel::correction(const Position& RX,
                                        const Position& SV,
                                        const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
      {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }

      // compute height from RX
      setReceiverHeight(RX.getHeight());

      return TropModel::correction(RX.elevation(SV));

   }  // end GGHeightTropModel::correction(RX,SV,TT)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag
      // may vary; it computes the elevation (and other receiver location
      // information) and passes them to appropriate set...() routines and
      // the correction(elevation) routine.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time tag of the signal
      // This function is deprecated; use the Position version
   double GGHeightTropModel::correction(const Xvt& RX,
                                        const Xvt& SV,
                                        const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      Position R(RX),S(SV);
      return GGHeightTropModel::correction(R,S,tt);
   }

      // Compute and return the zenith delay for dry component of the troposphere
   double GGHeightTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }
      double hrate=6.5e-3;
      double Ts=temp+hrate*height;
      double em=978.77/(2.8704e4*hrate);
      double Tp=Ts-hrate*hpress;
      double ps=press*std::pow(Ts/Tp,em)/1000.0;
      double rs=77.624e-3/Ts;
      double ho=11.385/rs;
      rs *= ps;
      double zen=(ho-height)/ho;
      zen = rs*zen*zen*zen*zen;
         // normalize
      zen *= (ho-height)/5;
      return zen;

   }  // end GGHeightTropModel::dry_zenith_delay()

      // Compute and return the zenith delay for wet component of the troposphere
   double GGHeightTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }
      double hrate=6.5e-3; //   deg K / m
      double Th=temp-273.15-hrate*(hhumid-htemp);
      double Ta=7.5*Th/(237.3+Th);
         // water vapor partial pressure
      double e0=6.11e-5*humid*std::pow(10.0,Ta);
      double Ts=temp+hrate*htemp;
      double em=978.77/(2.8704e4*hrate);
      double Tk=Ts-hrate*hhumid;
      double es=e0*std::pow(Ts/Tk,4.0*em);
      double rs=(371900.0e-3/Ts-12.92e-3)/Ts;
      double ho=11.385*(1255/Ts+0.05)/rs;
      double zen=(ho-height)/ho;
      zen = rs*es*zen*zen*zen*zen;
         //normalize
      zen *= (ho-height)/5;
      return zen;

   }  // end GGHeightTropModel::wet_zenith_delay()

      // Compute and return the mapping function for dry component
      // of the troposphere
      // @param elevation Elevation of satellite as seen at receiver,
      //                  in degrees
   double GGHeightTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }
      if(elevation < 0.0) return 0.0;

      double hrate=6.5e-3;
      double Ts=temp+hrate*htemp;
      double ho=(11.385/77.624e-3)*Ts;
      double se=std::sin(elevation*DEG_TO_RAD);
      if(se < 0.0) se=0.0;

      GPSEllipsoid ell;
      double rt,a,b,rn[8],al[8],er=ell.a();
      rt = (er+ho)/(er+height);
      rt = rt*rt - (1.0-se*se);
      if(rt < 0) rt=0.0;
      rt = (er+height)*(SQRT(rt)-se);
      a = -se/(ho-height);
      b = -(1.0-se*se)/(2.0*er*(ho-height));
      rn[0] = rt*rt;
      for(int j=1; j<8; j++) rn[j]=rn[j-1]*rt;
      al[0] = 2*a;
      al[1] = 2*a*a+4*b/3;
      al[2] = a*(a*a+3*b);
      al[3] = a*a*a*a/5+2.4*a*a*b+1.2*b*b;
      al[4] = 2*a*b*(a*a+3*b)/3;
      al[5] = b*b*(6*a*a+4*b)*0.1428571;
      if(b*b > 1.0e-35) {
         al[6] = a*b*b*b/2;
         al[7] = b*b*b*b/9;
      } else {
         al[6] = 0.0;
         al[7] = 0.0;
      }
      double map=rt;
      for(int k=0; k<8; k++) map += al[k]*rn[k];
         // normalize
      double norm=(ho-height)/5;
      return map/norm;

   }  // end GGHeightTropModel::dry_mapping_function(elevation)

      // Compute and return the mapping function for wet component
      // of the troposphere
      // @param elevation Elevation of satellite as seen at receiver,
      //                  in degrees
   double GGHeightTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Weather"));
         if(!validHeights)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Heights"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid GGH trop model: Rx Height"));
      }
      if(elevation < 0.0) return 0.0;

      double hrate=6.5e-3;
      double Ts=temp+hrate*htemp;
      double rs=(371900.0e-3/Ts-12.92e-3)/Ts;
      double ho=11.385*(1255/Ts+0.05)/rs;
      double se=std::sin(elevation*DEG_TO_RAD);
      if(se < 0.0) se=0.0;

      GPSEllipsoid ell;
      double rt,a,b,rn[8],al[8],er=ell.a();
      rt = (er+ho)/(er+height);
      rt = rt*rt - (1.0-se*se);
      if(rt < 0) rt=0.0;
      rt = (er+height)*(SQRT(rt)-se);
      a = -se/(ho-height);
      b = -(1.0-se*se)/(2.0*er*(ho-height));
      rn[0] = rt*rt;
      for(int i=1; i<8; i++) rn[i]=rn[i-1]*rt;
      al[0] = 2*a;
      al[1] = 2*a*a+4*b/3;
      al[2] = a*(a*a+3*b);
      al[3] = a*a*a*a/5+2.4*a*a*b+1.2*b*b;
      al[4] = 2*a*b*(a*a+3*b)/3;
      al[5] = b*b*(6*a*a+4*b)*0.1428571;
      if(b*b > 1.0e-35) {
         al[6] = a*b*b*b/2;
         al[7] = b*b*b*b/9;
      } else {
         al[6] = 0.0;
         al[7] = 0.0;
      }
      double map=rt;
      for(int j=0; j<8; j++) map += al[j]*rn[j];
         // normalize map function
      double norm=(ho-height)/5;
      return map/norm;

   }  // end GGHeightTropModel::wet_mapping_function(elevation)

      // Re-define the weather data.
      // Typically called just before correction().
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   void GGHeightTropModel::setWeather(const double& T,
                                      const double& P,
                                      const double& H)
      throw(InvalidParameter)
   {
      try
      {
         TropModel::setWeather(T,P,H);
         validWeather = true;
         valid = validWeather && validHeights && validRxHeight;
      }
      catch(InvalidParameter& e)
      {
         valid = validWeather = false;
         GPSTK_RETHROW(e);
      }
   }  // end GGHeightTropModel::setWeather(T,P,H)

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void GGHeightTropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      try
      {
         TropModel::setWeather(wx);
         validWeather = true;
         valid = validWeather && validHeights && validRxHeight;
      }
      catch(InvalidParameter& e)
      {
         valid = validWeather = false;
         GPSTK_RETHROW(e);
      }
   }


      // Re-define the heights at which the weather parameters apply.
      // Typically called just before correction().
      // @param hT height (m) at which temperature applies
      // @param hP height (m) at which atmospheric pressure applies
      // @param hH height (m) at which relative humidity applies
   void GGHeightTropModel::setHeights(const double& hT,
                                      const double& hP,
                                      const double& hH)
   {
      htemp = hT;                 // height (m) at which temp applies
      hpress = hP;                // height (m) at which press applies
      hhumid = hH;                // height (m) at which humid applies
      validHeights = true;
      valid = validWeather && validHeights && validRxHeight;
   }  // end GGHeightTropModel::setHeights(hT,hP,hH)

      // Define the receiver height; this required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void GGHeightTropModel::setReceiverHeight(const double& ht)
   {
      height = ht;
      validRxHeight = true;
      if(!validHeights) {
         htemp = hpress = hhumid = ht;
         validHeights = true;
      }
      valid = validWeather && validHeights && validRxHeight;
   }  // end GGHeightTropModel::setReceiverHeight(const double& ht)

   // ------------------------------------------------------------------------
   // Tropospheric model developed by University of New Brunswick and described in
   // "A Tropospheric Delay Model for the User of the Wide Area Augmentation
   // System," J. Paul Collins and Richard B. Langley, Technical Report No. 187,
   // Dept. of Geodesy and Geomatics Engineering, University of New Brunswick,
   // 1997. See particularly Appendix C.
   //
   // This model includes a wet and dry component, and was designed for the user
   // without access to measurements of temperature, pressure and relative humidity
   // at ground level. It requires input of the latitude, day of year and height
   // above the ellipsoid, and it interpolates a table of values, using these
   // inputs, to get the ground level weather parameters plus other parameters and
   // the mapping function constants.
   //
   // NB in this model, units of temp are degrees Celsius, and humid is the water
   // vapor partial pressure.

   static const double NBRd=287.054;   // J/kg/K = m*m*K/s*s
   static const double NBg=9.80665;    // m/s*s
   static const double NBk1=77.604;    // K/mbar
   static const double NBk3p=382000;   // K*K/mbar

   static const double NBLat[5]={   15.0,   30.0,   45.0,   60.0,   75.0};

   // zenith delays, averages
   static const double NBZP0[5]={1013.25,1017.25,1015.75,1011.75,1013.00};
   static const double NBZT0[5]={ 299.65, 294.15, 283.15, 272.15, 263.65};
   static const double NBZW0[5]={  26.31,  21.79,  11.66,   6.78,   4.11};
   static const double NBZB0[5]={6.30e-3,6.05e-3,5.58e-3,5.39e-3,4.53e-3};
   static const double NBZL0[5]={   2.77,   3.15,   2.57,   1.81,   1.55};

   // zenith delays, amplitudes
   static const double NBZPa[5]={    0.0,  -3.75,  -2.25,  -1.75,  -0.50};
   static const double NBZTa[5]={    0.0,    7.0,   11.0,   15.0,   14.5};
   static const double NBZWa[5]={    0.0,   8.85,   7.24,   5.36,   3.39};
   static const double NBZBa[5]={    0.0,0.25e-3,0.32e-3,0.81e-3,0.62e-3};
   static const double NBZLa[5]={    0.0,   0.33,   0.46,   0.74,   0.30};

   // mapping function, dry, averages
   static const double NBMad[5]={1.2769934e-3,1.2683230e-3,1.2465397e-3,1.2196049e-3,
                                 1.2045996e-3};
   static const double NBMbd[5]={2.9153695e-3,2.9152299e-3,2.9288445e-3,2.9022565e-3,
                                 2.9024912e-3};
   static const double NBMcd[5]={62.610505e-3,62.837393e-3,63.721774e-3,63.824265e-3,
                                 64.258455e-3};

   // mapping function, dry, amplitudes
   static const double NBMaa[5]={0.0,1.2709626e-5,2.6523662e-5,3.4000452e-5,
                                 4.1202191e-5};
   static const double NBMba[5]={0.0,2.1414979e-5,3.0160779e-5,7.2562722e-5,
                                 11.723375e-5};
   static const double NBMca[5]={0.0,9.0128400e-5,4.3497037e-5,84.795348e-5,
                                 170.37206e-5};

   // mapping function, wet, averages (there are no amplitudes for wet)
   static const double NBMaw[5]={5.8021897e-4,5.6794847e-4,5.8118019e-4,5.9727542e-4,
                           6.1641693e-4};
   static const double NBMbw[5]={1.4275268e-3,1.5138625e-3,1.4572752e-3,1.5007428e-3,
                           1.7599082e-3};
   static const double NBMcw[5]={4.3472961e-2,4.6729510e-2,4.3908931e-2,4.4626982e-2,
                           5.4736038e-2};

   // labels for use with the interpolation routine
   enum TableEntry { ZP=1, ZT, ZW, ZB, ZL, Mad, Mbd, Mcd, Maw, Mbw, Mcw };

   // the interpolation routine
   static double NB_Interpolate(double lat, int doy, TableEntry entry)
   {
      const double *pave, *pamp;
      double ret, day=double(doy);

         // assign pointer to the right array
      switch(entry) {
         case ZP:  pave=&NBZP0[0]; pamp=&NBZPa[0]; break;
         case ZT:  pave=&NBZT0[0]; pamp=&NBZTa[0]; break;
         case ZW:  pave=&NBZW0[0]; pamp=&NBZWa[0]; break;
         case ZB:  pave=&NBZB0[0]; pamp=&NBZBa[0]; break;
         case ZL:  pave=&NBZL0[0]; pamp=&NBZLa[0]; break;
         case Mad: pave=&NBMad[0]; pamp=&NBMaa[0]; break;
         case Mbd: pave=&NBMbd[0]; pamp=&NBMba[0]; break;
         case Mcd: pave=&NBMcd[0]; pamp=&NBMca[0]; break;
         case Maw: pave=&NBMaw[0];                 break;
         case Mbw: pave=&NBMbw[0];                 break;
         case Mcw: pave=&NBMcw[0];                 break;
      }

         // find the index i such that NBLat[i] <= lat < NBLat[i+1]
      int i = int(ABS(lat)/15.0)-1;

      if(i>=0 && i<=3) {               // mid-latitude -> regular interpolation
         double m=(ABS(lat)-NBLat[i])/(NBLat[i+1]-NBLat[i]);
         ret = pave[i]+m*(pave[i+1]-pave[i]);
         if(entry < Maw)
            ret -= (pamp[i]+m*(pamp[i+1]-pamp[i]))
               * std::cos(TWO_PI*(day-28.0)/365.25);
      }
      else {                           // < 15 or > 75 -> simpler interpolation
         if(i<0) i=0; else i=4;
         ret = pave[i];
         if(entry < Maw)
            ret -= pamp[i]*std::cos(TWO_PI*(day-28.0)/365.25);
      }

      return ret;

   }  // end double NB_Interpolate(lat,doy,entry)

      // Default constructor
   NBTropModel::NBTropModel(void)
   {
      validWeather = false;
      validRxLatitude = false;
      validDOY = false;
      validRxHeight = false;
   } // end NBTropModel::NBTropModel()

      // Create a trop model using the minimum information: latitude and doy.
      // Interpolate the weather unless setWeather (optional) is called.
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
   NBTropModel::NBTropModel(const double& lat,
                            const int& day)
   {
      validRxHeight = false;
      setReceiverLatitude(lat);
      setDayOfYear(day);
      setWeather();
   }  // end NBTropModel::NBTropModel(lat, day);

      // Create a trop model with weather.
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
      // @param wx the weather to use for this correction.
   NBTropModel::NBTropModel(const double& lat,
                            const int& day,
                            const WxObservation& wx)
      throw(InvalidParameter)
   {
      validRxHeight = false;
      setReceiverLatitude(lat);
      setDayOfYear(day);
      setWeather(wx);
   }  // end NBTropModel::NBTropModel(weather)

      // Create a tropospheric model from explicit weather data
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   NBTropModel::NBTropModel(const double& lat,
                            const int& day,
                            const double& T,
                            const double& P,
                            const double& H)
      throw(InvalidParameter)
   {
      validRxHeight = false;
      setReceiverLatitude(lat);
      setDayOfYear(day);
      setWeather(T,P,H);
   } // end NBTropModel::NBTropModel()

      // Create a valid model from explicit input (weather will be estimated
      // internally by this model).
      // @param ht Height of the receiver in meters.
      // @param lat Latitude of the receiver in degrees.
      // @param d Day of year.
   NBTropModel::NBTropModel(const double& ht,
                            const double& lat,
                            const int& day)
   {
      setReceiverHeight(ht);
      setReceiverLatitude(lat);
      setDayOfYear(day);
      setWeather();
   }

      // re-define this to get the throws
   double NBTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: weather"));
         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }

      if(elevation < 0.0) return 0.0;

      return (dry_zenith_delay() * dry_mapping_function(elevation)
            + wet_zenith_delay() * wet_mapping_function(elevation));
   }  // end NBTropModel::correction(elevation)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag
      // may vary; it computes the elevation (and other receiver location
      // information) and passes them to appropriate set...() routines
      // and the correction(elevation) routine.
      // @param RX  Receiver position
      // @param SV  Satellite position
      // @param tt  Time tag of the signal
   double NBTropModel::correction(const Position& RX,
                                  const Position& SV,
                                  const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)

         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }

         // compute height and latitude from RX
      setReceiverHeight(RX.getHeight());
      setReceiverLatitude(RX.getGeodeticLatitude());

         // compute day of year from tt
      setDayOfYear(int((static_cast<YDSTime>(tt)).doy));

      return TropModel::correction(RX.elevation(SV));

   }  // end NBTropModel::correction(RX,SV,TT)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag
      // may vary; it computes the elevation (and other receiver location
      // information) and passes them to appropriate set...() routines
      // and the correction(elevation) routine.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time tag of the signal
      // This function is deprecated; use the Position version
   double NBTropModel::correction(const Xvt& RX,
                                  const Xvt& SV,
                                  const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      Position R(RX),S(SV);
      return NBTropModel::correction(R,S,tt);
   }

      // Compute and return the zenith delay for dry component of the troposphere
   double NBTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: weather"));
         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }
      double beta = NB_Interpolate(latitude,doy,ZB);
      double gm = 9.784*(1.0-2.66e-3*std::cos(2.0*latitude*DEG_TO_RAD)-2.8e-7*height);

         // scale factors for height above mean sea level
         // if weather is given, assume it's measured at ht -> kw=kd=1
      double kd=1, base=std::log(1.0-beta*height/temp);
      if(interpolateWeather)
         kd = std::exp(base*NBg/(NBRd*beta));

         // compute the zenith delay for dry component
      return ((1.0e-6*NBk1*NBRd/gm) * kd * press);

   }  // end NBTropModel::dry_zenith_delay()

      // Compute and return the zenith delay for wet component of the troposphere
   double NBTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: weather"));
         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }
      double beta = NB_Interpolate(latitude,doy,ZB);
      double lam = NB_Interpolate(latitude,doy,ZL);
      double gm = 9.784*(1.0-2.66e-3*std::cos(2.0*latitude*DEG_TO_RAD)-2.8e-7*height);

         // scale factors for height above mean sea level
         // if weather is given, assume it's measured at ht -> kw=kd=1
      double kw=1, base=std::log(1.0-beta*height/temp);
      if(interpolateWeather)
         kw = std::exp(base*(-1.0+(lam+1)*NBg/(NBRd*beta)));

         // compute the zenith delay for wet component
      return ((1.0e-6*NBk3p*NBRd/(gm*(lam+1)-beta*NBRd)) * kw * humid/temp);

   }  // end NBTropModel::wet_zenith_delay()

      // Compute and return the mapping function for dry component
      // of the troposphere
      // @param elevation Elevation of satellite as seen at receiver,
      //                  in degrees
   double NBTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: weather"));
         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }
      if(elevation < 0.0) return 0.0;

      double a,b,c,se,map;
      se = std::sin(elevation*DEG_TO_RAD);

      a = NB_Interpolate(latitude,doy,Mad);
      b = NB_Interpolate(latitude,doy,Mbd);
      c = NB_Interpolate(latitude,doy,Mcd);
      map = (1.0+a/(1.0+b/(1.0+c))) / (se+a/(se+b/(se+c)));

      a = 2.53e-5;
      b = 5.49e-3;
      c = 1.14e-3;
      if(ABS(elevation)<=0.001) se=0.001;
      map += ((1.0/se)-(1.0+a/(1.0+b/(1.0+c)))/(se+a/(se+b/(se+c))))*height/1000.0;

      return map;

   }  // end NBTropModel::dry_mapping_function()

      // Compute and return the mapping function for wet component
      // of the troposphere
      // @param elevation Elevation of satellite as seen at receiver,
      //                  in degrees
   double NBTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: weather"));
         if(!validRxLatitude)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Latitude"));
         if(!validRxHeight)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: Rx Height"));
         if(!validDOY)
            GPSTK_THROW(InvalidTropModel("Invalid NB trop model: day of year"));
      }
      if(elevation < 0.0) return 0.0;

      double a,b,c,se;
      se = std::sin(elevation*DEG_TO_RAD);
      a = NB_Interpolate(latitude,doy,Maw);
      b = NB_Interpolate(latitude,doy,Mbw);
      c = NB_Interpolate(latitude,doy,Mcw);

      return ( (1.0+a/(1.0+b/(1.0+c))) / (se+a/(se+b/(se+c))) );

   }  // end NBTropModel::wet_mapping_function()

      // Re-define the weather data.
      // If called, typically called before any calls to correction().
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   void NBTropModel::setWeather(const double& T,
                                const double& P,
                                const double& H)
      throw(InvalidParameter)
   {
      interpolateWeather=false;
      TropModel::setWeather(T,P,H);
            // humid actually stores water vapor partial pressure
      double th=300./temp;
      humid = 2.409e9*H*th*th*th*th*std::exp(-22.64*th);
      validWeather = true;
      valid = validWeather && validRxHeight && validRxLatitude && validDOY;

   }  // end NBTropModel::setWeather()

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void NBTropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      interpolateWeather = false;
      try
      {
         TropModel::setWeather(wx);
            // humid actually stores vapor partial pressure
         double th=300./temp;
         humid = 2.409e9*humid*th*th*th*th*std::exp(-22.64*th);
         validWeather = true;
         valid = validWeather && validRxHeight && validRxLatitude && validDOY;
      }
      catch(InvalidParameter& e)
      {
         valid = validWeather = false;
         GPSTK_RETHROW(e);
      }
   }

      // configure the model to estimate the weather from the internal model,
      // using lat and doy
   void NBTropModel::setWeather()
      throw(TropModel::InvalidTropModel)
   {
      interpolateWeather = true;
      if(!validRxLatitude)
      {
         valid = validWeather = false;
         GPSTK_THROW(InvalidTropModel(
            "NBTropModel must have Rx latitude before interpolating weather"));
      }
      if(!validDOY)
      {
         valid = validWeather = false;
         GPSTK_THROW(InvalidTropModel(
            "NBTropModel must have day of year before interpolating weather"));
      }
      temp = NB_Interpolate(latitude,doy,ZT);
      press = NB_Interpolate(latitude,doy,ZP);
      humid = NB_Interpolate(latitude,doy,ZW);
      validWeather = true;
      valid = validWeather && validRxHeight && validRxLatitude && validDOY;
   }

      // Define the receiver height; this required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void NBTropModel::setReceiverHeight(const double& ht)
   {
      height = ht;
      validRxHeight = true;
      valid = validWeather && validRxHeight && validRxLatitude && validDOY;
      if(!validWeather && validRxLatitude && validDOY)
         setWeather();
   }  // end NBTropModel::setReceiverHeight()

      // Define the latitude of the receiver; this is required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void NBTropModel::setReceiverLatitude(const double& lat)
   {
      latitude = lat;
      validRxLatitude = true;
      valid = validWeather && validRxHeight && validRxLatitude && validDOY;
      if(!validWeather && validRxLatitude && validDOY)
         setWeather();
   }  // end NBTropModel::setReceiverLatitude(lat)

      // Define the day of year; this is required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void NBTropModel::setDayOfYear(const int& d)
   {
      doy = d;
      if(doy > 0 && doy < 367) validDOY=true; else validDOY = false;
      valid = validWeather && validRxHeight && validRxLatitude && validDOY;
      if(!validWeather && validRxLatitude && validDOY)
         setWeather();
   }  // end NBTropModel::setDayOfYear(doy)

   // ------------------------------------------------------------------------
   // Saastamoinen tropospheric model.
   // This model needs work; it is not the Saastamoinen model, but appears to be
   // a combination of the Neill mapping functions and an unknown delay model.
   // Based on Saastamoinen, J., 'Atmospheric
   // Correction for the Troposphere and Stratosphere in Radio Ranging of
   // Satellites,' Geophysical Monograph 15, American Geophysical Union, 1972,
   // and Ch. 9 of McCarthy, D. and Petit, G., IERS Conventions (2003), IERS
   // Technical Note 32, IERS, 2004. The mapping functions are from
   // Neill, A.E., 1996, 'Global Mapping Functions for the Atmosphere Delay of
   // Radio Wavelengths,' J. Geophys. Res., 101, pp. 3227-3246 (also see IERS TN 32).
   //
   // This model includes a wet and dry component, and requires input of the
   // geodetic latitude, day of year and height above the ellipsoid of the receiver.
   //
   // Usually, the caller will set the latitude and day of year at the same
   // time the weather is set
   //   SaasTropModel stm;
   //   stm.setReceiverLatitude(lat);
   //   stm.setDayOfYear(doy);
   //   stm.setWeather(T,P,H);
   // Then, when the correction (and/or delay and map) is computed, receiver height
   // should be set before the call to correction(elevation):
   //   stm.setReceiverHeight(height);
   //   trop_corr = stm.correction(elevation);
   //
   // NB in this model, units of 'temp' are degrees Celcius and
   // humid actually stores water vapor partial pressure in mbars
   //

   // constants for wet mapping function
   static const double SaasWetA[5]=
     { 0.00058021897, 0.00056794847, 0.00058118019, 0.00059727542, 0.00061641693 };
   static const double SaasWetB[5]=
     { 0.0014275268, 0.0015138625, 0.0014572752, 0.0015007428, 0.0017599082 };
   static const double SaasWetC[5]=
     { 0.043472961, 0.046729510, 0.043908931, 0.044626982, 0.054736038 };

   // constants for dry mapping function
   static const double SaasDryA[5]=
     { 0.0012769934, 0.0012683230, 0.0012465397, 0.0012196049, 0.0012045996 };
   static const double SaasDryB[5]=
     { 0.0029153695, 0.0029152299, 0.0029288445, 0.0029022565, 0.0029024912 };
   static const double SaasDryC[5]=
     { 0.062610505, 0.062837393, 0.063721774, 0.063824265, 0.064258455 };

   static const double SaasDryA1[5]=
     { 0.0, 0.000012709626, 0.000026523662, 0.000034000452, 0.000041202191 };
   static const double SaasDryB1[5]=
     { 0.0, 0.000021414979, 0.000030160779, 0.000072562722, 0.00011723375 };
   static const double SaasDryC1[5]=
     { 0.0, 0.000090128400, 0.000043497037, 0.00084795348, 0.0017037206 };

      // Default constructor
   SaasTropModel::SaasTropModel(void)
   {
      validWeather = false;
      validRxLatitude = false;
      validDOY = false;
      validRxHeight = false;
   } // end SaasTropModel::SaasTropModel()

      // Create a trop model using the minimum information: latitude and doy.
      // Interpolate the weather unless setWeather (optional) is called.
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
   SaasTropModel::SaasTropModel(const double& lat,
                                const int& day)
   {
      validWeather = false;
      validRxHeight = false;
      SaasTropModel::setReceiverLatitude(lat);
      SaasTropModel::setDayOfYear(day);
   } // end SaasTropModel::SaasTropModel

      // Create a trop model with weather.
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
      // @param wx the weather to use for this correction.
   SaasTropModel::SaasTropModel(const double& lat,
                                const int& day,
                                const WxObservation& wx)
      throw(InvalidParameter)
   {
      validRxHeight = false;
      SaasTropModel::setReceiverLatitude(lat);
      SaasTropModel::setDayOfYear(day);
      SaasTropModel::setWeather(wx);
   }  // end SaasTropModel::SaasTropModel(weather)

      // Create a tropospheric model from explicit weather data
      // @param lat Latitude of the receiver in degrees.
      // @param day Day of year.
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   SaasTropModel::SaasTropModel(const double& lat,
                                const int& day,
                                const double& T,
                                const double& P,
                                const double& H)
      throw(InvalidParameter)
   {
      validRxHeight = false;
      SaasTropModel::setReceiverLatitude(lat);
      SaasTropModel::setDayOfYear(day);
      SaasTropModel::setWeather(T,P,H);
   } // end SaasTropModel::SaasTropModel()

      // re-define this to get the throws correct
   double SaasTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         GPSTK_THROW(
            InvalidTropModel("Valid flag corrupted in Saastamoinen trop model"));
      }

      if(elevation < 0.0) return 0.0;

      double corr=0.0;
      try {
         corr = (dry_zenith_delay() * dry_mapping_function(elevation)
            + wet_zenith_delay() * wet_mapping_function(elevation));
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }

      return corr;

   }  // end SaasTropModel::correction(elevation)

      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position and timetag
      // may vary; it computes the elevation (and other receiver location
      // information) and passes them to appropriate set...() routines
      // and the correction(elevation) routine.
      // @param RX  Receiver position
      // @param SV  Satellite position
      // @param tt  Time tag of the signal
   double SaasTropModel::correction(const Position& RX,
                                    const Position& SV,
                                    const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      SaasTropModel::setReceiverHeight(RX.getHeight());
      SaasTropModel::setReceiverLatitude(RX.getGeodeticLatitude());
      SaasTropModel::setDayOfYear(int((static_cast<YDSTime>(tt).doy)));

      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         valid = true;
      }

      double corr=0.0;
      try {
         corr = SaasTropModel::correction(RX.elevation(SV));
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }

      return corr;

   }  // end SaasTropModel::correction(RX,SV,TT)

   double SaasTropModel::correction(const Xvt& RX,
                                    const Xvt& SV,
                                    const CommonTime& tt)
      throw(TropModel::InvalidTropModel)
   {
      Position R(RX),S(SV);
      return SaasTropModel::correction(R,S,tt);
   }

      // Compute and return the zenith delay for dry component of the troposphere
   double SaasTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         GPSTK_THROW(
            InvalidTropModel("Valid flag corrupted in Saastamoinen trop model"));
      }

      // correct pressure for height
      //double press_at_h =
      //   press * std::pow((temp+273.16-4.5*height/1000.0)/(temp+273.16),34.1/4.5);
      // humid is zero for the dry component
      double delay = 0.0022768 * press //_at_h
            / (1 - 0.00266 * ::cos(2*latitude*DEG_TO_RAD) - 0.00028 * height/1000.);

      return delay;

   }  // end SaasTropModel::dry_zenith_delay()

      // Compute and return the zenith delay for wet component of the troposphere
   double SaasTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         GPSTK_THROW(
            InvalidTropModel("Valid flag corrupted in Saastamoinen trop model"));
      }

      // press is zero for the wet component
      double delay = 0.0022768 * humid * 1255/((temp+CELSIUS_TO_KELVIN) + 0.05)
            / (1 - 0.00266 * ::cos(2*latitude*DEG_TO_RAD) - 0.00028 * height/1000.);

      return delay;

   }  // end SaasTropModel::wet_zenith_delay()

      // Compute and return the mapping function for dry component of the troposphere
      // @param elevation Elevation of satellite as seen at receiver, in degrees
   double SaasTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         GPSTK_THROW(
            InvalidTropModel("Valid flag corrupted in Saastamoinen trop model"));
      }
      if(elevation < 0.0) return 0.0;

      double lat,t,ct;
      lat = fabs(latitude);         // degrees
      t = doy - 28.;                // mid-winter
      if(latitude < 0)              // southern hemisphere
         t += 365.25/2.;
      t *= 360.0/365.25;            // convert to degrees
      ct = ::cos(t*DEG_TO_RAD);

      double a,b,c;
      if(lat < 15.) {
         a = SaasDryA[0];
         b = SaasDryB[0];
         c = SaasDryC[0];
      }
      else if(lat < 75.) {          // coefficients are for 15,30,45,60,75 deg
         int i=int(lat/15.0)-1;
         double frac=(lat-15.*(i+1))/15.;
         a = SaasDryA[i] + frac*(SaasDryA[i+1]-SaasDryA[i]);
         b = SaasDryB[i] + frac*(SaasDryB[i+1]-SaasDryB[i]);
         c = SaasDryC[i] + frac*(SaasDryC[i+1]-SaasDryC[i]);

         a -= ct * (SaasDryA1[i] + frac*(SaasDryA1[i+1]-SaasDryA1[i]));
         b -= ct * (SaasDryB1[i] + frac*(SaasDryB1[i+1]-SaasDryB1[i]));
         c -= ct * (SaasDryC1[i] + frac*(SaasDryC1[i+1]-SaasDryC1[i]));
      }
      else {
         a = SaasDryA[4] - ct * SaasDryA1[4];
         b = SaasDryB[4] - ct * SaasDryB1[4];
         c = SaasDryC[4] - ct * SaasDryC1[4];
      }

      double se = ::sin(elevation*DEG_TO_RAD);
      double map = (1.+a/(1.+b/(1.+c)))/(se+a/(se+b/(se+c)));

      a = 0.0000253;
      b = 0.00549;
      c = 0.00114;
      map += (height/1000.0)*(1./se-(1+a/(1.+b/(1.+c)))/(se+a/(se+b/(se+c))));

      return map;

   }  // end SaasTropModel::dry_mapping_function()

      // Compute and return the mapping function for wet component of the troposphere
      // @param elevation Elevation of satellite as seen at receiver, in degrees.
   double SaasTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) {
         if(!validWeather) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: weather"));
         if(!validRxLatitude) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Latitude"));
         if(!validRxHeight) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: Rx Height"));
         if(!validDOY) GPSTK_THROW(
            InvalidTropModel("Invalid Saastamoinen trop model: day of year"));
         GPSTK_THROW(
            InvalidTropModel("Valid flag corrupted in Saastamoinen trop model"));
      }
      if(elevation < 0.0) return 0.0;

      double a,b,c,lat;
      lat = fabs(latitude);         // degrees
      if(lat < 15.) {
         a = SaasWetA[0];
         b = SaasWetB[0];
         c = SaasWetC[0];
      }
      else if(lat < 75.) {          // coefficients are for 15,30,45,60,75 deg
         int i=int(lat/15.0)-1;
         double frac=(lat-15.*(i+1))/15.;
         a = SaasWetA[i] + frac*(SaasWetA[i+1]-SaasWetA[i]);
         b = SaasWetB[i] + frac*(SaasWetB[i+1]-SaasWetB[i]);
         c = SaasWetC[i] + frac*(SaasWetC[i+1]-SaasWetC[i]);
      }
      else {
         a = SaasWetA[4];
         b = SaasWetB[4];
         c = SaasWetC[4];
      }

      double se = ::sin(elevation*DEG_TO_RAD);
      double map = (1.+a/(1.+b/(1.+c)))/(se+a/(se+b/(se+c)));

      return map;

   }  // end SaasTropModel::wet_mapping_function()

      // Re-define the weather data.
      // If called, typically called before any calls to correction().
      // @param T temperature in degrees Celsius
      // @param P atmospheric pressure in millibars
      // @param H relative humidity in percent
   void SaasTropModel::setWeather(const double& T,
                                  const double& P,
                                  const double& H)
      throw(InvalidParameter)
   {
      temp = T;
      press = P;
         // humid actually stores water vapor partial pressure
      double exp=7.5*T/(T+237.3);
      humid = 6.11 * (H/100.) * std::pow(10.0,exp);

      validWeather = true;
      valid = (validWeather && validRxHeight && validRxLatitude && validDOY);

   }  // end SaasTropModel::setWeather()

      // Re-define the tropospheric model with explicit weather data.
      // Typically called just before correction().
      // @param wx the weather to use for this correction
   void SaasTropModel::setWeather(const WxObservation& wx)
      throw(InvalidParameter)
   {
      try
      {
         SaasTropModel::setWeather(wx.temperature,wx.pressure,wx.humidity);
      }
      catch(InvalidParameter& e)
      {
         valid = validWeather = false;
         GPSTK_RETHROW(e);
      }
   }

      // Define the receiver height; this required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void SaasTropModel::setReceiverHeight(const double& ht)
   {
      height = ht;
      validRxHeight = true;
      valid = (validWeather && validRxHeight && validRxLatitude && validDOY);
   }  // end SaasTropModel::setReceiverHeight()

      // Define the latitude of the receiver; this is required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void SaasTropModel::setReceiverLatitude(const double& lat)
   {
      latitude = lat;
      validRxLatitude = true;
      valid = (validWeather && validRxHeight && validRxLatitude && validDOY);
   }  // end SaasTropModel::setReceiverLatitude(lat)

      // Define the day of year; this is required before calling
      // correction() or any of the zenith_delay or mapping_function routines.
   void SaasTropModel::setDayOfYear(const int& d)
   {
      doy = d;
      if(doy > 0 && doy < 367) validDOY=true; else validDOY = false;
      valid = (validWeather && validRxHeight && validRxLatitude && validDOY);
   }  // end SaasTropModel::setDayOfYear(doy)


   //-----------------------------------------------------------------------
      /* Tropospheric model implemented in "GPS Code Analysis Tool" (GCAT)
       * software.
       *
       * This model is described in the book "GPS Data processing: code and
       * phase Algorithms, Techniques and Recipes" by Hernandez-Pajares, M.,
       * J.M. Juan-Zornoza and Sanz-Subirana, J. See Chapter 5.
       *
       * This book and associated software are freely available at:
       *
       * http://gage152.upc.es/~manuel/tdgps/tdgps.html
       *
       * This is a simple but efective model composed of the wet and dry
       * vertical tropospheric delays as defined in Gipsy/Oasis-II GPS
       * analysis software, and the mapping function as defined by Black and
       * Eisner (H. D. Black, A. Eisner. Correcting Satellite Doppler
       * Data for Tropospheric Effects. Journal of  Geophysical Research.
       * Vol 89. 1984.) and used in MOPS (RTCA/DO-229C) standards.
       *
       * Usually, the caller will set the receiver height using
       * setReceiverHeight() method, and then call the correction() method
       * with the satellite elevation as parameter.
       *
       * @code
       *   GCATTropModel gcatTM();
       *   ...
       *   gcatTM.setReceiverHeight(150.0);
       *   trop = gcatTM.correction(elevation);
       * @endcode
       *
       * Another posibility is to set the receiver height when calling
       * the constructor.
       *
       * @code
       *   GCATTropModel gcatTM(150.0);    // Receiver height is 150.0 meters
       *   ...
       *   trop = gcatTM.correction(elevation);
       * @endcode
       */

      // Constructor to create a GCAT trop model providing  the height of the
      // receiver above mean sea level (as defined by ellipsoid model).
      //
      // @param ht Height of the receiver above mean sea level, in meters.
   GCATTropModel::GCATTropModel(const double& ht)
   {
      setReceiverHeight(ht);
      valid = true;
   }


      /* Compute and return the full tropospheric delay. The receiver height
       * must has been provided before, whether using the appropriate
       * constructor or with the setReceiverHeight() method.
       *
       * @param elevation  Elevation of satellite as seen at receiver, in
       *                   degrees
       */
   double GCATTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) throw InvalidTropModel("Invalid model");

      if(elevation < 5.0) return 0.0;

      return ( (dry_zenith_delay() + wet_zenith_delay()) *
               mapping_function(elevation));
   }  // end GCATTropModel::correction(elevation)


      /* Compute and return the full tropospheric delay, given the positions of
       * receiver and satellite. This version is most useful within positioning
       * algorithms, where the receiver position may vary; it computes the
       * elevation and the receiver height and passes them to appropriate
       * set...() routines and the correction(elevation) routine.
       *
       * @param RX  Receiver position in ECEF cartesian coordinates (meters)
       * @param SV  Satellite position in ECEF cartesian coordinates (meters)
       */
   double GCATTropModel::correction( const Position& RX,
                                     const Position& SV )
      throw(TropModel::InvalidTropModel)
   {

      try
      {
         setReceiverHeight( RX.getAltitude() );
      }
      catch(GeometryException& e)
      {
         valid = false;
      }

      if(!valid) throw InvalidTropModel("Invalid model");

      double c;
      try
      {
         c = correction(RX.elevationGeodetic(SV));
      }
      catch(InvalidTropModel& e)
      {
         GPSTK_RETHROW(e);
      }

      return c;

   }  // end GCATTropModel::correction(RX,SV,TT)


      /* Compute and return the full tropospheric delay, given the positions of
       * receiver and satellite and the time tag. This version is most useful
       * within positioning algorithms, where the receiver position and timetag
       * may vary; it computes the elevation (and other receiver location
       * information) and passes them to appropriate set...() routines and the
       * correction(elevation) routine.
       *
       * @param RX  Receiver position in ECEF cartesian coordinates (meters)
       * @param SV  Satellite position in ECEF cartesian coordinates (meters)
       * @param tt  Time. In this model, tt is a dummy parameter kept only for
       *            consistency
       *
       * This function is deprecated; use the Position version
       */
   double GCATTropModel::correction( const Xvt& RX,
                                     const Xvt& SV,
                                     const CommonTime& tt )
      throw(TropModel::InvalidTropModel)
   {

      Position R(RX),S(SV);

      return GCATTropModel::correction(R,S);
   }  // end GCATTropModel::correction(RX,SV,tt)


      /* Compute and return the zenith delay for the dry component of the
       * troposphere.
       */
   double GCATTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) throw InvalidTropModel("Invalid model");

      double ddry(2.29951*std::exp((-0.000116 * gcatHeight) ));

      return ddry;
   }  // end GCATTropModel::dry_zenith_delay()


      /* Compute and return the mapping function of the troposphere
       * @param elevation  Elevation of satellite as seen at receiver,
       *                   in degrees
       */
   double GCATTropModel::mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid) throw InvalidTropModel("Invalid model");

      if(elevation < 5.0) return 0.0;

      double d = std::sin(elevation*DEG_TO_RAD);
      d = SQRT(0.002001+(d*d));

      return (1.001/d);
   }  // end GCATTropModel::mapping_function(elevation)


      /* Define the receiver height; this is required before calling
       * correction() or any of the zenith_delay or mapping_function routines.
       * @param ht Height of the receiver above mean sea level, in meters.
       */
   void GCATTropModel::setReceiverHeight(const double& ht)
   {
      gcatHeight = ht;
      valid = true;
   }


   //---------------------------------------------------------------
      /* Tropospheric model implemented in the RTCA "Minimum Operational
       * Performance Standards" (MOPS), version C.
       *
       * This model is described in the RTCA "Minimum Operational Performance
       * Standards" (MOPS), version C (RTCA/DO-229C), in Appendix A.4.2.4.
       * Although originally developed for SBAS systems (EGNOS, WAAS), it may
       * be suitable for other uses as well.
       *
       * This model needs the day of year, satellite elevation (degrees),
       * receiver height over mean sea level (meters) and receiver latitude in
       * order to start computing.
       *
       * On the other hand, the outputs are the tropospheric correction (in
       * meters) and the sigma-squared of tropospheric delay residual error
       * (meters^2).
       *
       * A typical way to use this model follows:
       *
       * @code
       *   MOPSTropModel mopsTM;
       *   mopsTM.setReceiverLatitude(lat);
       *   mopsTM.setReceiverHeight(height);
       *   mopsTM.setDayOfYear(doy);
       * @endcode
       *
       * Once all the basic model parameters are set (latitude, height and day
       * of year), then we are able to compute the tropospheric correction as
       * a function of elevation:
       *
       * @code
       *   trop = mopsTM.correction(elevation);
       * @endcode
       *
       */

      // Some specific constants
   static const double MOPSg=9.80665;
   static const double MOPSgm=9.784;
   static const double MOPSk1=77.604;
   static const double MOPSk2=382000.0;
   static const double MOPSRd=287.054;


      // Empty constructor
   MOPSTropModel::MOPSTropModel(void)
   {
      validHeight = false;
      validLat    = false;
      validTime   = false;
      valid       = false;
   }


      /* Constructor to create a MOPS trop model providing the height of
       *  the receiver above mean sea level (as defined by ellipsoid model),
       *  its latitude and the day of year.
       *
       * @param ht   Height of the receiver above mean sea level, in meters.
       * @param lat  Latitude of receiver, in degrees.
       * @param doy  Day of year.
       */
   MOPSTropModel::MOPSTropModel( const double& ht,
                                 const double& lat,
                                 const int& doy )
   {
      setReceiverHeight(ht);
      setReceiverLatitude(lat);
      setDayOfYear(doy);
   }


      /* Constructor to create a MOPS trop model providing the position of
       *  the receiver and current time.
       *
       * @param RX   Receiver position.
       * @param time Time.
       */
   MOPSTropModel::MOPSTropModel(const Position& RX, const CommonTime& time)
   {
      setReceiverHeight(RX.getAltitude());
      setReceiverLatitude(RX.getGeodeticLatitude());
      setDayOfYear(time);
   }


      // Compute and return the full tropospheric delay. The receiver height,
      // latitude and Day oy Year must has been set before using the
      // appropriate constructor or the provided methods.
      // @param elevation Elevation of satellite as seen at receiver, in
      // degrees
   double MOPSTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
      {
         if(!validLat)
            throw InvalidTropModel("Invalid MOPS trop model: Rx Latitude");
         if(!validHeight)
            throw InvalidTropModel("Invalid MOPS trop model: Rx Height");
         if(!validTime)
            throw InvalidTropModel("Invalid MOPS trop model: day of year");
      }

      if(elevation < 5.0) return 0.0;

      double map = MOPSTropModel::mapping_function(elevation);

         // Compute tropospheric delay
      double tropDelay = ( MOPSTropModel::dry_zenith_delay() +
                           MOPSTropModel::wet_zenith_delay() ) * map;

      return tropDelay;

   }  // end MOPSTropModel::correction(elevation)


      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite. This version is most useful within
      // positioning algorithms, where the receiver position may vary; it
      // computes the elevation (and other receiver location information as
      // height and latitude) and passes them to appropriate methods. You must
      // set time using method setDayOfYear() before calling this method.
      // @param RX  Receiver position
      // @param SV  Satellite position
   double MOPSTropModel::correction( const Position& RX,
                                     const Position& SV )
      throw(TropModel::InvalidTropModel)
   {
      try
      {
         setReceiverHeight( RX.getAltitude() );
         setReceiverLatitude(RX.getGeodeticLatitude());
         setWeather();
      }
      catch(GeometryException& e)
      {
         valid = false;
      }

      if(!valid) throw InvalidTropModel("Invalid model");

      double c;
      try
      {
         c = MOPSTropModel::correction(RX.elevationGeodetic(SV));
      }
      catch(InvalidTropModel& e)
      {
         GPSTK_RETHROW(e);
      }

      return c;

   }  // end MOPSTropModel::correction(RX,SV)


      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position may vary;
      // it computes the elevation (and other receiver location information as
      // height  and latitude) and passes them to appropriate methods.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time (CommonTime object).
   double MOPSTropModel::correction( const Position& RX,
                                     const Position& SV,
                                     const CommonTime& tt )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(tt);

      return MOPSTropModel::correction(RX,SV);
   }  // end MOPSTropModel::correction(RX,SV,TT)


      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the day of the year. This version is most
      // useful within positioning algorithms, where the receiver position may
      // vary; it computes the elevation (and other receiver location
      // information as height and latitude) and passes them to appropriate
      // methods.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param doy Day of year.
   double MOPSTropModel::correction( const Position& RX,
                                     const Position& SV,
                                     const int& doy )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(doy);

      return MOPSTropModel::correction(RX,SV);
   }  // end MOPSTropModel::correction(RX,SV,doy)



      // deprecated
      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite. You must set time using method setDayOfYear()
      // before calling this method.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // This function is deprecated; use the Position version
   double MOPSTropModel::correction( const Xvt& RX,
                                     const Xvt& SV )
      throw(TropModel::InvalidTropModel)
   {
      Position R(RX),S(SV);

      return MOPSTropModel::correction(R,S);
   }  // end MOPSTropModel::correction(RX,SV)


      // deprecated
      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is most useful
      // within positioning algorithms, where the receiver position may vary;
      // it computes the elevation (and other receiver location information as
      // height and latitude) and passes them to appropriate methods.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time (CommonTime object).
      // This function is deprecated; use the Position version
   double MOPSTropModel::correction( const Xvt& RX,
                                     const Xvt& SV,
                                     const CommonTime& tt )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(tt);
      Position R(RX),S(SV);

      return MOPSTropModel::correction(R,S);
   }  // end MOPSTropModel::correction(RX,SV,tt)


      // deprecated
      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the day of the year. This version is most
      // useful within positioning algorithms, where the receiver position may
      // vary; it computes the elevation (and other receiver location
      // information as height and latitude) and passes them to appropriate
      // methods.
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param doy Day of year.
      // This function is deprecated; use the Position version
   double MOPSTropModel::correction( const Xvt& RX,
                                     const Xvt& SV,
                                     const int& doy )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(doy);
      Position R(RX),S(SV);

      return MOPSTropModel::correction(R,S);
   }  // end MOPSTropModel::correction(RX,SV,doy)


      // Compute and return the zenith delay for the dry component of the
      // troposphere
   double MOPSTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {

      if(!valid) throw InvalidTropModel("Invalid model");

      double ddry, zh_dry, exponent;

         // Set the extra parameters
      double P = MOPSParameters(0);
      double T = MOPSParameters(1);
      double beta = MOPSParameters(3);

         // Zero-altitude dry zenith delay:
      zh_dry = 0.000001*(MOPSk1*MOPSRd)*P/MOPSgm;

         // Zenith delay terms at MOPSHeight meters of height above mean sea
         // level
      exponent = MOPSg/MOPSRd/beta;
      ddry = zh_dry * std::pow( (1.0 - beta*MOPSHeight/T), exponent );

      return ddry;

   }  // end MOPSTropModel::dry_zenith_delay()


      // Compute and return the zenith delay for the wet component of the
      // troposphere
   double MOPSTropModel::wet_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {

      if(!valid) throw InvalidTropModel("Invalid model");

         double dwet, zh_wet, exponent;

         // Set the extra parameters
      double T = MOPSParameters(1);
      double e = MOPSParameters(2);
      double beta = MOPSParameters(3);
      double lambda = MOPSParameters(4);

         // Zero-altitude wet zenith delay:
      zh_wet = (0.000001*MOPSk2)*MOPSRd/(MOPSgm*(lambda+1.0)-beta*MOPSRd)*e/T;

         // Zenith delay terms at MOPSHeight meters of height above mean sea
         // level
      exponent = ( (lambda+1.0)*MOPSg/MOPSRd/beta)-1.0;
      dwet= zh_wet * std::pow( (1.0 - beta*MOPSHeight/T), exponent );

      return dwet;

   }  // end MOPSTropModel::wet_zenith_delay()


      // This method configure the model to estimate the weather using height,
      // latitude and day of year (DOY). It is called automatically when
      // setting those parameters.
   void MOPSTropModel::setWeather()
      throw(TropModel::InvalidTropModel)
   {

      if(!validLat)
      {
         valid = false;
         throw InvalidTropModel(
            "MOPSTropModel must have Rx latitude before computing weather");
      }

      if(!validTime)
      {
         valid = false;
         throw InvalidTropModel(
            "MOPSTropModel must have day of year before computing weather");
      }

         // In order to compute tropospheric delay we need to compute some
         // extra parameters
      try
      {
         prepareParameters();
      }
      catch(InvalidTropModel& e)
      {
         GPSTK_RETHROW(e);
      }

      valid = validHeight && validLat && validTime;
   }


      /* Define the receiver height; this is required before calling
       *  correction() or any of the zenith_delay routines.
       *
       * @param ht   Height of the receiver above mean sea level, in meters.
       */
   void MOPSTropModel::setReceiverHeight(const double& ht)
   {
      MOPSHeight = ht;
      validHeight = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validTime;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      /* Define the receiver latitude; this is required before calling
       *  correction() or any of the zenith_delay routines.
       *
       * @param lat  Latitude of receiver, in degrees.
       */
   void MOPSTropModel::setReceiverLatitude(const double& lat)
   {
      MOPSLat = lat;
      validLat = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validTime;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      /* Set the time when tropospheric correction will be computed for, in
       *  days of the year.
       *
       * @param doy  Day of the year.
       */
   void MOPSTropModel::setDayOfYear(const int& doy)
   {

      if ( (doy>=1) && (doy<=366))
      {
         validTime = true;
      }
      else
      {
         validTime = false;
      }

      MOPSTime = doy;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validTime;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();
   }


      /* Set the time when tropospheric correction will be computed for, in
       *  days of the year.
       *
       * @param time  Time object.
       */
   void MOPSTropModel::setDayOfYear(const CommonTime& time)
   {
      MOPSTime = (int)(static_cast<YDSTime>(time)).doy;
      validTime = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validTime;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      /* Convenient method to set all model parameters in one pass.
       *
       * @param time  Time object.
       * @param rxPos Receiver position object.
       */
   void MOPSTropModel::setAllParameters( const CommonTime& time,
                                         const Position& rxPos )
   {

      MOPSTime = (int)(static_cast<YDSTime>(time)).doy;
      validTime = true;
      MOPSLat = rxPos.getGeodeticLatitude();
      validHeight = true;
      MOPSLat = rxPos.getHeight();
      validLat = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validTime;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      // Compute and return the sigma-squared value of tropospheric delay
      // residual error (meters^2)
      // @param elevation  Elevation of satellite as seen at receiver,
      //                   in degrees
   double MOPSTropModel::MOPSsigma2(double elevation)
      throw(TropModel::InvalidTropModel)
   {

      double map_f;

         // If elevation is below bounds, fail in a sensible way returning a
         // very big sigma value
      if(elevation < 5.0)
      {
         return 9.9e9;
      }
      else
      {
         map_f = MOPSTropModel::mapping_function(elevation);
      }

         // Compute residual error for tropospheric delay
      double MOPSsigma2trop = (0.12*map_f)*(0.12*map_f);

      return MOPSsigma2trop;

   }  // end MOPSTropModel::MOPSsigma(elevation)


      // The MOPS tropospheric model needs to compute several extra parameters
   void MOPSTropModel::prepareParameters(void)
      throw(TropModel::InvalidTropModel)
   {

      if(!valid) throw InvalidTropModel("Invalid model");

      try
      {
            // We need to read some data
         prepareTables();

            // Declare some variables
         int idmin, j, index;
         double fact, axfi;
         Vector<double> avr0(5);
         Vector<double> svr0(5);

            // Resize MOPSParameters as appropriate
         MOPSParameters.resize(5);

         if (MOPSLat >= 0.0)
         {
            idmin = 28;
         }
         else
         {
            idmin = 211;
         }

            // Fraction of the year in radians
         fact = 2.0*PI*((double)(MOPSTime-idmin))/365.25;

         axfi = ABS(MOPSLat);

         if ( axfi <= 15.0 )                    index=0;
         if ( (axfi > 15.0) && (axfi <= 30.0) ) index=1;
         if ( (axfi > 30.0) && (axfi <= 45.0) ) index=2;
         if ( (axfi > 45.0) && (axfi <= 60.0) ) index=3;
         if ( (axfi > 60.0) && (axfi <  75.0) ) index=4;
         if ( axfi >= 75.0 )                     index=5;

         for (j=0; j<5; j++)
         {
            if (index == 0) {
               avr0(j)=avr(index,j);
               svr0(j)=svr(index,j);
            }
            else
            {
               if (index < 5)
               {
                  avr0(j) = avr(index-1,j) + (avr(index,j)-avr(index-1,j)) *
                            (axfi-fi0(index-1))/(fi0( index)-fi0(index-1));

                  svr0(j) = svr(index-1,j) + (svr(index,j)-svr(index-1,j)) *
                            (axfi-fi0(index-1))/(fi0( index)-fi0(index-1));
               }
               else
               {
                  avr0(j) = avr(index-1,j);
                  svr0(j) = svr(index-1,j);
               }
            }

            MOPSParameters(j) = avr0(j)-svr0(j)*std::cos(fact);
         }

      } // end try
      catch (...)
      {
         InvalidTropModel e("Problem computing extra MOPS parameters.");
         GPSTK_RETHROW(e);
      }

   }  // end MOPSTropModel::prepareParameters()


      // The MOPS tropospheric model uses several predefined data tables
   void MOPSTropModel::prepareTables(void)
   {
      avr.resize(5,5);
      svr.resize(5,5);
      fi0.resize(5);


         // Table avr (Average):

      avr(0,0) = 1013.25; avr(0,1) = 299.65; avr(0,2) = 26.31;
         avr(0,3) = 0.0063; avr(0,4) = 2.77;

      avr(1,0) = 1017.25; avr(1,1) = 294.15; avr(1,2) = 21.79;
         avr(1,3) = 0.00605; avr(1,4) = 3.15;

      avr(2,0) = 1015.75; avr(2,1) = 283.15; avr(2,2) = 11.66;
         avr(2,3) = 0.00558; avr(2,4) = 2.57;

      avr(3,0) = 1011.75; avr(3,1) = 272.15; avr(3,2) = 6.78;
         avr(3,3) = 0.00539; avr(3,4) = 1.81;

      avr(4,0) = 1013.00; avr(4,1) = 263.65; avr(4,2) = 4.11;
         avr(4,3) = 0.00453; avr(4,4) = 1.55;


         // Table svr (Seasonal Variation):

      svr(0,0) = 0.00; svr(0,1) = 0.00; svr(0,2) = 0.00;
         svr(0,3) = 0.00000; svr(0,4) = 0.00;

      svr(1,0) = -3.75; svr(1,1) = 7.00; svr(1,2) = 8.85;
         svr(1,3) = 0.00025; svr(1,4) = 0.33;

      svr(2,0) = -2.25; svr(2,1) = 11.00; svr(2,2) = 7.24;
         svr(2,3) = 0.00032; svr(2,4) = 0.46;

      svr(3,0) = -1.75; svr(3,1) = 15.00; svr(3,2) = 5.36;
         svr(3,3) = 0.00081; svr(3,4) = 0.74;

      svr(4,0) = -0.50; svr(4,1) = 14.50; svr(4,2) = 3.39;
         svr(4,3) = 0.00062; svr(4,4) = 0.30;


         // Table fi0 (Latitude bands):

      fi0(0) = 15.0; fi0(1) = 30.0; fi0(2) = 45.0;
         fi0(3) = 60.0; fi0(4) = 75.0;

   }


   //---------------------------------------------------------------------
      /* Tropospheric model based in the Neill mapping functions.
       *
       * This model uses the mapping functions developed by A.E. Niell and
       * published in Neill, A.E., 1996, 'Global Mapping Functions for the
       * Atmosphere Delay of Radio Wavelengths,' J. Geophys. Res., 101,
       * pp. 3227-3246 (also see IERS TN 32).
       *
       * The coefficients of the hydrostatic mapping function depend on the
       * latitude and height above sea level of the receiver station, and on
       * the day of the year. On the other hand, the wet mapping function
       * depends only on latitude.
       *
       * This mapping is independent from surface meteorology, while having
       * comparable accuracy and precision to those that require such data.
       * This characteristic makes this model very useful, and it is
       * implemented in geodetic software such as JPL's Gipsy/OASIS.
       *
       * A typical way to use this model follows:
       *
       * @code
       *   NeillTropModel neillTM;
       *   neillTM.setReceiverLatitude(lat);
       *   neillTM.setReceiverHeight(height);
       *   neillTM.setDayOfYear(doy);
       * @endcode
       *
       * Once all the basic model parameters are set (latitude, height and
       * day of year), then we are able to compute the tropospheric correction
       * as a function of elevation:
       *
       * @code
       *   trop = neillTM.correction(elevation);
       * @endcode
       *
       * @warning The Neill mapping functions are defined for elevation
       * angles down to 3 degrees.
       *
       */

      // Constructor to create a Neill trop model providing the position
      // of the receiver and current time.
      //
      // @param RX   Receiver position.
      // @param time Time.
   NeillTropModel::NeillTropModel( const Position& RX,
                                   const CommonTime& time )
   {
      setReceiverHeight(RX.getAltitude());
      setReceiverLatitude(RX.getGeodeticLatitude( ));
      setDayOfYear(time);
   }



      // Parameters borrowed from Saastamoinen tropospheric model
      // Constants for wet mapping function
   static const double NeillWetA[5] =
                              { 0.00058021897, 0.00056794847, 0.00058118019,
                                0.00059727542, 0.00061641693 };
   static const double NeillWetB[5] =
                              { 0.0014275268, 0.0015138625, 0.0014572752,
                                0.0015007428, 0.0017599082 };
   static const double NeillWetC[5] =
                              { 0.043472961, 0.046729510, 0.043908931,
                                0.044626982, 0.054736038 };

      // constants for dry mapping function
   static const double NeillDryA[5] =
                              { 0.0012769934, 0.0012683230, 0.0012465397,
                                0.0012196049, 0.0012045996 };
   static const double NeillDryB[5] =
                              { 0.0029153695, 0.0029152299, 0.0029288445,
                                0.0029022565, 0.0029024912 };
   static const double NeillDryC[5] =
                              { 0.062610505, 0.062837393, 0.063721774,
                                0.063824265, 0.064258455 };

   static const double NeillDryA1[5] =
                              { 0.0, 0.000012709626, 0.000026523662,
                                0.000034000452, 0.000041202191 };
   static const double NeillDryB1[5] =
                              { 0.0, 0.000021414979, 0.000030160779,
                                0.000072562722, 0.00011723375 };
   static const double NeillDryC1[5] =
                              { 0.0, 0.000090128400, 0.000043497037,
                                0.00084795348, 0.0017037206 };


      // Compute and return the full tropospheric delay. The receiver height,
      // latitude and Day oy Year must has been set before using the
      // appropriate constructor or the provided methods.
      //
      // @param elevation Elevation of satellite as seen at receiver,
      // in degrees
   double NeillTropModel::correction(double elevation) const
      throw(TropModel::InvalidTropModel)
   {

      if(!valid)
      {
         if(!validLat)
         {
            throw InvalidTropModel("Invalid Neill trop model: Rx Latitude");
         }

         if(!validHeight)
         {
            throw InvalidTropModel("Invalid Neill trop model: Rx Height");
         }

         if(!validDOY)
         {
            throw InvalidTropModel("Invalid Neill trop model: day of year");
         }
      }

         // Neill mapping functions work down to 3 degrees of elevation
      if(elevation < 3.0)
      {
         return 0.0;
      }

      double map_dry(NeillTropModel::dry_mapping_function(elevation));

      double map_wet(NeillTropModel::wet_mapping_function(elevation));

         // Compute tropospheric delay
      double tropDelay( (NeillTropModel::dry_zenith_delay() * map_dry) +
                        (NeillTropModel::wet_zenith_delay() * map_wet) );

      return tropDelay;

   }  // end NeillTropModel::correction(elevation)


      /* Compute and return the full tropospheric delay, given the
       * positions of receiver and satellite.
       *
       * This version is more useful within positioning algorithms, where
       * the receiver position may vary; it computes the elevation (and
       * other receiver location information as height and latitude) and
       * passes them to appropriate methods.
       *
       * You must set time using method setReceiverDOY() before calling
       * this method.
       *
       * @param RX  Receiver position.
       * @param SV  Satellite position.
       */
   double NeillTropModel::correction( const Position& RX,
                                      const Position& SV )
      throw(TropModel::InvalidTropModel)
   {

      try
      {
         setReceiverHeight( RX.getAltitude() );
         setReceiverLatitude(RX.getGeodeticLatitude());
         setWeather();
      }
      catch(GeometryException& e)
      {
         valid = false;
      }

      if(!valid)
      {
         throw InvalidTropModel("Invalid model");
      }

      double c;
      try
      {
         c = NeillTropModel::correction(RX.elevationGeodetic(SV));
      }
      catch(InvalidTropModel& e)
      {
         GPSTK_RETHROW(e);
      }

      return c;

   }  // end NeillTropModel::correction(RX,SV)


      /* Compute and return the full tropospheric delay, given the
       * positions of receiver and satellite and the time tag.
       *
       * This version is more useful within positioning algorithms, where
       * the receiver position may vary; it computes the elevation (and
       * other receiver location information as height and latitude), and
       * passes them to appropriate methods.
       *
       * @param RX  Receiver position.
       * @param SV  Satellite position.
       * @param tt  Time (CommonTime object).
       */
   double NeillTropModel::correction( const Position& RX,
                                      const Position& SV,
                                      const CommonTime& tt )
      throw(TropModel::InvalidTropModel)
   {

      setDayOfYear(tt);

      return NeillTropModel::correction(RX,SV);

   }  // end NeillTropModel::correction(RX,SV,TT)


      /* Compute and return the full tropospheric delay, given the
       * positions of receiver and satellite and the day of the year.
       *
       * This version is more useful within positioning algorithms, where
       * the receiver position may vary; it computes the elevation (and
       * other receiver location information as height and latitude), and
       * passes them to appropriate methods.
       *
       * @param RX  Receiver position.
       * @param SV  Satellite position.
       * @param doy Day of year.
       */
   double NeillTropModel::correction( const Position& RX,
                                      const Position& SV,
                                      const int& doy )
      throw(TropModel::InvalidTropModel)
   {

      setDayOfYear(doy);

      return NeillTropModel::correction(RX,SV);

   }  // end NeillTropModel::correction(RX,SV,doy)



      // deprecated
      // Compute and return the full tropospheric delay, given the positions
      // of receiver and satellite. . You must set time using method
      // setDayOfYear() before calling this method.
      //
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // This function is deprecated; use the Position version
   double NeillTropModel::correction( const Xvt& RX,
                                      const Xvt& SV )
      throw(TropModel::InvalidTropModel)
   {
      Position R(RX),S(SV);

      return NeillTropModel::correction(R,S);

   }  // end NeillTropModel::correction(RX,SV)


      // deprecated
      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the time tag. This version is more useful
      // within positioning algorithms, where the receiver position may vary;
      // it computes the elevation (and other receiver location information as
      // height and latitude) and passes them to appropriate methods.
      //
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param tt  Time (CommonTime object).
      // This function is deprecated; use the Position version
   double NeillTropModel::correction( const Xvt& RX,
                                      const Xvt& SV,
                                      const CommonTime& tt )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(tt);
      Position R(RX),S(SV);

      return NeillTropModel::correction(R,S);

   }  // end NeillTropModel::correction(RX,SV,tt)


      // deprecated
      // Compute and return the full tropospheric delay, given the positions of
      // receiver and satellite and the day of the year. This version is more
      // within positioning algorithms, where the receiver position may vary;
      // it computes the elevation (and other receiver location information as
      // height and latitude) and passes them to appropriate methods.
      //
      // @param RX  Receiver position in ECEF cartesian coordinates (meters)
      // @param SV  Satellite position in ECEF cartesian coordinates (meters)
      // @param doy Day of year.
      // This function is deprecated; use the Position version
   double NeillTropModel::correction( const Xvt& RX,
                                      const Xvt& SV,
                                      const int& doy )
      throw(TropModel::InvalidTropModel)
   {
      setDayOfYear(doy);
      Position R(RX),S(SV);

      return NeillTropModel::correction(R,S);

   }  // end NeillTropModel::correction(RX,SV,doy)


      // Compute and return the zenith delay for the dry component of
      // the troposphere.
   double NeillTropModel::dry_zenith_delay(void) const
      throw(TropModel::InvalidTropModel)
   {

      if( !valid )
      {
         throw InvalidTropModel("Invalid model");
      }

         // Note: 1.013*2.27 = 2.29951
      double ddry( 2.29951*std::exp( (-0.000116 * NeillHeight) ) );

      return ddry;

   }  // end NeillTropModel::dry_zenith_delay()


      // Compute and return the mapping function for dry component of
      // the troposphere.
      //
      // @param elevation Elevation of satellite as seen at receiver, in
      //                  degrees
   double NeillTropModel::dry_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
      {
         if(!validLat)
         {
            GPSTK_THROW( InvalidTropModel( "Invalid Neill trop model: Rx \
                                            Latitude" ) );
         }

         if(!validHeight)
         {
            GPSTK_THROW( InvalidTropModel( "Invalid Neill trop model: Rx \
                                            Height" ) );
         }

         if(!validDOY)
         {
            GPSTK_THROW( InvalidTropModel( "Invalid Neill trop model: day \
                                            of year" ) );
         }

         GPSTK_THROW(InvalidTropModel( "Valid flag corrupted in Neill trop \
                                        model" ) );
      }

      if(elevation < 3.0)
      {
         return 0.0;
      }

      double lat, t, ct;
      lat = fabs(NeillLat);         // degrees
      t = static_cast<double>(NeillDOY) - 28.0;  // mid-winter

      if(NeillLat < 0.0)              // southern hemisphere
      {
         t += 365.25/2.;
      }

      t *= 360.0/365.25;            // convert to degrees
      ct = ::cos(t*DEG_TO_RAD);

      double a, b, c;
      if(lat < 15.0)
      {
         a = NeillDryA[0];
         b = NeillDryB[0];
         c = NeillDryC[0];
      }
      else if(lat < 75.)      // coefficients are for 15,30,45,60,75 deg
      {
         int i=int(lat/15.0)-1;
         double frac=(lat-15.*(i+1))/15.;
         a = NeillDryA[i] + frac*(NeillDryA[i+1]-NeillDryA[i]);
         b = NeillDryB[i] + frac*(NeillDryB[i+1]-NeillDryB[i]);
         c = NeillDryC[i] + frac*(NeillDryC[i+1]-NeillDryC[i]);

         a -= ct * (NeillDryA1[i] + frac*(NeillDryA1[i+1]-NeillDryA1[i]));
         b -= ct * (NeillDryB1[i] + frac*(NeillDryB1[i+1]-NeillDryB1[i]));
         c -= ct * (NeillDryC1[i] + frac*(NeillDryC1[i+1]-NeillDryC1[i]));
      }
      else
      {
         a = NeillDryA[4] - ct * NeillDryA1[4];
         b = NeillDryB[4] - ct * NeillDryB1[4];
         c = NeillDryC[4] - ct * NeillDryC1[4];
      }

      double se = ::sin(elevation*DEG_TO_RAD);
      double map = (1.+a/(1.+b/(1.+c)))/(se+a/(se+b/(se+c)));

      a = 0.0000253;
      b = 0.00549;
      c = 0.00114;
      map += ( NeillHeight/1000.0 ) *
             ( 1./se - ( (1.+a/(1.+b/(1.+c))) / (se+a/(se+b/(se+c))) ) );

      return map;

   }  // end NeillTropModel::dry_mapping_function()


      // Compute and return the mapping function for wet component of the
      // troposphere.
      //
      // @param elevation Elevation of satellite as seen at receiver,
      //                  in degrees.
   double NeillTropModel::wet_mapping_function(double elevation) const
      throw(TropModel::InvalidTropModel)
   {
      if(!valid)
      {
         if(!validLat)
         {
            GPSTK_THROW( InvalidTropModel( "Invalid Neill trop model: Rx \
                                            Latitude" ) );
         }
         if(!validHeight)
         {
            GPSTK_THROW( InvalidTropModel( "Invalid Neill trop model: Rx \
                                            Height" ) );
         }
         if(!validDOY)
         {
            GPSTK_THROW( InvalidTropModel(" Invalid Neill trop model: day \
                                            of year" ) );
         }

         GPSTK_THROW(InvalidTropModel( "Valid flag corrupted in Neill trop \
                                        model" ) );
      }

      if(elevation < 3.0)
      {
         return 0.0;
      }

      double a,b,c,lat;
      lat = fabs(NeillLat);         // degrees
      if(lat < 15.0)
      {
         a = NeillWetA[0];
         b = NeillWetB[0];
         c = NeillWetC[0];
      }
      else if(lat < 75.)          // coefficients are for 15,30,45,60,75 deg
      {
         int i=int(lat/15.0)-1;
         double frac=(lat-15.*(i+1))/15.;
         a = NeillWetA[i] + frac*(NeillWetA[i+1]-NeillWetA[i]);
         b = NeillWetB[i] + frac*(NeillWetB[i+1]-NeillWetB[i]);
         c = NeillWetC[i] + frac*(NeillWetC[i+1]-NeillWetC[i]);
      }
      else
      {
         a = NeillWetA[4];
         b = NeillWetB[4];
         c = NeillWetC[4];
      }

      double se = ::sin(elevation*DEG_TO_RAD);
      double map = ( 1.+ a/ (1.+ b/(1.+c) ) ) / (se + a/(se + b/(se+c) ) );

      return map;

   }  // end NeillTropModel::wet_mapping_function()


      // This method configure the model to estimate the weather using height,
      // latitude and day of year (DOY). It is called automatically when
      // setting those parameters.
   void NeillTropModel::setWeather()
      throw(TropModel::InvalidTropModel)
   {

      if(!validLat)
      {
         valid = false;
         throw InvalidTropModel( "NeillTropModel must have Rx latitude \
                                  before computing weather ");
      }
      if(!validDOY)
      {
         valid = false;
         throw InvalidTropModel( "NeillTropModel must have day of year \
                                  before computing weather" );
      }

      valid = validHeight && validLat && validDOY;

   }


      // Define the receiver height; this is required before calling
      // correction() or any of the zenith_delay routines.
      //
      // @param ht   Height of the receiver above mean sea level,
      //             in meters.
   void NeillTropModel::setReceiverHeight(const double& ht)
   {
      NeillHeight = ht;
      validHeight = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validDOY;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      // Define the receiver latitude; this is required before calling
      // correction() or any of the zenith_delay routines.
      //
      // @param lat  Latitude of receiver, in degrees.
   void NeillTropModel::setReceiverLatitude(const double& lat)
   {
      NeillLat = lat;
      validLat = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validDOY;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      // Set the time when tropospheric correction will be computed for,
      // in days of the year.
      //
      // @param doy  Day of the year.
   void NeillTropModel::setDayOfYear(const int& doy)
   {

      if( (doy>=1) && (doy<=366) )
      {
         validDOY = true;
      }
      else
      {
         validDOY = false;
      }

      NeillDOY = doy;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validDOY;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      // Set the time when tropospheric correction will be computed for,
      // in days of the year.
      //
      // @param time  Time object.
   void NeillTropModel::setDayOfYear(const CommonTime& time)
   {

      NeillDOY = static_cast<int>((static_cast<YDSTime>(time)).doy);
      validDOY = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validDOY;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


      /* Convenient method to set all model parameters in one pass.
       *
       * @param time  Time object.
       * @param rxPos Receiver position object.
       */
   void NeillTropModel::setAllParameters( const CommonTime& time,
                                          const Position& rxPos )
   {
   	YDSTime ydst = static_cast<YDSTime>(time);
      NeillDOY = static_cast<int>(ydst.doy);
      validDOY = true;
      NeillLat = rxPos.getGeodeticLatitude();
      validHeight = true;
      NeillLat = rxPos.getHeight();
      validLat = true;

         // Change the value of field "valid" if everything is already set
      valid = validHeight && validLat && validDOY;

         // If model is valid, set the appropriate parameters
      if (valid) setWeather();

   }


} // end namespace gpstk
