#pragma ident "$Id$"

/**
 * @file MoonPosition.cpp
 * Returns the approximate position of the Moon at the given epoch in the 
 * ECEF system.
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
//  Dagoberto Salazar - gAGE ( http://www.gage.es ). 2007
//
//============================================================================


#include "MoonPosition.hpp"
#include "MJD.hpp"
#include "CivilTime.hpp"
#include "EpochDataStore.hpp"


namespace gpstk
{


      // Time of the first valid time
   const CommonTime MoonPosition::initialTime = CivilTime(1900, 3, 1, 0, 0, 0.0,TimeSystem::Any);

      // Time of the last valid time
   const CommonTime MoonPosition::finalTime = CivilTime(2100, 2, 28, 0, 0, 0.0,TimeSystem::Any);


      // Coefficients for fundamental arguments
      // Units are degrees for position and Julian centuries for time

      // Moon's mean longitude
   const double MoonPosition::ELP0(270.434164),
                MoonPosition::ELP1(481267.8831),
                MoonPosition::ELP2(-0.001133),
                MoonPosition::ELP3(0.0000019);

      // Sun's mean anomaly
   const double MoonPosition::EM0(358.475833),
                MoonPosition::EM1(35999.0498),
                MoonPosition::EM2(-0.000150),
                MoonPosition::EM3(-0.0000033);

      // Moon's mean anomaly
   const double MoonPosition::EMP0(296.104608),
                MoonPosition::EMP1(477198.8491),
                MoonPosition::EMP2(0.009192),
                MoonPosition::EMP3(0.0000144);

      // Moon's mean elongation
   const double MoonPosition::D0(350.737486),
                MoonPosition::D1(445267.1142),
                MoonPosition::D2(-0.001436),
                MoonPosition::D3(0.0000019);

      // Mean distance of the Moon from its ascending node
   const double MoonPosition::F0(11.250889),
                MoonPosition::F1(483202.0251),
                MoonPosition::F2(-0.003211),
                MoonPosition::F3(-0.0000003);

      // Longitude of the Moon's ascending node
   const double MoonPosition::OM0(259.183275),
                MoonPosition::OM1(-1934.1420),
                MoonPosition::OM2(0.002078),
                MoonPosition::OM3(0.0000022);


      // Coefficients for (dimensionless) E factor
   const double MoonPosition::E1(-0.002495),
                MoonPosition::E2(-0.00000752);

      // Coefficients for periodic variations, etc
   const double MoonPosition::PAC(0.000233),
                MoonPosition::PA0(51.2),
                MoonPosition::PA1(20.2);
   const double MoonPosition::PBC(-0.001778);
   const double MoonPosition::PCC(0.000817);
   const double MoonPosition::PDC(0.002011);
   const double MoonPosition::PEC(0.003964),
                MoonPosition::PE0(346.560),
                MoonPosition::PE1(132.870),
                MoonPosition::PE2(-0.0091731);
   const double MoonPosition::PFC(0.001964);
   const double MoonPosition::PGC(0.002541);
   const double MoonPosition::PHC(0.001964);
   const double MoonPosition::cPIC(-0.024691);
   const double MoonPosition::PJC(-0.004328),
                MoonPosition::PJ0(275.05),
                MoonPosition::PJ1(-2.30);
   const double MoonPosition::CW1(0.0004664);
   const double MoonPosition::CW2(0.0000754);

      // Coefficients for Moon position
      //      Tx(N): coefficient of L, B or P term (deg)
      //      ITx(N,0-4): coefficients of M, M', D, F, E**n in argument
      //
   const size_t MoonPosition::NL(50),
                MoonPosition::NB(45),
                MoonPosition::NP(31);
   Vector<double> MoonPosition::TL(NL,0.0),
                  MoonPosition::TB(NB,0.0),
                  MoonPosition::TP(NP,0.0);
   Matrix<int> MoonPosition::ITL(5,NL,0),
               MoonPosition::ITB(5,NB,0),
               MoonPosition::ITP(5,NP,0);



      // Returns the position of Moon ECEF coordinates (meters) at 
      // the indicated time.
      //
      // @param[in] t   the time to look up
      //
      // @return the position of the Moon at time (as a Triple)
      //
      // @throw InvalidRequest If the request can not be completed for 
      // any reason, this is thrown. The text may have additional
      // information as to why the request failed.
   Triple MoonPosition::getPosition(const CommonTime& t) const
      throw(InvalidRequest)
   {
         // Test if the time interval is correct
      if ( (t < MoonPosition::initialTime) ||
           (t > MoonPosition::finalTime) )
      {
         InvalidRequest ir("Provided epoch is out of bounds.");
         GPSTK_THROW(ir);
      }


         // We will store here the results
      Triple res;

      res = MoonPosition::getPositionCIS(t);
      res = CIS2CTS(res, t);

      return res;

   } // End MoonPosition::getPosition



      /* Function to compute Moon position in CIS system (coordinates MJD
       * in meters)
       *
       * @param t Epoch
       */
   Triple MoonPosition::getPositionCIS(const CommonTime& t) const
      throw(InvalidRequest)
   {
         // Test if the time interval is correct
      if ( (t < MoonPosition::initialTime) ||
           (t > MoonPosition::finalTime) )
      {
         InvalidRequest ir("Provided epoch is out of bounds.");
         GPSTK_THROW(ir);
      }



         // Coefficients for Moon position
         //      Tx(N): coefficient of L, B or P term (deg)
         //      ITx(N,0-4): coefficients of M, M', D, F, E**n in argument
         //

         // Longitude
      TL( 0) = +6.288750;
      //          M              M'             D              F             n

      ITL(0, 0)= +0; ITL(1, 0)= +1; ITL(2, 0)= +0; ITL(3, 0)= +0, ITL(4, 0)= 0;
      TL( 1) = +1.274018;
      ITL(0, 1)= +0; ITL(1, 1)= -1; ITL(2, 1)= +2; ITL(3, 1)= +0, ITL(4, 1)= 0;
      TL( 2) = +0.658309;
      ITL(0, 2)= +0; ITL(1, 2)= +0; ITL(2, 2)= +2; ITL(3, 2)= +0, ITL(4, 2)= 0;
      TL( 3) = +0.213616;
      ITL(0, 3)= +0; ITL(1, 3)= +2; ITL(2, 3)= +0; ITL(3, 3)= +0, ITL(4, 3)= 0;
      TL( 4) = -0.185596;
      ITL(0, 4)= +1; ITL(1, 4)= +0; ITL(2, 4)= +0; ITL(3, 4)= +0, ITL(4, 4)= 1;
      TL( 5) = -0.114336;
      ITL(0, 5)= +0; ITL(1, 5)= +0; ITL(2, 5)= +0; ITL(3, 5)= +2, ITL(4, 5)= 0;
      TL( 6) = +0.058793;
      ITL(0, 6)= +0; ITL(1, 6)= -2; ITL(2, 6)= +2; ITL(3, 6)= +0, ITL(4, 6)= 0;
      TL( 7) = +0.057212;
      ITL(0, 7)= -1; ITL(1, 7)= -1; ITL(2, 7)= +2; ITL(3, 7)= +0, ITL(4, 7)= 1;
      TL( 8) = +0.053320;
      ITL(0, 8)= +0; ITL(1, 8)= +1; ITL(2, 8)= +2; ITL(3, 8)= +0, ITL(4, 8)= 0;
      TL( 9) = +0.045874;
      ITL(0, 9)= -1; ITL(1, 9)= +0; ITL(2, 9)= +2; ITL(3, 9)= +0, ITL(4, 9)= 1;
      TL(10) = +0.041024;
      ITL(0,10)= -1; ITL(1,10)= +1; ITL(2,10)= +0; ITL(3,10)= +0, ITL(4,10)= 1;
      TL(11) = -0.034718;
      ITL(0,11)= +0; ITL(1,11)= +0; ITL(2,11)= +1; ITL(3,11)= +0, ITL(4,11)= 0;
      TL(12) = -0.030465;
      ITL(0,12)= +1; ITL(1,12)= +1; ITL(2,12)= +0; ITL(3,12)= +0, ITL(4,12)= 1;
      TL(13) = +0.015326;
      ITL(0,13)= +0; ITL(1,13)= +0; ITL(2,13)= +2; ITL(3,13)= -2, ITL(4,13)= 0;
      TL(14) = -0.012528;
      ITL(0,14)= +0; ITL(1,14)= +1; ITL(2,14)= +0; ITL(3,14)= +2, ITL(4,14)= 0;
      TL(15) = -0.010980;
      ITL(0,15)= +0; ITL(1,15)= -1; ITL(2,15)= +0; ITL(3,15)= +2, ITL(4,15)= 0;
      TL(16) = +0.010674;
      ITL(0,16)= +0; ITL(1,16)= -1; ITL(2,16)= +4; ITL(3,16)= +0, ITL(4,16)= 0;
      TL(17) = +0.010034;
      ITL(0,17)= +0; ITL(1,17)= +3; ITL(2,17)= +0; ITL(3,17)= +0, ITL(4,17)= 0;
      TL(18) = +0.008548;
      ITL(0,18)= +0; ITL(1,18)= -2; ITL(2,18)= +4; ITL(3,18)= +0, ITL(4,18)= 0;
      TL(19) = -0.007910;
      ITL(0,19)= +1; ITL(1,19)= -1; ITL(2,19)= +2; ITL(3,19)= +0, ITL(4,19)= 1;
      TL(20) = -0.006783;
      ITL(0,20)= +1; ITL(1,20)= +0; ITL(2,20)= +2; ITL(3,20)= +0, ITL(4,20)= 1;
      TL(21) = +0.005162;
      ITL(0,21)= +0; ITL(1,21)= +1; ITL(2,21)= -1; ITL(3,21)= +0, ITL(4,21)= 0;
      TL(22) = +0.005000;
      ITL(0,22)= +1; ITL(1,22)= +0; ITL(2,22)= +1; ITL(3,22)= +0, ITL(4,22)= 1;
      TL(23) = +0.004049;
      ITL(0,23)= -1; ITL(1,23)= +1; ITL(2,23)= +2; ITL(3,23)= +0, ITL(4,23)= 1;
      TL(24) = +0.003996;
      ITL(0,24)= +0; ITL(1,24)= +2; ITL(2,24)= +2; ITL(3,24)= +0, ITL(4,24)= 0;
      TL(25) = +0.003862;
      ITL(0,25)= +0; ITL(1,25)= +0; ITL(2,25)= +4; ITL(3,25)= +0, ITL(4,25)= 0;
      TL(26) = +0.003665;
      ITL(0,26)= +0; ITL(1,26)= -3; ITL(2,26)= +2; ITL(3,26)= +0, ITL(4,26)= 0;
      TL(27) = +0.002695;
      ITL(0,27)= -1; ITL(1,27)= +2; ITL(2,27)= +0; ITL(3,27)= +0, ITL(4,27)= 1;
      TL(28) = +0.002602;
      ITL(0,28)= +0; ITL(1,28)= +1; ITL(2,28)= -2; ITL(3,28)= -2, ITL(4,28)= 0;
      TL(29) = +0.002396;
      ITL(0,29)= -1; ITL(1,29)= -2; ITL(2,29)= +2; ITL(3,29)= +0, ITL(4,29)= 1;
      TL(30) = -0.002349;
      ITL(0,30)= +0; ITL(1,30)= +1; ITL(2,30)= +1; ITL(3,30)= +0, ITL(4,30)= 0;
      TL(31) = +0.002249;
      ITL(0,31)= -2; ITL(1,31)= +0; ITL(2,31)= +2; ITL(3,31)= +0, ITL(4,31)= 2;
      TL(32) = -0.002125;
      ITL(0,32)= +1; ITL(1,32)= +2; ITL(2,32)= +0; ITL(3,32)= +0, ITL(4,32)= 1;
      TL(33) = -0.002079;
      ITL(0,33)= +2; ITL(1,33)= +0; ITL(2,33)= +0; ITL(3,33)= +0, ITL(4,33)= 2;
      TL(34) = +0.002059;
      ITL(0,34)= -2; ITL(1,34)= -1; ITL(2,34)= +2; ITL(3,34)= +0, ITL(4,34)= 2;
      TL(35) = -0.001773;
      ITL(0,35)= +0; ITL(1,35)= +1; ITL(2,35)= +2; ITL(3,35)= -2, ITL(4,35)= 0;
      TL(36) = -0.001595;
      ITL(0,36)= +0; ITL(1,36)= +0; ITL(2,36)= +2; ITL(3,36)= +2, ITL(4,36)= 0;
      TL(37) = +0.001220;
      ITL(0,37)= -1; ITL(1,37)= -1; ITL(2,37)= +4; ITL(3,37)= +0, ITL(4,37)= 1;
      TL(38) = -0.001110;
      ITL(0,38)= +0; ITL(1,38)= +2; ITL(2,38)= +0; ITL(3,38)= +2, ITL(4,38)= 0;
      TL(39) = +0.000892;
      ITL(0,39)= +0; ITL(1,39)= +1; ITL(2,39)= -3; ITL(3,39)= +0, ITL(4,39)= 0;
      TL(40) = -0.000811;
      ITL(0,40)= +1; ITL(1,40)= +1; ITL(2,40)= +2; ITL(3,40)= +0, ITL(4,40)= 1;
      TL(41) = +0.000761;
      ITL(0,41)= -1; ITL(1,41)= -2; ITL(2,41)= +4; ITL(3,41)= +0, ITL(4,41)= 1;
      TL(42) = +0.000717;
      ITL(0,42)= -2; ITL(1,42)= +1; ITL(2,42)= +0; ITL(3,42)= +0, ITL(4,42)= 2;
      TL(43) = +0.000704;
      ITL(0,43)= -2; ITL(1,43)= +1; ITL(2,43)= -2; ITL(3,43)= +0, ITL(4,43)= 2;
      TL(44) = +0.000693;
      ITL(0,44)= +1; ITL(1,44)= -2; ITL(2,44)= +2; ITL(3,44)= +0, ITL(4,44)= 1;
      TL(45) = +0.000598;
      ITL(0,45)= -1; ITL(1,45)= +0; ITL(2,45)= +2; ITL(3,45)= -2, ITL(4,45)= 1;
      TL(46) = +0.000550;
      ITL(0,46)= +0; ITL(1,46)= +1; ITL(2,46)= +4; ITL(3,46)= +0, ITL(4,46)= 0;
      TL(47) = +0.000538;
      ITL(0,47)= +0; ITL(1,47)= +4; ITL(2,47)= +0; ITL(3,47)= +0, ITL(4,47)= 0;
      TL(48) = +0.000521;
      ITL(0,48)= -1; ITL(1,48)= +0; ITL(2,48)= +4; ITL(3,48)= +0, ITL(4,48)= 1;
      TL(49) = +0.000486;
      ITL(0,49)= +0; ITL(1,49)= +2; ITL(2,49)= -1; ITL(3,49)= +0, ITL(4,49)= 0;


         // Latitude
      TB( 0) = +5.128189;
      //          M              M'             D              F             n
      ITB(0, 0)= +0; ITB(1, 0)= +0; ITB(2, 0)= +0; ITB(3, 0)= +1, ITB(4, 0)= 0;
      TB( 1) = +0.280606;
      ITB(0, 1)= +0; ITB(1, 1)= +1; ITB(2, 1)= +0; ITB(3, 1)= +1, ITB(4, 1)= 0;
      TB( 2) = +0.277693;
      ITB(0, 2)= +0; ITB(1, 2)= +1; ITB(2, 2)= +0; ITB(3, 2)= -1, ITB(4, 2)= 0;
      TB( 3) = +0.173238;
      ITB(0, 3)= +0; ITB(1, 3)= +0; ITB(2, 3)= +2; ITB(3, 3)= -1, ITB(4, 3)= 0;
      TB( 4) = +0.055413;
      ITB(0, 4)= +0; ITB(1, 4)= -1; ITB(2, 4)= +2; ITB(3, 4)= +1, ITB(4, 4)= 0;
      TB( 5) = +0.046272;
      ITB(0, 5)= +0; ITB(1, 5)= -1; ITB(2, 5)= +2; ITB(3, 5)= -1, ITB(4, 5)= 0;
      TB( 6) = +0.032573;
      ITB(0, 6)= +0; ITB(1, 6)= +0; ITB(2, 6)= +2; ITB(3, 6)= +1, ITB(4, 6)= 0;
      TB( 7) = +0.017198;
      ITB(0, 7)= +0; ITB(1, 7)= +2; ITB(2, 7)= +0; ITB(3, 7)= +1, ITB(4, 7)= 0;
      TB( 8) = +0.009267;
      ITB(0, 8)= +0; ITB(1, 8)= +1; ITB(2, 8)= +2; ITB(3, 8)= -1, ITB(4, 8)= 0;
      TB( 9) = +0.008823;
      ITB(0, 9)= +0; ITB(1, 9)= +2; ITB(2, 9)= +0; ITB(3, 9)= -1, ITB(4, 9)= 0;
      TB(10) = +0.008247;
      ITB(0,10)= -1; ITB(1,10)= +0; ITB(2,10)= +2; ITB(3,10)= -1, ITB(4,10)= 1;
      TB(11) = +0.004323;
      ITB(0,11)= +0; ITB(1,11)= -2; ITB(2,11)= +2; ITB(3,11)= -1, ITB(4,11)= 0;
      TB(12) = +0.004200;
      ITB(0,12)= +0; ITB(1,12)= +1; ITB(2,12)= +2; ITB(3,12)= +1, ITB(4,12)= 0;
      TB(13) = +0.003372;
      ITB(0,13)= -1; ITB(1,13)= +0; ITB(2,13)= -2; ITB(3,13)= +1, ITB(4,13)= 1;
      TB(14) = +0.002472;
      ITB(0,14)= -1; ITB(1,14)= -1; ITB(2,14)= +2; ITB(3,14)= +1, ITB(4,14)= 1;
      TB(15) = +0.002222;
      ITB(0,15)= -1; ITB(1,15)= +0; ITB(2,15)= +2; ITB(3,15)= +1, ITB(4,15)= 1;
      TB(16) = +0.002072;
      ITB(0,16)= -1; ITB(1,16)= -1; ITB(2,16)= +2; ITB(3,16)= -1, ITB(4,16)= 1;
      TB(17) = +0.001877;
      ITB(0,17)= -1; ITB(1,17)= +1; ITB(2,17)= +0; ITB(3,17)= +1, ITB(4,17)= 1;
      TB(18) = +0.001828;
      ITB(0,18)= +0; ITB(1,18)= -1; ITB(2,18)= +4; ITB(3,18)= -1, ITB(4,18)= 0;
      TB(19) = -0.001803;
      ITB(0,19)= +1; ITB(1,19)= +0; ITB(2,19)= +0; ITB(3,19)= +1, ITB(4,19)= 1;
      TB(20) = -0.001750;
      ITB(0,20)= +0; ITB(1,20)= +0; ITB(2,20)= +0; ITB(3,20)= +3, ITB(4,20)= 0;
      TB(21) = +0.001570;
      ITB(0,21)= -1; ITB(1,21)= +1; ITB(2,21)= +0; ITB(3,21)= -1, ITB(4,21)= 1;
      TB(22) = -0.001487;
      ITB(0,22)= +0; ITB(1,22)= +0; ITB(2,22)= +1; ITB(3,22)= +1, ITB(4,22)= 0;
      TB(23) = -0.001481;
      ITB(0,23)= +1; ITB(1,23)= +1; ITB(2,23)= +0; ITB(3,23)= +1, ITB(4,23)= 1;
      TB(24) = +0.001417;
      ITB(0,24)= -1; ITB(1,24)= -1; ITB(2,24)= +0; ITB(3,24)= +1, ITB(4,24)= 1;
      TB(25) = +0.001350;
      ITB(0,25)= -1; ITB(1,25)= +0; ITB(2,25)= +0; ITB(3,25)= +1, ITB(4,25)= 1;
      TB(26) = +0.001330;
      ITB(0,26)= +0; ITB(1,26)= +0; ITB(2,26)= -1; ITB(3,26)= +1, ITB(4,26)= 0;
      TB(27) = +0.001106;
      ITB(0,27)= +0; ITB(1,27)= +3; ITB(2,27)= +0; ITB(3,27)= +1, ITB(4,27)= 0;
      TB(28) = +0.001020;
      ITB(0,28)= +0; ITB(1,28)= +0; ITB(2,28)= +4; ITB(3,28)= -1, ITB(4,28)= 0;
      TB(29) = +0.000833;
      ITB(0,29)= +0; ITB(1,29)= -1; ITB(2,29)= +4; ITB(3,29)= +1, ITB(4,29)= 0;
      TB(30) = +0.000781;
      ITB(0,30)= +0; ITB(1,30)= +1; ITB(2,30)= +0; ITB(3,30)= -3, ITB(4,30)= 0;
      TB(31) = +0.000670;
      ITB(0,31)= +0; ITB(1,31)= -2; ITB(2,31)= +4; ITB(3,31)= +1, ITB(4,31)= 0;
      TB(32) = +0.000606;
      ITB(0,32)= +0; ITB(1,32)= +0; ITB(2,32)= +2; ITB(3,32)= -3, ITB(4,32)= 0;
      TB(33) = +0.000597;
      ITB(0,33)= +0; ITB(1,33)= +2; ITB(2,33)= +2; ITB(3,33)= -1, ITB(4,33)= 0;
      TB(34) = +0.000492;
      ITB(0,34)= -1; ITB(1,34)= +1; ITB(2,34)= +2; ITB(3,34)= -1, ITB(4,34)= 1;
      TB(35) = +0.000450;
      ITB(0,35)= +0; ITB(1,35)= +2; ITB(2,35)= -2; ITB(3,35)= -1, ITB(4,35)= 0;
      TB(36) = +0.000439;
      ITB(0,36)= +0; ITB(1,36)= +3; ITB(2,36)= +0; ITB(3,36)= -1, ITB(4,36)= 0;
      TB(37) = +0.000423;
      ITB(0,37)= +0; ITB(1,37)= +2; ITB(2,37)= +2; ITB(3,37)= +1, ITB(4,37)= 0;
      TB(38) = +0.000422;
      ITB(0,38)= +0; ITB(1,38)= -3; ITB(2,38)= +2; ITB(3,38)= -1, ITB(4,38)= 0;
      TB(39) = -0.000367;
      ITB(0,39)= +1; ITB(1,39)= -1; ITB(2,39)= +2; ITB(3,39)= +1, ITB(4,39)= 1;
      TB(40) = -0.000353;
      ITB(0,40)= +1; ITB(1,40)= +0; ITB(2,40)= +2; ITB(3,40)= +1, ITB(4,40)= 1;
      TB(41) = +0.000331;
      ITB(0,41)= +0; ITB(1,41)= +0; ITB(2,41)= +4; ITB(3,41)= +1, ITB(4,41)= 0;
      TB(42) = +0.000317;
      ITB(0,42)= -1; ITB(1,42)= +1; ITB(2,42)= +2; ITB(3,42)= +1, ITB(4,42)= 1;
      TB(43) = +0.000306;
      ITB(0,43)= -2; ITB(1,43)= +0; ITB(2,43)= +2; ITB(3,43)= -1, ITB(4,43)= 2;
      TB(44) = -0.000283;
      ITB(0,44)= +0; ITB(1,44)= +1; ITB(2,44)= +0; ITB(3,44)= +3, ITB(4,44)= 0;


         // Parallax
      TP( 0) = +0.950724;
      //          M              M'             D              F             n
      ITP(0, 0)= +0; ITP(1, 0)= +0; ITP(2, 0)= +0; ITP(3, 0)= +0, ITP(4, 0)= 0;
      TP( 1) = +0.051818;
      ITP(0, 1)= +0; ITP(1, 1)= +1; ITP(2, 1)= +0; ITP(3, 1)= +0, ITP(4, 1)= 0;
      TP( 2) = +0.009531;
      ITP(0, 2)= +0; ITP(1, 2)= -1; ITP(2, 2)= +2; ITP(3, 2)= +0, ITP(4, 2)= 0;
      TP( 3) = +0.007843;
      ITP(0, 3)= +0; ITP(1, 3)= +0; ITP(2, 3)= +2; ITP(3, 3)= +0, ITP(4, 3)= 0;
      TP( 4) = +0.002824;
      ITP(0, 4)= +0; ITP(1, 4)= +2; ITP(2, 4)= +0; ITP(3, 4)= +0, ITP(4, 4)= 0;
      TP( 5) = +0.000857;
      ITP(0, 5)= +0; ITP(1, 5)= +1; ITP(2, 5)= +2; ITP(3, 5)= +0, ITP(4, 5)= 0;
      TP( 6) = +0.000533;
      ITP(0, 6)= -1; ITP(1, 6)= +0; ITP(2, 6)= +2; ITP(3, 6)= +0, ITP(4, 6)= 1;
      TP( 7) = +0.000401;
      ITP(0, 7)= -1; ITP(1, 7)= -1; ITP(2, 7)= +2; ITP(3, 7)= +0, ITP(4, 7)= 1;
      TP( 8) = +0.000320;
      ITP(0, 8)= -1; ITP(1, 8)= +1; ITP(2, 8)= +0; ITP(3, 8)= +0, ITP(4, 8)= 1;
      TP( 9) = -0.000271;
      ITP(0, 9)= +0; ITP(1, 9)= +0; ITP(2, 9)= +1; ITP(3, 9)= +0, ITP(4, 9)= 0;
      TP(10) = -0.000264;
      ITP(0,10)= +1; ITP(1,10)= +1; ITP(2,10)= +0; ITP(3,10)= +0, ITP(4,10)= 1;
      TP(11) = -0.000198;
      ITP(0,11)= +0; ITP(1,11)= -1; ITP(2,11)= +0; ITP(3,11)= +2, ITP(4,11)= 0;
      TP(12) = +0.000173;
      ITP(0,12)= +0; ITP(1,12)= +3; ITP(2,12)= +0; ITP(3,12)= +0, ITP(4,12)= 0;
      TP(13) = +0.000167;
      ITP(0,13)= +0; ITP(1,13)= -1; ITP(2,13)= +4; ITP(3,13)= +0, ITP(4,13)= 0;
      TP(14) = -0.000111;
      ITP(0,14)= +1; ITP(1,14)= +0; ITP(2,14)= +0; ITP(3,14)= +0, ITP(4,14)= 1;
      TP(15) = +0.000103;
      ITP(0,15)= +0; ITP(1,15)= -2; ITP(2,15)= +4; ITP(3,15)= +0, ITP(4,15)= 0;
      TP(16) = -0.000084;
      ITP(0,16)= +0; ITP(1,16)= +2; ITP(2,16)= -2; ITP(3,16)= +0, ITP(4,16)= 0;
      TP(17) = -0.000083;
      ITP(0,17)= +1; ITP(1,17)= +0; ITP(2,17)= +2; ITP(3,17)= +0, ITP(4,17)= 1;
      TP(18) = +0.000079;
      ITP(0,18)= +0; ITP(1,18)= +2; ITP(2,18)= +2; ITP(3,18)= +0, ITP(4,18)= 0;
      TP(19) = +0.000072;
      ITP(0,19)= +0; ITP(1,19)= +0; ITP(2,19)= +4; ITP(3,19)= +0, ITP(4,19)= 0;
      TP(20) = +0.000064;
      ITP(0,20)= -1; ITP(1,20)= +1; ITP(2,20)= +2; ITP(3,20)= +0, ITP(4,20)= 1;
      TP(21) = -0.000063;
      ITP(0,21)= +1; ITP(1,21)= -1; ITP(2,21)= +2; ITP(3,21)= +0, ITP(4,21)= 1;
      TP(22) = +0.000041;
      ITP(0,22)= +1; ITP(1,22)= +0; ITP(2,22)= +1; ITP(3,22)= +0, ITP(4,22)= 1;
      TP(23) = +0.000035;
      ITP(0,23)= -1; ITP(1,23)= +2; ITP(2,23)= +0; ITP(3,23)= +0, ITP(4,23)= 1;
      TP(24) = -0.000033;
      ITP(0,24)= +0; ITP(1,24)= +3; ITP(2,24)= -2; ITP(3,24)= +0, ITP(4,24)= 0;
      TP(25) = -0.000030;
      ITP(0,25)= +0; ITP(1,25)= +1; ITP(2,25)= +1; ITP(3,25)= +0, ITP(4,25)= 0;
      TP(26) = -0.000029;
      ITP(0,26)= +0; ITP(1,26)= +0; ITP(2,26)= -2; ITP(3,26)= +2, ITP(4,26)= 0;
      TP(27) = -0.000029;
      ITP(0,27)= +1; ITP(1,27)= +2; ITP(2,27)= +0; ITP(3,27)= +0, ITP(4,27)= 1;
      TP(28) = +0.000026;
      ITP(0,28)= -2; ITP(1,28)= +0; ITP(2,28)= +2; ITP(3,28)= +0, ITP(4,28)= 2;
      TP(29) = -0.000023;
      ITP(0,29)= +0; ITP(1,29)= +1; ITP(2,29)= -2; ITP(3,29)= +2, ITP(4,29)= 0;
      TP(30) = +0.000019;
      ITP(0,30)= -1; ITP(1,30)= -1; ITP(2,30)= +4; ITP(3,30)= +0, ITP(4,30)= 1;


         // Centuries since J1900
      double tt((MJD(t).mjd-15019.5)/36525.0);


         // Fundamental arguments (radians) and derivatives (radians per
         // Julian century) for the current epoch

         // Moon's mean longitude
      double ELP(D2R*fmod( (ELP0+(ELP1+(ELP2+ELP3*tt)*tt)*tt), 360.0));
      double DELP(D2R*(ELP1+(2.0*ELP2+3.0*ELP3*tt)*tt));

         // Sun's mean anomaly
      double EM(D2R*fmod( (EM0+(EM1+(EM2+EM3*tt)*tt)*tt), 360.0));
      double DEM(D2R*(EM1+(2.0*EM2+3.0*EM3*tt)*tt));

         // Moon's mean anomaly
      double EMP(D2R*fmod( (EMP0+(EMP1+(EMP2+EMP3*tt)*tt)*tt), 360.0));
      double DEMP(D2R*(EMP1+(2.0*EMP2+3.0*EMP3*tt)*tt));

         // Moon's mean elongation
      double D(D2R*fmod( (D0+(D1+(D2+D3*tt)*tt)*tt), 360.0));
      double DD(D2R*(D1+(2.0*D2+3.0*D3*tt)*tt));

         // Mean distance of the Moon from its ascending node
      double F(D2R*fmod( (F0+(F1+(F2+F3*tt)*tt)*tt), 360.0));
      double DF(D2R*(F1+(2.0*F2+3.0*F3*tt)*tt));

         // Longitude of the Moon's ascending node
      double OM(D2R*fmod( (OM0+(OM1+(OM2+OM3*tt)*tt)*tt), 360.0));
      double DOM(D2R*(OM1+(2.0*OM2+3.0*OM3*tt)*tt));
      double SINOM(std::sin(OM));
      double COSOM(std::cos(OM));
      double DOMCOM(DOM*COSOM);


         // Let's add the periodic variations
      double  THETA(D2R*(PA0+PA1*tt));
      double  WA(std::sin(THETA));
      double  DWA(D2R*PA1*std::cos(THETA));
      THETA = D2R*(PE0+(PE1+PE2*tt)*tt);
      double  WB(PEC*std::sin(THETA));
      double  DWB(D2R*PEC*(PE1+2.0*PE2*tt)*std::cos(THETA));
      ELP   = ELP+D2R*(PAC*WA+WB+PFC*SINOM);
      DELP  = DELP+D2R*(PAC*DWA+DWB+PFC*DOMCOM);
      EM    = EM+D2R*PBC*WA;
      DEM   = DEM+D2R*PBC*DWA;
      EMP   = EMP+D2R*(PCC*WA+WB+PGC*SINOM);
      DEMP  = DEMP+D2R*(PCC*DWA+DWB+PGC*DOMCOM);
      D     = D+D2R*(PDC*WA+WB+PHC*SINOM);
      DD    = DD+D2R*(PDC*DWA+DWB+PHC*DOMCOM);
      double  WOM(OM+D2R*(PJ0+PJ1*tt));
      double  DWOM(DOM+D2R*PJ1);
      double  SINWOM(std::sin(WOM));
      double  COSWOM(std::cos(WOM));
      F     = F+D2R*(WB+cPIC*SINOM+PJC*SINWOM);
      DF    = DF+D2R*(DWB+cPIC*DOMCOM+PJC*DWOM*COSWOM);


         // E-factor, and square
      double  E(1.0+(E1+E2*tt)*tt);
      double  DE(E1+2.0*E2*tt);
      double  ESQ(E*E);
      double  DESQ(2.0*E*DE);


         // Series expansions

         // Longitude
      double V(0.0);
      double DV(0.0);
      for (int n=(NL-1); n>=0; n--)
      {
         double EN, DEN;
         double COEFF(TL(n));
         double EMN(static_cast<double>(ITL(0,n)));
         double EMPN(static_cast<double>(ITL(1,n)));
         double DN(static_cast<double>(ITL(2,n)));
         double FN(static_cast<double>(ITL(3,n)));
         int I=ITL(4,n);
         if (I == 0)
         {
            EN  = 1.0;
            DEN = 0.0;
         }
         else
         {
            if (I == 1)
            {
               EN  = E;
               DEN = DE;
            }
            else
            {
               EN  = ESQ;
               DEN = DESQ;
            }
         }
         THETA = EMN*EM+EMPN*EMP+DN*D+FN*F;
         double DTHETA(EMN*DEM+EMPN*DEMP+DN*DD+FN*DF);
         double FTHETA(std::sin(THETA));
         V  = V+COEFF*FTHETA*EN;
         DV = DV+COEFF*(std::cos(THETA)*DTHETA*EN+FTHETA*DEN);
      }
      double EL(ELP+D2R*V);


         // Latitude
      V  = 0.0;
      DV = 0.0;
      for (int n=(NB-1); n>=0; n--)
      {
         double EN, DEN;
         double COEFF(TB(n));
         double EMN(static_cast<double>(ITB(0,n)));
         double EMPN(static_cast<double>(ITB(1,n)));
         double DN(static_cast<double>(ITB(2,n)));
         double FN(static_cast<double>(ITB(3,n)));
         int I=ITB(4,n);
         if (I == 0)
         {
            EN  = 1.0;
            DEN = 0.0;
         }
         else
         {
            if (I == 1)
            {
               EN  = E;
               DEN = DE;
            }
            else
            {
               EN  = ESQ;
               DEN = DESQ;
            }
         }
         THETA = EMN*EM+EMPN*EMP+DN*D+FN*F;
         double DTHETA(EMN*DEM+EMPN*DEMP+DN*DD+FN*DF);
         double FTHETA(std::sin(THETA));
         V  = V+COEFF*FTHETA*EN;
         DV = DV+COEFF*(std::cos(THETA)*DTHETA*EN+FTHETA*DEN);
      }
      double BF(1.0-CW1*COSOM-CW2*COSWOM);
      double B(D2R*V*BF);


         // Parallax
      V  = 0.0;
      DV = 0.0;
      for (int n=(NP-1); n>=0; n--)
      {
         double EN, DEN;
         double COEFF(TP(n));
         double EMN(static_cast<double>(ITP(0,n)));
         double EMPN(static_cast<double>(ITP(1,n)));
         double DN(static_cast<double>(ITP(2,n)));
         double FN(static_cast<double>(ITP(3,n)));
         int I=ITP(4,n);
         if (I == 0)
         {
            EN  = 1.0;
            DEN = 0.0;
         }
         else
         {
            if (I == 1)
            {
               EN  = E;
               DEN = DE;
            }
            else
            {
               EN  = ESQ;
               DEN = DESQ;
            }
         }
         THETA = EMN*EM+EMPN*EMP+DN*D+FN*F;
         double DTHETA(EMN*DEM+EMPN*DEMP+DN*DD+FN*DF);
         double FTHETA(std::cos(THETA));
         V  = V+COEFF*FTHETA*EN;
         DV = DV+COEFF*(-std::sin(THETA)*DTHETA*EN+FTHETA*DEN);
      }
      double P(D2R*V);


         // Transformation into final form

         // Parallax to distance (AU, AU/sec)
      double SP(std::sin(P));
      double R(ERADAU/SP);

         // Longitude, latitude to x,y,z (AU)
      double SEL(std::sin(EL));
      double CEL(std::cos(EL));
      double SB(std::sin(B));
      double CB(std::cos(B));
      double RCB(R*CB);
      double X(RCB*CEL);
      double Y(RCB*SEL);
      double Z(R*SB);


         // Julian centuries since J2000
      tt=(MJD(t).mjd-51544.5)/36525.0;

         // Fricke equinox correction
      double EPJ(2000.0+tt*100.0);
      double EQCOR(DS2R*(0.035+0.00085*(EPJ-B1950)));

         // Mean obliquity (IAU 1976)
      double EPS(DAS2R*(84381.448 + (-46.8150 + 
                        (-0.00059+0.001813*tt)*tt)*tt));

         // Change to equatorial system, mean of date, FK5 system
      double SINEPS(std::sin(EPS));
      double COSEPS(std::cos(EPS));
      double ES(EQCOR*SINEPS);
      double EC(EQCOR*COSEPS);

      Triple res;

      res.theArray[0] = (X-EC*Y+ES*Z)*AU_CONST;
      res.theArray[1] = (EQCOR*X+Y*COSEPS-Z*SINEPS)*AU_CONST;
      res.theArray[2] = (Y*SINEPS+Z*COSEPS)*AU_CONST;


      return res;

   } // End MoonPosition::getPositionCIS()


} // end namespace gpstk
