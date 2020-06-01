#pragma ident "$Id$"

/**
 * @file StreamBuf.hpp
 * 
 */

#ifndef GPSTK_STREAMBUF_HPP
#define GPSTK_STREAMBUF_HPP

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
//  Wei Yan - Chinese Academy of Sciences . 2011
//
//============================================================================

#include <streambuf>
#include <iosfwd>
#include <ios>

namespace gpstk
{
      /// This class easy implement the custom streambufs.
   template <typename ch, typename tr> 
   class BasicStreamBuf: public std::basic_streambuf<ch, tr>
   {
   protected:
      typedef std::basic_streambuf<ch, tr> Base;
      typedef std::basic_ios<ch, tr> IOS;
      typedef ch char_type;
      typedef tr char_traits;
      typedef typename Base::int_type int_type;
      typedef typename Base::pos_type pos_type;
      typedef typename Base::off_type off_type;
      typedef typename IOS::openmode openmode;

   public:
      BasicStreamBuf() : _pb(char_traits::eof()), _ispb(false)
      {
         this->setg(0, 0, 0);
         this->setp(0, 0);
      }

      ~BasicStreamBuf()
      {
      }

      virtual int_type overflow(int_type c)
      {
         if (c != char_traits::eof()) 
            return writeToDevice(char_traits::to_char_type(c));
         else
            return c;
      }

      virtual int_type underflow()
      {
         if (_ispb)
         {
            return _pb;
         }
         else
         {
            int_type c = readFromDevice();
            if (c != char_traits::eof())
            {
               _ispb = true;
               _pb   = c;
            }
            return c;
         }
      }

      virtual int_type uflow()
      {
         if (_ispb)
         {
            _ispb = false;
            return _pb;
         }
         else
         {
            int_type c = readFromDevice();
            if (c != char_traits::eof())
            {
               _pb = c;
            }
            return c;
         }
      }

      virtual int_type pbackfail(int_type c)
      {
         if (_ispb)
         {
            return char_traits::eof();
         }
         else
         {
            _ispb = true;
            _pb   = c;
            return c;
         }
      }

      virtual std::streamsize xsgetn(char_type* p, std::streamsize count)
      {
         std::streamsize copied = 0;
         while (count > 0)
         {
            int_type c = uflow();
            if (c == char_traits::eof()) break;
            *p++ = char_traits::to_char_type(c);
            ++copied;
            --count;
         }
         return copied;
      }

   protected:
      static int_type charToInt(char_type c)
      {
         return char_traits::to_int_type(c);
      }

   private:
      virtual int_type readFromDevice()
      {
         return char_traits::eof();
      }

      virtual int_type writeToDevice(char_type)
      {
         return char_traits::eof();
      }

      int_type _pb;
      bool     _ispb;

      BasicStreamBuf(const BasicStreamBuf&);
      BasicStreamBuf& operator = (const BasicStreamBuf&);

   }; // End of class 'BasicStreamBuf'

   typedef BasicStreamBuf<char, std::char_traits<char> > StreamBuf;

}   // End of namespace gpstk



#endif  //GPSTK_BASICSTREAMBUF_HPP

