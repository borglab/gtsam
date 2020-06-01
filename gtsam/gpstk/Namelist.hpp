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
 * @file Namelist.hpp
 * Include file defining class Namelist.
 * class gpstk::Namelist encapsulates a list of labels for use with classes Matrix,
 * Vector and SRI.
 */

#ifndef CLASS_NAMELIST_INCLUDE
#define CLASS_NAMELIST_INCLUDE

//------------------------------------------------------------------------------------
// system includes
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <ostream>
#include <sstream>
// GPSTk
#include "Matrix.hpp"

namespace gpstk
{

//------------------------------------------------------------------------------------
class Namelist;
/// class LabelledVector. Pretty print a Vector using the labels in a Namelist.
class LabelledVector {
public:
   int wid,prec,form;
   std::string msg;
   std::string tag;
   const Namelist& NL;
   Vector<double>& V;
   LabelledVector(const Namelist& nl, Vector<double>& v)
      : wid(12), prec(5), form(1), NL(nl), V(v) { }
   LabelledVector& setw(int w) { wid = w; return *this; }
   LabelledVector& setprecision(int p) { prec = p; return *this; }
   LabelledVector& fixed(void) { form = 1; return *this; }
   LabelledVector& scientific(void) { form = 2; return *this; }
   LabelledVector& message(const std::string& m) { msg=m; return *this; }
   LabelledVector& linetag(const std::string& m) { tag=m; return *this; }
};

/// class LabelledMatrix. Pretty print a Matrix using the labels in a Namelist.
class LabelledMatrix {
public:
   int wid,prec;
   int form;         // format: 1=fixed, 2=scientific
   int rc;           // rows only (1) columns only (2) or both (0)
   std::string msg;
   std::string tag;
   const Namelist& NLrows;
   const Namelist& NLcols;
   const Matrix<double>& M;
   LabelledMatrix(const Namelist& nl, const Matrix<double>& m)
      : wid(12), prec(5), form(1), rc(0), NLrows(nl), NLcols(nl), M(m) { }
   LabelledMatrix(const Namelist& nlr, const Namelist& nlc, const Matrix<double>& m)
      : wid(12), prec(5), form(1), rc(0), NLrows(nlr), NLcols(nlc), M(m) { }
   LabelledMatrix& setw(int w) { wid = w; return *this; }
   LabelledMatrix& setprecision(int p) { prec = p; return *this; }
   LabelledMatrix& fixed(void) { form = 1; return *this; }
   LabelledMatrix& scientific(void) { form = 2; return *this; }
   LabelledMatrix& both(void) { rc=0; return *this; }
   LabelledMatrix& rows(void) { rc=1; return *this; }
   LabelledMatrix& cols(void) { rc=2; return *this; }
   LabelledMatrix& message(const std::string& m) { msg=m; return *this; }
   LabelledMatrix& linetag(const std::string& m) { tag=m; return *this; }
};

std::ostream& operator<<(std::ostream&, const LabelledMatrix&);
std::ostream& operator<<(std::ostream&, const LabelledVector&);


//------------------------------------------------------------------------------------
/** class Namelist. A Namelist is simply an ordered set of unique strings ('names' or
 * 'labels') of any length. Namelists are used to label or identify elements of
 * Vectors or Matrix rows and columns. Namelist is particularly useful in class SRI,
 * which includes a Matrix and Vector that hold state and covariance information in
 * an estimation problem; SRI include a Namelist which associates readable labels with
 * the elements of the state and covariance.
 */

class Namelist {
   friend class SRI;
   friend std::ostream& operator<<(std::ostream&, const LabelledMatrix&);
   friend std::ostream& operator<<(std::ostream&, const LabelledVector&);
public:
      /// empty constructor
   Namelist(void) {}
      /// constructor given dimension - creates default labels
   Namelist(const unsigned int&);
      /// explicit constructor - only a unique subset of the input will be included.
   Namelist(const std::vector<std::string>&);
      /// copy constructor
   Namelist(const Namelist& names) { labels = names.labels; }
      /// destructor
   ~Namelist(void) { labels.clear(); }

      /// operator=
   Namelist& operator=(const Namelist& right)
      { labels = right.labels; return *this; }
      /// add a single name to the Namelist; do nothing if the name is not unique.
   Namelist& operator+=(const std::string&);
      /// remove a name from the Namelist; does nothing if the name is not found.
   Namelist& operator-=(const std::string&);

      /// swap two elements, as given by their indexes; no effect if either index
      /// is out of range.
   void swap(const unsigned int&, const unsigned int&);
      /// reorder the list by sorting
   void sort(void);
      /// resize the list by either truncation or adding default names.
   void resize(unsigned int);
      /// randomize the list
   void randomize(long seed=0);
      /// empty the list
   void clear(void) { labels.clear(); }

      /// is the Namelist valid? checks for repeated names
      /// (? not possible to create an invalid Namelist?)
   bool valid(void) const;
      /// does the Namelist contain the input name?
   bool contains(const std::string&) const;
      /// are two Namelists identical, ignoring permutations?
   friend bool operator==(const Namelist&, const Namelist&);
      /// are two Namelists different, ignoring permutations?
   friend bool operator!=(const Namelist&, const Namelist&);
      /// are two Namelists exactly identical, even considering permutations?
   friend bool identical(const Namelist&, const Namelist&);

      /// construct the subset Namelist which is common to the two inputs (AND)
   friend Namelist operator&(const Namelist&, const Namelist&);
      /// merge two Namelists, i.e. construct a non-redundant combination (OR)
   friend Namelist operator|(const Namelist&, const Namelist&);
      /// construct the subset Namelist which is NOT common to two others (XOR)
   friend Namelist operator^(const Namelist&, const Namelist&);
      /// replace this with (this & input)
   Namelist& operator&=(const Namelist&);
      /// replace this with (this | input)
   Namelist& operator|=(const Namelist&);
      /// replace this with (this ^ input)
   Namelist& operator^=(const Namelist&);

      /// bind a Namelist to a Matrix<double> before sending it to an output
      /// stream, to get a 'labelled display' of the matrix.
   LabelledMatrix operator()(Matrix<double>& m)
      { return LabelledMatrix(*this,m); }

      /// bind a Namelist to a Vector<double> before sending it to an output
      /// stream, to get a 'labelled display' of the vector.
   LabelledVector operator()(Vector<double>& v)
      { return LabelledVector(*this,v); }

      // member access

      /// return the size of the list.
   inline unsigned int size(void) const { return labels.size(); }

   //std::string& operator[](const unsigned int);
   //Don't do this, b/c it means user could create invalid Namelists,
   //and b/c passing an invalid int would require a throw

      /// access to a specific name, given its index.
      /// returns 'out-of-range' if the index is out of range.
   std::string getName(const unsigned int) const;

      /// assign a specific name, given its index;
      /// no effect if the name is not unique;
      /// return true if successful.
   bool setName(const unsigned int, const std::string&);

      /// return the index of the name in the list that matches the input,
      /// -1 if not found.
   int index(const std::string&) const;

      /// output operator
   friend std::ostream& operator<<(std::ostream& s, const Namelist&);

   // member data

      /// vector of names (strings)
   std::vector<std::string> labels;

//private:
}; // end class Namelist

} // end of namespace gpstk

#endif
