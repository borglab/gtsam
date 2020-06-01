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
 * @file Namelist.cpp
 * Implementation of class Namelist.
 * class gpstk::Namelist encapsulates a list of labels for use with classes Matrix,
 * Vector and SRI.
 */

//------------------------------------------------------------------------------------
// system includes
#include <string>
#include <vector>
#include <algorithm>
#include <ostream>
#include <fstream> // for copyfmt
// GPSTk
#include "Exception.hpp"
#include "StringUtils.hpp"
#include "Namelist.hpp"

using namespace std;

namespace gpstk
{

using namespace StringUtils;

//------------------------------------------------------------------------------------
// constructor given dimension - creates default labels
Namelist::Namelist(const unsigned int& n)
{
try {
   if(n == 0) return;
   string name;
   for(unsigned int i=0; i<n; i++) {
      ostringstream oss;
      oss << "NAME" << setw(3) << setfill('0') << i;
      name = oss.str();
      labels.push_back(name);
   }
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// explicit constructor - only a unique subset of names will be included.
Namelist::Namelist(const vector<string>& names)
{
try {
   for(unsigned int i=0; i<names.size(); i++) {
      bool unique=true;
      for(unsigned int j=i+1; j<names.size(); j++) {
         if(names[i] == names[j]) { unique=false; break; }
      }
      if(unique) labels.push_back(names[i]);
   }
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// add a name to the Namelist; do nothing if the name is not unique.
Namelist& Namelist::operator+=(const string& name)
{
try {
   if(this->contains(name)) return *this;
   labels.push_back(name);
   return *this;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// remove a name from the Namelist; does nothing if the name is not found.
Namelist& Namelist::operator-=(const string& name)
{
try {
   vector<string>::iterator it;
   it = find(labels.begin(),labels.end(),name);
   if(it != labels.end())
      labels.erase(it);
   return *this;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// swap two elements, as given by their indexes; no effect if either index
// is out of range.
void Namelist::swap(const unsigned int& i, const unsigned int& j)
{
try {
   if(i == j) return;
   if(i >= labels.size() || j >= labels.size()) return;
   string str = labels[i];
   labels[i] = labels[j];
   labels[j] = str;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// reorder the list by sorting
void Namelist::sort(void)
{
try {
   // compiler tries Namelist::sort() first...
   std::sort(labels.begin(),labels.end());
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// resize the list by either truncation or adding default names.
void Namelist::resize(unsigned int n)
{
try {
   if(n == labels.size()) return;
   int N=labels.size();
   while(labels.size() < n) {
      string s;
      do {
         ostringstream oss;
         oss << "NAME" << setw(3) << setfill('0') << N;
         s = oss.str();
         N++;
      } while(this->contains(s));
      labels.push_back(s);
   }
   while(labels.size() > n) {
      labels.pop_back();
   }
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// randomize the list
void Namelist::randomize(long seed)
{
try {
   if(labels.size() <= 1) return;
   random_shuffle(labels.begin(), labels.end());
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// is the Namelist valid? checks for repeated names
bool Namelist::valid(void) const
{
try {
   for(unsigned int i=0; i<labels.size(); i++)
      for(unsigned int j=i+1; j<labels.size(); j++)
         if(labels[i] == labels[j]) return false;
   return true;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// does the Namelist contain the input name?
bool Namelist::contains(const string& name) const
{
try {
   for(unsigned int i=0; i<labels.size(); i++)
      if(labels[i] == name) return true;
   return false;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// are two Namelists identical, ignoring a permutation?
bool operator==(const Namelist& N1, const Namelist& N2)
{
try {
   if(N1.size() != N2.size()) return false;
   if(N1.size() == 0) return true;
   for(unsigned int i=0; i<N1.size(); i++) {
      unsigned int match=0;
      for(unsigned int j=0; j<N2.size(); j++)
         if(N1.labels[i] == N2.labels[j]) match++;
      if(match != 1) return false;     // if > 1, N2 is invalid
   }
   return true;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// are two Namelists different, ignoring a permutation?
bool operator!=(const Namelist& N1, const Namelist& N2)
{
try {
   return !(N1==N2);
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// are two Namelists exactly identical, even considering permutations?
bool identical(const Namelist& N1, const Namelist& N2)
{
try {
   if(N1.size() != N2.size()) return false;
   if(N1.size() == 0) return true;
   for(unsigned int i=0; i<N1.size(); i++) {
      if(N1.labels[i] != N2.labels[i]) return false;
   }
   return true;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// construct the subset Namelist which is common to the two input (AND)
Namelist operator&(const Namelist& N1, const Namelist& N2)
{
try {
   Namelist N(N1);
   N &= N2;
   return N;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// merge two Namelists, i.e. construct a non-redundant combination (OR)
Namelist operator|(const Namelist& N1, const Namelist& N2)
{
try {
   Namelist N(N1);
   N |= N2;
   return N;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// construct the subset Namelist which is NOT common to two others (XOR)
Namelist operator^(const Namelist& N1, const Namelist& N2)
{
try {
   Namelist N(N1);
   N ^= N2;
   return N;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// replace this with (this & input) (AND - common to both)
Namelist& Namelist::operator&=(const Namelist& N)
{
try {
   Namelist NAND;
   for(unsigned int i=0; i<labels.size(); i++)
      if(N.contains(labels[i])) NAND += labels[i];
   *this = NAND;
   return *this;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// replace this with (this | input) (OR - merge - superset)
// NB new elements must be added at the end (for class SRI).
Namelist& Namelist::operator|=(const Namelist& N)
{
try {
   Namelist NOR(*this);
   for(unsigned int i=0; i<N.labels.size(); i++)
      if(!(this->contains(N.labels[i]))) NOR += N.labels[i];
   *this = NOR;
   return *this;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// replace this with (this ^ input) (XOR - not common)
Namelist& Namelist::operator^=(const Namelist& N)
{
try {
   unsigned int i;
   Namelist NXOR;
   for(i=0; i<labels.size(); i++)
      if(!(N.contains(labels[i]))) NXOR += labels[i];
   for(i=0; i<N.labels.size(); i++)
      if(!(this->contains(N.labels[i]))) NXOR += N.labels[i];
   *this = NXOR;
   return *this;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}


// access to a specific name, given its index; may be used as lvalue.
//string& Namelist::operator[](const unsigned int in)
//{
//   if(in >= labels.size()) throw ...
//   return labels[in];
//}

// access to a specific name, given its index.
// returns 'out-of-range' if the index is out of range.
string Namelist::getName(const unsigned int in) const
{
try {
   if(in >= labels.size()) return string("out-of-range");
   return labels[in];
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// assign a specific name, given its index;
// no effect if the index is out of range or the name is not unique.
// return true if successful
bool Namelist::setName(const unsigned int in, const string& name)
{
try {
   if(in >= labels.size()) return false;
   if(labels[in] == name) return true;    // NB b/c contains(name) would be true..
   if(contains(name)) return false;
   labels[in] = name;
   return true;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// return the index of the name in the list that matches the input, -1 if not found.
int Namelist::index(const string& name) const
{
try {
   for(unsigned int i=0; i<labels.size(); i++)
      if(labels[i] == name) return i;
   return -1;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

// output operator
ostream& operator<<(ostream& os, const Namelist& N)
{
try {
   if(N.labels.size() > 0) {
      for(unsigned int i=0; i<N.labels.size(); i++)
         os << " / " << N.labels[i];
      os << " / ";
   }
   return os;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

ostream& operator<<(ostream& os, const LabelledVector& nlp)
{
try {
   size_t i;
   int nlp_size;
#pragma unused(nlp_size)
   string s;
   //ofstream savefmt; 
   //savefmt.copyfmt(os);
   //int wid=os.width(),prec=os.precision();

   // print message or blanks
   os << nlp.tag << " ";
   if(nlp.msg.size() > 0)
      s = nlp.msg; //s = leftJustify(nlp.msg,nlp.wid);
   else
      s = rightJustify(string(""),nlp.msg.size()); //nlp.wid);
   os << s << " ";

   // print each label
   for(i=0; i<nlp.NL.size(); i++) {
      if(int(nlp.NL.getName(i).size()) > nlp.wid)
         s = leftJustify(nlp.NL.getName(i),nlp.wid);
      else
         s = rightJustify(nlp.NL.getName(i),nlp.wid);
      os << s;
      if(i-nlp.NL.size()+1) os << " ";
   }
   os << endl;       // next line

   // print same space as with labels
   s = rightJustify(string(""),nlp.msg.size()); //nlp.wid);
   os << nlp.tag << " " << s << " ";
   if(nlp.form == 1) os << fixed;
   if(nlp.form == 2) os << scientific;
   for(i=0; i<nlp.V.size(); i++) {
      //os.copyfmt(savefmt);
      //os << nlp.V(i);
      os << setw(nlp.wid) << setprecision(nlp.prec) << nlp.V(i);
      if(i-nlp.V.size()+1) os << " ";
   }

   return os;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

ostream& operator<<(ostream& os, const LabelledMatrix& nlp)
{
try {
   int nspace;
   size_t i, j, n;
   string s;
   const Namelist *pNLcol = &nlp.NLcols;
   const Namelist *pNLrow = &nlp.NLrows;

      // first make sure we have both namelists
   if(nlp.NLrows.size() == 0 && nlp.NLcols.size() == 0) {
      os << " Error -- Namelists in LabelledMatrix are empty! ";
      return os;
   }
   if(nlp.NLrows.size() == 0) pNLrow = pNLcol;
   if(nlp.NLcols.size() == 0) pNLcol = pNLrow;

      // on column labels line
   os << setw(0);
   if(nlp.rc == 0) {    // only if printing both column and row labels
      os << nlp.tag << " ";                                       // tag
      if(nlp.msg.size() > 0)                                      // msg
         s = nlp.msg;
      else
         s = rightJustify(string(""),nlp.wid);
      os << s << " ";
      if(int(nlp.msg.size()) > 0 && int(nlp.msg.size()) < nlp.wid)
         os << rightJustify(string(""),nlp.wid-nlp.msg.size());   // space
   }
      // print column labels
   if(nlp.rc != 1) { // but not if 'rows only'
      n = (nlp.M.cols() < pNLcol->size() ? nlp.M.cols() : pNLcol->size());
      for(i=0; i<n; i++) {
         if(int(pNLcol->getName(i).size()) > nlp.wid)
            s = leftJustify(pNLcol->getName(i),nlp.wid);
         else
            s = rightJustify(pNLcol->getName(i),nlp.wid);
         os << s;                                                 // label
         if(i-n+1) os << " ";
      }
      os << endl;
   }

   if(nlp.form == 1) os << fixed;
   if(nlp.form == 2) os << scientific;
   if(int(nlp.msg.size()) > nlp.wid) nspace = nlp.msg.size()-nlp.wid;
   else nspace = 0;

      // print one row per line
   for(i=0; i<nlp.M.rows(); i++) {
      os << nlp.tag << " ";                                       // tag
      if(nspace) os << rightJustify(string(""),nspace);           // space
         // print row labels
      if(nlp.rc != 2) { // but not if 'columns only'
         if( int(pNLrow->getName(i).size()) > nlp.wid)
            s = leftJustify(pNLrow->getName(i),nlp.wid);
         else
            s = rightJustify(pNLrow->getName(i),nlp.wid);
         os << s << " ";                                          // label
      }
         // finally, print the data
      for(j=0; j<nlp.M.cols(); j++) {
         os << setw(nlp.wid) << setprecision(nlp.prec) << nlp.M(i,j);
         if(j-nlp.M.cols()+1) os << " ";                          // data
      }
      if(i<nlp.M.rows()-1) os << endl;
   }

   return os;
}
catch(Exception& e) { GPSTK_RETHROW(e); }
}

} // end namespace gpstk

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
