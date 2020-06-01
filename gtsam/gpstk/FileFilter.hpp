#pragma ident "$Id$"



/**
 * @file FileFilter.hpp
 * A framework for sorting and filtering file data.
 */

#ifndef GPSTK_FILEFILTER_HPP
#define GPSTK_FILEFILTER_HPP

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






#include <functional>
#include <algorithm>
#include <iterator>

#include "FFData.hpp"
#include "FileSpec.hpp"

namespace gpstk
{
   /** @defgroup filedirgroup File and Directory Processing Utilities */
   //@{

      /**
       * This class is a framework for sorting and filtering file data.
       * It borrows several datatypes from FileSpec for cohesion of data
       * types.  Add the data to the filter, specify the parameters (date,
       * exclusion filters, etc.), then process it.  For the specific data
       * type, you can specify your own operators for sorting, uniqueness,
       * or almost anything else you want to do to the data.
       *
       * @warning The FFData class you're sorting MUST have a weak strict
       * ordering as defined in the FileFilterOperator.
       */

   template<class FileData>
   class FileFilter
   {
   public:
         /// Default constructor
      FileFilter(void);

         /// Destructor
      virtual ~FileFilter();

         /// Adds the given data into the filter.
      FileFilter& addData(const FileData& ffd);

         /// Adds arbitrary data to the filter.
      FileFilter& addData(const std::list<FileData>& datavec);

         /// Sorts the data.
         /// @warning bp MUST be a strict weak ordering!
      template <class Compare>
      FileFilter& sort(Compare comp)
     {
         // FIX: this someday...
         // this is a total hack until Solaris gets their act together and
         // gets list::sort() working again
         // all the code below can be replaced (someday) by this one line:
         //      dataVec.sort(comp);

         // make a vector of pointer to list iterator objects...
      std::vector<lItrType> data(dataVec.size());
      lItrType itr = dataVec.begin();
      typename std::vector<lItrType>::size_type i = 0;
      while (itr != dataVec.end())
      {
         data[i] = itr;
         i++;
         itr++;
      }
      
         // use SortAdapter to use comp with the pointer vector
         // then sort the vector
      SortAdapter<Compare> sa(comp);
      std::stable_sort(data.begin(), data.end(), sa);
      
         // make a new list of the data in the right order, then copy that
         // over dataVec.
      lType fdlist;
      for (i = 0; i < data.size(); i++)
      {
         fdlist.push_back(*data[i]);
      }
      dataVec = fdlist;
      
/*
         // move the items into the correct order with splice.
         // splice does nothing if (itr == data[i]) || (itr == ++data[i])
         // so the data is inserted backwards to avoid this...
      i = data.size();
      while (i != 0)
      {
         itr = dataVec.begin();
         --i;
         dataVec.splice(itr, dataVec, data[i]);
      }
*/            
      return *this;
   }


         /// Combines the data from the input filter to this object.
      FileFilter& merge(const FileFilter& right);

         /// Combines the data from the input filter to this object using
         /// the predicate to sort the data it merges.
         /// This should use list::merge(list, bp) but since it's broken in
         /// forte...
      template <class Compare>
      FileFilter& merge(const FileFilter& right, Compare bp)
         { merge(right); sort(bp); return *this; }

         /// After sorting, use this to filter the data.
         /// @warning The data must be sorted first
      template <class BinaryPredicate>
      FileFilter& unique(BinaryPredicate bp)
   {
         //  FIX: unique is broken or doesnt like my syntax
         //  so i wrote my own version of it
//      list<FileData>::iterator itr = 
//         unique(dataVec.begin(), dataVec.end(), bp);
      filtered = 0;
     
      typename std::list<FileData>::iterator first = dataVec.begin();
      typename std::list<FileData>::iterator second= dataVec.begin();
      second++;
      
         // keep only the first of many unique values
      while (second != dataVec.end())
      {
         if ( bp(*first, *second))
         {
            second = dataVec.erase(second);
            filtered++;
         }
         else
         {
            first++;
            second++;
         }
      }
      
      return *this;
   }

         /// This filters data based on a single test.  All data that
         /// passes the UnaryPredicate (i.e. it returns true) is removed
         /// @warning Depending on the filter, your data may need to be sorted
      template <class Predicate>
      FileFilter& filter(Predicate up)
   {
         // delete all values for which up() is true
      filtered = 0;
      
      typename std::list<FileData>::iterator itr = dataVec.begin();
      
      while (itr != dataVec.end())
      {
         if (up(*itr))
         {
            itr = dataVec.erase(itr);
            filtered++;
         }
         else
            itr++;
      }
      
      return *this;
   }

         /// Applies Operation on all the data elements, counting each one that
         /// gets modified (for which Operation returns true). The operation
         /// is passed by reference so that it can retain state information
         /// for use by the program calling it.
      template <class Operation>
      FileFilter& touch(Operation& op)
   {
      filtered = 0;
      
      typename std::list<FileData>::iterator itr = dataVec.begin();
      
      while (itr != dataVec.end())
      {
         if (op(*itr))
            filtered++;
         itr++;
      }
      
      return *this;
   }

         /// a const operator touch for the classes that need it.
      template <class Operation>
      FileFilter& touch(const Operation& op)
         { Operation o(op); return touch(o); }

         /// Returns two lists - one of the data in *this that isn't in r and
         /// the second of data in r that isn't in *this.  Remember that /a p
         /// has to be a strict weak ordering on the data.  
         /// @warning the input data needs to be sorted according to /a p 
         /// before running diff().  This also means that /a p is a strict
         /// weak ordering on the data (i.e. /a p sorts the data).
      template <class BinaryPredicate>
      std::pair< std::list<FileData>, std::list<FileData> > 
      diff(const FileFilter<FileData>& r, BinaryPredicate p) const
   {
      std::pair< std::list<FileData>, std::list<FileData> > toReturn;
      
      std::set_difference(dataVec.begin(), dataVec.end(),
                          r.dataVec.begin(), r.dataVec.end(),
                          std::inserter(toReturn.first, 
                                        toReturn.first.begin()),
                          p);
      
      std::set_difference(r.dataVec.begin(), r.dataVec.end(),
                          dataVec.begin(), dataVec.end(),
                          std::inserter(toReturn.second, 
                                        toReturn.second.begin()),
                          p);
      
      return toReturn;
   }

         /// Returns a list of data matching the given unary predicate.
      template <class Predicate>
      std::list<FileData> findAll(Predicate p) const
   {
      std::list<FileData> toReturn;
      typename std::list<FileData>::const_iterator itr = dataVec.begin();
      
      while (itr != dataVec.end())
      {
         if (p(*itr))
            toReturn.push_back((*itr));
         itr++;
      }
      
      return toReturn;
      
   }

         /// Returns the number of items filtered from the last filter()
         /// touch() or unique() call.
      int getFiltered() const {return filtered;}

         /// Returns the contents of the data list.
      std::list<FileData>& getData(void) {return dataVec;}

         /// Returns the contents of the data list, const.
      std::list<FileData> getData(void) const {return dataVec;}

         /// Returns the number of data items in the filter.
      typename std::list<FileData>::size_type getDataCount(void) const 
         { return dataVec.size(); }

      typename std::list<FileData>::const_iterator begin() const
         { return dataVec.begin(); }

      typename std::list<FileData>::const_iterator end() const
         { return dataVec.end(); }

      typename std::list<FileData>::iterator begin() 
         { return dataVec.begin(); }

      typename std::list<FileData>::iterator end() 
         { return dataVec.end(); }

      bool empty() const
         { return dataVec.empty(); }

      void clear()
         { dataVec.clear(); }

      typename std::list<FileData>::size_type size()
         { return dataVec.size(); }

      FileData& front()
         { return dataVec.front(); }

      const FileData& front() const
         { return dataVec.front(); }

      FileData& back()
         { return dataVec.back(); }

      const FileData& back() const
         { return dataVec.back(); }

   protected:
         /// List of file data to be filtered.
      typedef std::list<FileData> lType;
      lType dataVec;
      typedef typename std::list<FileData>::iterator lItrType;

         /// SortAdapter is an adapter class that takes any comparison
         /// function and instead uses list iterator objects instead
         /// of FileData.  This is only used by sort() and shouldn't be 
         /// used elsewhere.
      template<class Compare>
      class SortAdapter  : 
         public std::binary_function<lItrType, lItrType, bool>
      {
      public:
         SortAdapter(Compare& c)
               : comp(c)
            {}

         bool operator()(const lItrType l,
                         const lItrType r) const
            {
               return comp(*l, *r);
            }
      private:
         Compare comp;
      };

         /// A count of the last number of items filtered
      int filtered;
   };

   //@}

      template<class FileData>
   FileFilter<FileData> :: FileFilter(void)
         : filtered(0)
   {}

   template<class FileData>
   FileFilter<FileData> :: ~FileFilter()
   {
   }

   template<class FileData>
   FileFilter<FileData>& FileFilter<FileData> :: 
   addData(const FileData& ffd)
   {
      dataVec.push_back(ffd);
      return *this;
   }

   template <class FileData>
   FileFilter<FileData>& FileFilter<FileData> :: 
   addData(const std::list<FileData>& datavec)
   {
      std::copy(datavec.begin(), datavec.end(),
                std::inserter(dataVec, dataVec.begin()));
      return *this;
   }
   
   template <class FileData>
   FileFilter<FileData>&
   FileFilter<FileData> ::
   merge(const FileFilter<FileData>& right)
   {
         // cast out const to use the non-const version of getData()
      FileFilter<FileData>& r = (FileFilter<FileData>&)(right);
      
         // copy rightData into *this
      std::list<FileData>& rightData = r.getData();
      std::copy(rightData.begin(), rightData.end(),
                std::inserter(dataVec, dataVec.begin()));
      
      return *this;
   }

} // namespace gpstk

#endif // GPSTK_FILEFILTER_HPP
