/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    statistics.h
* @brief   Tools for gathering statistics about a forest to aid tuning parallel traversal
* @author  Richard Roberts
* @date    April 9, 2013
*/
#pragma once

#include <gtsam/global_includes.h>
#include <gtsam/base/FastMap.h>

#include <ostream>

namespace gtsam {

  namespace treeTraversal {

    /* ************************************************************************* */
    /// Struct to store gathered statistics about a forest
    struct ForestStatistics
    {
      typedef FastMap<int, ValueWithDefault<int, 0> > Histogram;
      Histogram problemSizeHistogram;
      Histogram numberOfChildrenHistogram;
      Histogram problemSizeOfSecondLargestChildHistogram;

      static void Write(std::ostream& outStream, const Histogram& histogram)
      {
        if (!histogram.empty())
        {
          Histogram::const_iterator endIt = histogram.end();
          -- endIt;
          const int largest = endIt->first;
          for (int bin = 0; bin <= largest; ++bin)
          {
            Histogram::const_iterator item = histogram.find(bin);
            const int count = (item == histogram.end() ? 0 : *item->second);
            outStream << bin << " " << count << "\n";
          }
        }
      }
    };

    /* ************************************************************************* */
    namespace internal {
      template<class NODE>
      ForestStatistics* statisticsVisitor(const boost::shared_ptr<NODE>& node, ForestStatistics* stats)
      {
        (*stats->problemSizeHistogram[node->problemSize()]) ++;
        (*stats->numberOfChildrenHistogram[(int)node->children.size()]) ++;
        if (node->children.size() > 1)
        {
          int largestProblemSize = 0;
          int secondLargestProblemSize = 0;
          for(const boost::shared_ptr<NODE>& child: node->children)
          {
            if (child->problemSize() > largestProblemSize)
            {
              secondLargestProblemSize = largestProblemSize;
              largestProblemSize = child->problemSize();
            }
            else if (child->problemSize() > secondLargestProblemSize)
            {
              secondLargestProblemSize = child->problemSize();
            }
          }
          (*stats->problemSizeOfSecondLargestChildHistogram[secondLargestProblemSize]) ++;
        }
        return stats;
      }
    }

    /* ************************************************************************* */
    template<class FOREST>
    ForestStatistics GatherStatistics(const FOREST& forest)
    {
      ForestStatistics stats;
      ForestStatistics* statsPtr = &stats;
      DepthFirstForest(forest, statsPtr, internal::statisticsVisitor<typename FOREST::Node>);
      return stats;
    }

  }
}
