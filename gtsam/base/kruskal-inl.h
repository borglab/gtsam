/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphBuilder-inl.h
 * @date Dec 31, 2009
 * @date Jan 23, 2023
 * @author Frank Dellaert, Yong-Dian Jian
 */

#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/base/types.h>
#include <gtsam/base/DSFVector.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/VariableIndex.h>

#include <memory>

#include <vector>

namespace gtsam::utils
{

    /*****************************************************************************/
    /* sort the container and return permutation index with default comparator */
    template <typename Container>
    static std::vector<size_t> sort_idx(const Container &src)
    {
        typedef typename Container::value_type T;
        const size_t n = src.size();
        std::vector<std::pair<size_t, T>> tmp;
        tmp.reserve(n);
        for (size_t i = 0; i < n; i++)
            tmp.emplace_back(i, src[i]);

        /* sort */
        std::stable_sort(tmp.begin(), tmp.end());

        /* copy back */
        std::vector<size_t> idx;
        idx.reserve(n);
        for (size_t i = 0; i < n; i++)
        {
            idx.push_back(tmp[i].first);
        }
        return idx;
    }

    /****************************************************************/
    template <class Graph>
    std::vector<size_t> kruskal(const Graph &fg,
                                const FastMap<Key, size_t> &ordering,
                                const std::vector<double> &weights)
    {
        const VariableIndex variableIndex(fg);
        const size_t n = variableIndex.size();
        const std::vector<size_t> sortedIndices = sort_idx(weights);

        /* initialize buffer */
        std::vector<size_t> treeIndices;
        treeIndices.reserve(n - 1);

        // container for acsendingly sorted edges
        DSFVector dsf(n);

        size_t count = 0;
        for (const size_t index : sortedIndices)
        {
            const auto &f = *fg[index];
            const auto keys = f.keys();
            if (keys.size() != 2)
                continue;
            const size_t u = ordering.find(keys[0])->second,
                         v = ordering.find(keys[1])->second;
            if (dsf.find(u) != dsf.find(v))
            {
                dsf.merge(u, v);
                treeIndices.push_back(index);
                if (++count == n - 1)
                    break;
            }
        }
        return treeIndices;
    }

} // namespace gtsam::utils
