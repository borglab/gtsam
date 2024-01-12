/*
 * NestedDissection.h
 *
 *   Created on: Nov 27, 2010
 *       Author: nikai
 *  Description: apply nested dissection algorithm to the factor graph
 */

#pragma once

#include <vector>
#include <memory>
#include <gtsam/nonlinear/Ordering.h>

namespace gtsam { namespace partition {

  /**
   *  Apply nested dissection algorithm to nonlinear factor graphs
   */
  template <class NLG, class SubNLG, class GenericGraph>
  class NestedDissection {
  public:
    typedef std::shared_ptr<SubNLG> sharedSubNLG;

  private:
    NLG fg_;                          // the original nonlinear factor graph
    Ordering ordering_;               // the variable ordering in the nonlinear factor graph
    std::vector<Symbol> int2symbol_;  // the mapping from integer key to symbol
    sharedSubNLG root_;               // the root of generated cluster tree

  public:
    sharedSubNLG root() const { return root_; }

  public:
    /* constructor with post-determined partitoning*/
    NestedDissection(const NLG& fg, const Ordering& ordering, const int numNodeStopPartition, const int minNodesPerMap, const bool verbose = false);

    /* constructor with pre-determined cuts*/
    NestedDissection(const NLG& fg, const Ordering& ordering, const std::shared_ptr<Cuts>& cuts, const bool verbose = false);

  private:
    /* convert generic subgraph to nonlinear subgraph  */
    sharedSubNLG makeSubNLG(const NLG& fg, const std::vector<size_t>& frontals,  const std::vector<size_t>& sep,  const std::shared_ptr<SubNLG>& parent) const;

    void processFactor(const typename GenericGraph::value_type& factor, const std::vector<int>& partitionTable,  // input
          std::vector<GenericGraph>& frontalFactors, NLG& sepFactors, std::vector<std::set<size_t> >& childSeps, // output factor graphs
          typename SubNLG::Weeklinks& weeklinks) const;

    /* recursively partition the generic graph */
    void partitionFactorsAndVariables(
        const GenericGraph& fg, const GenericUnaryGraph& unaryFactors,
        const std::vector<size_t>& keys, const std::vector<int>& partitionTable, const int numSubmaps,  // input
        std::vector<GenericGraph>& frontalFactors, vector<GenericUnaryGraph>& frontalUnaryFactors, NLG& sepFactors,   // output factor graphs
        std::vector<std::vector<size_t> >& childFrontals, std::vector<std::vector<size_t> >& childSeps, std::vector<size_t>& localFrontals,  // output sub-orderings
        typename SubNLG::Weeklinks& weeklinks) const;

    NLG collectOriginalFactors(const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors) const;

    /* recursively partition the generic graph */
    sharedSubNLG recursivePartition(const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors, const std::vector<size_t>& frontals, const std::vector<size_t>& sep,
          const int numNodeStopPartition, const int minNodesPerMap, const std::shared_ptr<SubNLG>& parent, WorkSpace& workspace, const bool verbose) const;

    /* recursively partition the generic graph */
    sharedSubNLG recursivePartition(const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors, const std::vector<size_t>& frontals, const std::vector<size_t>& sep,
        const std::shared_ptr<Cuts>& cuts, const std::shared_ptr<SubNLG>& parent, WorkSpace& workspace, const bool verbose) const;

  };

}} //namespace
