/*
 * NestedDissection-inl.h
 *
 *   Created on: Nov 27, 2010
 *       Author: nikai
 *  Description:
 */

#pragma once


#include "partition/FindSeparator-inl.h"
#include "OrderedSymbols.h"
#include "NestedDissection.h"

namespace gtsam { namespace partition {

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  NestedDissection<NLG, SubNLG, GenericGraph>::NestedDissection(
      const NLG& fg, const Ordering& ordering, const int numNodeStopPartition, const int minNodesPerMap, const bool verbose) :
  fg_(fg), ordering_(ordering){
    GenericUnaryGraph unaryFactors;
    GenericGraph gfg;
    boost::tie(unaryFactors, gfg) = fg.createGenericGraph(ordering);

    // build reverse mapping from integer to symbol
    int numNodes = ordering.size();
    int2symbol_.resize(numNodes);
    Ordering::const_iterator it = ordering.begin(), itLast = ordering.end();
    while(it != itLast)
      int2symbol_[it->second] = (it++)->first;

    vector<size_t> keys;
    keys.reserve(numNodes);
    for(int i=0; i<ordering.size(); ++i)
      keys.push_back(i);

    WorkSpace workspace(numNodes);
    root_ = recursivePartition(gfg, unaryFactors, keys, vector<size_t>(), numNodeStopPartition, minNodesPerMap, std::shared_ptr<SubNLG>(), workspace, verbose);
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  NestedDissection<NLG, SubNLG, GenericGraph>::NestedDissection(
      const NLG& fg, const Ordering& ordering, const std::shared_ptr<Cuts>& cuts, const bool verbose) : fg_(fg), ordering_(ordering){
    GenericUnaryGraph unaryFactors;
    GenericGraph gfg;
    boost::tie(unaryFactors, gfg) = fg.createGenericGraph(ordering);

    // build reverse mapping from integer to symbol
    int numNodes = ordering.size();
    int2symbol_.resize(numNodes);
    Ordering::const_iterator it = ordering.begin(), itLast = ordering.end();
    while(it != itLast)
      int2symbol_[it->second] = (it++)->first;

    vector<size_t> keys;
    keys.reserve(numNodes);
    for(int i=0; i<ordering.size(); ++i)
      keys.push_back(i);

    WorkSpace workspace(numNodes);
    root_ = recursivePartition(gfg, unaryFactors, keys, vector<size_t>(), cuts, std::shared_ptr<SubNLG>(), workspace, verbose);
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  std::shared_ptr<SubNLG> NestedDissection<NLG, SubNLG, GenericGraph>::makeSubNLG(
      const NLG& fg, const vector<size_t>& frontals, const vector<size_t>& sep, const std::shared_ptr<SubNLG>& parent) const {
    OrderedSymbols frontalKeys;
    for(const size_t index: frontals)
      frontalKeys.push_back(int2symbol_[index]);

    UnorderedSymbols sepKeys;
    for(const size_t index: sep)
      sepKeys.insert(int2symbol_[index]);

    return std::make_shared<SubNLG>(fg, frontalKeys, sepKeys, parent);
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  void NestedDissection<NLG, SubNLG, GenericGraph>::processFactor(
      const typename GenericGraph::value_type& factor, const std::vector<int>& partitionTable,  // input
      vector<GenericGraph>& frontalFactors, NLG& sepFactors, vector<set<size_t> >& childSeps, // output factor graphs
      typename SubNLG::Weeklinks& weeklinks) const {                                                              // the links between child cliques
    list<size_t> sep_; // the separator variables involved in the current factor
    int partition1 = partitionTable[factor->key1.index];
    int partition2 = partitionTable[factor->key2.index];
    if (partition1 <= 0 && partition2 <= 0) {                                // is a factor in the current clique
      sepFactors.push_back(fg_[factor->index]);
    }
    else if (partition1 > 0 && partition2 > 0 && partition1 != partition2) {  // is a weeklink (factor between two child cliques)
      weeklinks.push_back(fg_[factor->index]);
    }
    else if (partition1 > 0 && partition2 > 0 && partition1 == partition2) { // is a local factor in one of the child cliques
      frontalFactors[partition1 - 1].push_back(factor);
    }
    else {                                                          // is a joint factor in the child clique (involving varaibles in the current clique)
      if (partition1 > 0 && partition2 <= 0) {
        frontalFactors[partition1 - 1].push_back(factor);
        childSeps[partition1 - 1].insert(factor->key2.index);
      } else if (partition1 <= 0 && partition2 > 0) {
        frontalFactors[partition2 - 1].push_back(factor);
        childSeps[partition2 - 1].insert(factor->key1.index);
      } else
        throw runtime_error("processFactor: unexpected entries in the partition table!");
    }
  }

  /* ************************************************************************* */
  /**
   * given a factor graph and its partition {nodeMap}, split the factors between the child cliques ({frontalFactors})
   *  and the current clique ({sepFactors}). Also split the variables between the child cliques ({childFrontals})
   *  and the current clique ({localFrontals}). Those separator variables involved in {frontalFactors} are put into
   *  the correspoding ordering in {childSeps}.
   */
  // TODO: frontalFactors and localFrontals should be generated in findSeparator
  template <class NLG, class SubNLG, class GenericGraph>
  void NestedDissection<NLG, SubNLG, GenericGraph>::partitionFactorsAndVariables(
      const GenericGraph& fg, const GenericUnaryGraph& unaryFactors, const std::vector<size_t>& keys, //input
      const std::vector<int>& partitionTable, const int numSubmaps,                                   // input
      vector<GenericGraph>& frontalFactors, vector<GenericUnaryGraph>& frontalUnaryFactors,  NLG& sepFactors,     // output factor graphs
      vector<vector<size_t> >& childFrontals, vector<vector<size_t> >& childSeps, vector<size_t>& localFrontals,  // output sub-orderings
      typename SubNLG::Weeklinks& weeklinks) const {                                                             // the links between child cliques

    // make three lists of variables A, B, and C
    int partition;
    childFrontals.resize(numSubmaps);
    for(const size_t key: keys){
      partition = partitionTable[key];
      switch (partition) {
      case -1: break;                                        // the separator of the separator variables
      case 0:   localFrontals.push_back(key); break;          // the separator variables
      default: childFrontals[partition-1].push_back(key);    // the frontal variables
      }
    }

    // group the factors to {frontalFactors} and {sepFactors},and find the joint variables
    vector<set<size_t> > childSeps_;
    childSeps_.resize(numSubmaps);
    childSeps.reserve(numSubmaps);
    frontalFactors.resize(numSubmaps);
    frontalUnaryFactors.resize(numSubmaps);
    for(typename GenericGraph::value_type factor: fg)
      processFactor(factor, partitionTable, frontalFactors, sepFactors, childSeps_, weeklinks);
    for(const set<size_t>& childSep: childSeps_)
      childSeps.push_back(vector<size_t>(childSep.begin(), childSep.end()));

    // add unary factor to the current cluster or pass it to one of the child clusters
    for(const sharedGenericUnaryFactor& unaryFactor_: unaryFactors) {
      partition = partitionTable[unaryFactor_->key.index];
      if (!partition) sepFactors.push_back(fg_[unaryFactor_->index]);
      else frontalUnaryFactors[partition-1].push_back(unaryFactor_);
    }
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  NLG NestedDissection<NLG, SubNLG, GenericGraph>::collectOriginalFactors(
      const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors) const {
    NLG sepFactors;
    typename GenericGraph::const_iterator it = gfg.begin(), itLast = gfg.end();
    while(it!=itLast) sepFactors.push_back(fg_[(*it++)->index]);
    for(const sharedGenericUnaryFactor& unaryFactor_: unaryFactors)
      sepFactors.push_back(fg_[unaryFactor_->index]);
    return sepFactors;
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  std::shared_ptr<SubNLG> NestedDissection<NLG, SubNLG, GenericGraph>::recursivePartition(
      const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors, const vector<size_t>& frontals, const vector<size_t>& sep,
      const int numNodeStopPartition, const int minNodesPerMap, const std::shared_ptr<SubNLG>& parent, WorkSpace& workspace, const bool verbose) const {

    // if no split needed
    NLG sepFactors; // factors that should remain in the current cluster
    if (frontals.size() <= numNodeStopPartition || gfg.size() <= numNodeStopPartition) {
      sepFactors = collectOriginalFactors(gfg, unaryFactors);
      return makeSubNLG(sepFactors, frontals, sep, parent);
    }

    // find the nested dissection separator
    int numSubmaps = findSeparator(gfg, frontals, minNodesPerMap, workspace, verbose, int2symbol_, NLG::reduceGraph(),
        NLG::minNrConstraintsPerCamera(),NLG::minNrConstraintsPerLandmark());
    partition::PartitionTable& partitionTable = workspace.partitionTable;
    if (numSubmaps == 0) throw runtime_error("recursivePartition: get zero submap after ND!");

    // split the factors between child cliques and the current clique
    vector<GenericGraph> frontalFactors; vector<GenericUnaryGraph> frontalUnaryFactors; typename SubNLG::Weeklinks weeklinks;
    vector<size_t> localFrontals; vector<vector<size_t> > childFrontals, childSeps;
    partitionFactorsAndVariables(gfg, unaryFactors, frontals, partitionTable, numSubmaps,
        frontalFactors, frontalUnaryFactors, sepFactors, childFrontals, childSeps, localFrontals, weeklinks);

    // make a new cluster
    std::shared_ptr<SubNLG> current = makeSubNLG(sepFactors, localFrontals, sep, parent);
    current->setWeeklinks(weeklinks);

    // check whether all the submaps are fully constrained
    for (int i=0; i<numSubmaps; i++) {
      checkSingularity(frontalFactors[i], childFrontals[i], workspace, NLG::minNrConstraintsPerCamera(),NLG::minNrConstraintsPerLandmark());
    }

    // create child clusters
    for (int i=0; i<numSubmaps; i++) {
      std::shared_ptr<SubNLG> child = recursivePartition(frontalFactors[i], frontalUnaryFactors[i], childFrontals[i], childSeps[i],
          numNodeStopPartition, minNodesPerMap, current, workspace, verbose);
      current->addChild(child);
    }

    return current;
  }

  /* ************************************************************************* */
  template <class NLG, class SubNLG, class GenericGraph>
  std::shared_ptr<SubNLG> NestedDissection<NLG, SubNLG, GenericGraph>::recursivePartition(
      const GenericGraph& gfg, const GenericUnaryGraph& unaryFactors, const vector<size_t>& frontals, const vector<size_t>& sep,
      const std::shared_ptr<Cuts>& cuts, const std::shared_ptr<SubNLG>& parent, WorkSpace& workspace, const bool verbose) const {

    // if there is no need to cut any more
    NLG sepFactors; // factors that should remain in the current cluster
    if (!cuts.get()) {
      sepFactors = collectOriginalFactors(gfg, unaryFactors);
      return makeSubNLG(sepFactors, frontals, sep, parent);
    }

    // retrieve the current partitioning info
    int numSubmaps = 2;
    partition::PartitionTable& partitionTable = cuts->partitionTable;

    // split the factors between child cliques and the current clique
    vector<GenericGraph> frontalFactors; vector<GenericUnaryGraph> frontalUnaryFactors; typename SubNLG::Weeklinks weeklinks;
    vector<size_t> localFrontals; vector<vector<size_t> > childFrontals, childSeps;
    partitionFactorsAndVariables(gfg, unaryFactors, frontals, partitionTable, numSubmaps,
        frontalFactors, frontalUnaryFactors, sepFactors, childFrontals, childSeps, localFrontals, weeklinks);

    // make a new cluster
    std::shared_ptr<SubNLG> current = makeSubNLG(sepFactors, localFrontals, sep, parent);
    current->setWeeklinks(weeklinks);

    // create child clusters
    for (int i=0; i<2; i++) {
      std::shared_ptr<SubNLG> child = recursivePartition(frontalFactors[i], frontalUnaryFactors[i], childFrontals[i], childSeps[i],
          cuts->children.empty() ? std::shared_ptr<Cuts>() : cuts->children[i], current, workspace, verbose);
      current->addChild(child);
    }
    return current;
  }
}} //namespace
