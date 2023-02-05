/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphBuilder.h
 * @date Dec 31, 2009
 * @author Frank Dellaert, Yong-Dian Jian
 */

#pragma once

#include <gtsam/base/FastMap.h>
#include <gtsam/base/types.h>
#include <gtsam/dllexport.h>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/version.hpp>
#include <boost/serialization/nvp.hpp>
#endif
#include <memory>

#include <vector>

namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */

namespace gtsam {

// Forward declarations
class GaussianFactorGraph;
struct PreconditionerParameters;

/**************************************************************************/
class GTSAM_EXPORT Subgraph {
 public:
  struct GTSAM_EXPORT Edge {
    size_t index;  /* edge id */
    double weight; /* edge weight */
    inline bool isUnitWeight() const { return (weight == 1.0); }
    friend std::ostream &operator<<(std::ostream &os, const Edge &edge);

   private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive &ar, const unsigned int /*version*/) {
      ar &BOOST_SERIALIZATION_NVP(index);
      ar &BOOST_SERIALIZATION_NVP(weight);
    }
#endif
  };

  typedef std::vector<Edge> Edges;
  typedef std::vector<size_t> EdgeIndices;
  typedef Edges::iterator iterator;
  typedef Edges::const_iterator const_iterator;

 protected:
  Edges edges_; /* index to the factors */

 public:
  Subgraph() {}
  Subgraph(const Subgraph &subgraph) : edges_(subgraph.edges()) {}
  Subgraph(const Edges &edges) : edges_(edges) {}
  Subgraph(const std::vector<size_t> &indices);

  inline const Edges &edges() const { return edges_; }
  inline size_t size() const { return edges_.size(); }
  EdgeIndices edgeIndices() const;

  iterator begin() { return edges_.begin(); }
  const_iterator begin() const { return edges_.begin(); }
  iterator end() { return edges_.end(); }
  const_iterator end() const { return edges_.end(); }

  friend std::ostream &operator<<(std::ostream &os, const Subgraph &subgraph);

 private:
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_NVP(edges_);
  }
  void save(const std::string &fn) const;
  static Subgraph load(const std::string &fn);
#endif
};

/****************************************************************************/
struct GTSAM_EXPORT SubgraphBuilderParameters {
  typedef std::shared_ptr<SubgraphBuilderParameters> shared_ptr;

  enum Skeleton {
    /* augmented tree */
    NATURALCHAIN = 0, /* natural ordering of the graph */
    BFS,              /* breadth-first search tree */
    KRUSKAL,          /* maximum weighted spanning tree */
  } skeletonType;

  enum SkeletonWeight {            /* how to weigh the graph edges */
                        EQUAL = 0, /* every block edge has equal weight */
                        RHS_2NORM, /* use the 2-norm of the rhs */
                        LHS_FNORM, /* use the frobenius norm of the lhs */
                        RANDOM,    /* bounded random edge weight */
  } skeletonWeight;

  enum AugmentationWeight {               /* how to weigh the graph edges */
                            SKELETON = 0, /* use the same weights in building
                                             the skeleton */
                            //      STRETCH,                /* stretch in the
                            //      laplacian sense */ GENERALIZED_STRETCH /*
                            //      the generalized stretch defined in
                            //      jian2013iros */
  } augmentationWeight;

  /// factor multiplied with n, yields number of extra edges.
  double augmentationFactor; 

  SubgraphBuilderParameters()
      : skeletonType(KRUSKAL),
        skeletonWeight(RANDOM),
        augmentationWeight(SKELETON),
        augmentationFactor(1.0) {}
  virtual ~SubgraphBuilderParameters() {}

  /* for serialization */
  void print() const;
  virtual void print(std::ostream &os) const;
  friend std::ostream &operator<<(std::ostream &os,
                                  const PreconditionerParameters &p);

  static Skeleton skeletonTranslator(const std::string &s);
  static std::string skeletonTranslator(Skeleton s);
  static SkeletonWeight skeletonWeightTranslator(const std::string &s);
  static std::string skeletonWeightTranslator(SkeletonWeight w);
  static AugmentationWeight augmentationWeightTranslator(const std::string &s);
  static std::string augmentationWeightTranslator(AugmentationWeight w);
};

/*****************************************************************************/
class GTSAM_EXPORT SubgraphBuilder {
 public:
  typedef SubgraphBuilder Base;
  typedef std::vector<double> Weights;

  SubgraphBuilder(
      const SubgraphBuilderParameters &p = SubgraphBuilderParameters())
      : parameters_(p) {}
  virtual ~SubgraphBuilder() {}
  virtual Subgraph operator()(const GaussianFactorGraph &jfg) const;

 private:
  std::vector<size_t> buildTree(const GaussianFactorGraph &gfg,
                                const std::vector<double> &weights) const;
  std::vector<size_t> unary(const GaussianFactorGraph &gfg) const;
  std::vector<size_t> natural_chain(const GaussianFactorGraph &gfg) const;
  std::vector<size_t> bfs(const GaussianFactorGraph &gfg) const;
  std::vector<size_t> kruskal(const GaussianFactorGraph &gfg,
                              const std::vector<double> &weights) const;
  std::vector<size_t> sample(const std::vector<double> &weights,
                             const size_t t) const;
  Weights weights(const GaussianFactorGraph &gfg) const;
  SubgraphBuilderParameters parameters_;
};

/** Select the factors in a factor graph according to the subgraph. */
GaussianFactorGraph buildFactorSubgraph(const GaussianFactorGraph &gfg,
                                        const Subgraph &subgraph,
                                        const bool clone);

/** Split the graph into a subgraph and the remaining edges. 
 * Note that the remaining factorgraph has null factors. */
std::pair<GaussianFactorGraph, GaussianFactorGraph> splitFactorGraph(
    const GaussianFactorGraph &factorGraph, const Subgraph &subgraph);

}  // namespace gtsam