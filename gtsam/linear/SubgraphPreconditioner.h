/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SubgraphPreconditioner.h
 * @date Dec 31, 2009
 * @author Frank Dellaert, Yong-Dian Jian
 */

#pragma once

#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/Preconditioner.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/types.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>

#include <boost/serialization/nvp.hpp>
#include <boost/shared_ptr.hpp>

#include <map>
#include <utility>
#include <vector>

namespace boost {
namespace serialization {
class access;
} /* namespace serialization */
} /* namespace boost */

namespace gtsam {

  // Forward declarations
  class GaussianBayesNet;
  class GaussianFactorGraph;
  class VectorValues;

  struct GTSAM_EXPORT SubgraphEdge {
    size_t index_;   /* edge id */
    double weight_; /* edge weight */
    SubgraphEdge() : index_(0), weight_(1.0) {}
    SubgraphEdge(const SubgraphEdge &e) : index_(e.index()), weight_(e.weight()) {}
    SubgraphEdge(const size_t index, const double weight = 1.0): index_(index), weight_(weight) {}
    inline size_t index() const { return index_; }
    inline double weight() const { return weight_; }
    inline bool isUnitWeight() const { return (weight_ == 1.0); }
    friend std::ostream &operator<<(std::ostream &os, const SubgraphEdge &edge);
  private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(index_);
      ar & BOOST_SERIALIZATION_NVP(weight_);
    }
  };

  /**************************************************************************/
  class GTSAM_EXPORT Subgraph {
  public:
    typedef boost::shared_ptr<Subgraph> shared_ptr;
    typedef std::vector<shared_ptr> vector_shared_ptr;
    typedef std::vector<SubgraphEdge> Edges;
    typedef std::vector<size_t> EdgeIndices;
    typedef Edges::iterator iterator;
    typedef Edges::const_iterator const_iterator;

  protected:
    Edges edges_;                 /* index to the factors */

  public:
    Subgraph() {}
    Subgraph(const Subgraph &subgraph) : edges_(subgraph.edges()) {}
    Subgraph(const Edges &edges) : edges_(edges) {}
    Subgraph(const std::vector<size_t> &indices) ;

    inline const Edges& edges() const { return edges_; }
    inline size_t size() const { return edges_.size(); }
    EdgeIndices edgeIndices() const;

    iterator begin() { return edges_.begin(); }
    const_iterator begin() const { return edges_.begin(); }
    iterator end() { return edges_.end(); }
    const_iterator end() const { return edges_.end(); }

    void save(const std::string &fn) const;
    static shared_ptr load(const std::string &fn);
    friend std::ostream &operator<<(std::ostream &os, const Subgraph &subgraph);

  private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(edges_);
    }
  };

  /****************************************************************************/
  struct GTSAM_EXPORT SubgraphBuilderParameters {
  public:
    typedef boost::shared_ptr<SubgraphBuilderParameters> shared_ptr;

    enum Skeleton {
      /* augmented tree */
      NATURALCHAIN = 0,  /* natural ordering of the graph */
      BFS,      /* breadth-first search tree */
      KRUSKAL,  /* maximum weighted spanning tree */
    } skeleton_ ;

    enum SkeletonWeight {   /* how to weigh the graph edges */
      EQUAL = 0,    /* every block edge has equal weight */
      RHS_2NORM,    /* use the 2-norm of the rhs */
      LHS_FNORM,    /* use the frobenius norm of the lhs */
      RANDOM,       /* bounded random edge weight */
    } skeletonWeight_ ;

    enum AugmentationWeight {   /* how to weigh the graph edges */
      SKELETON = 0,             /* use the same weights in building the skeleton */
//      STRETCH,                  /* stretch in the laplacian sense */
//      GENERALIZED_STRETCH       /* the generalized stretch defined in jian2013iros */
    } augmentationWeight_ ;

    double complexity_;

    SubgraphBuilderParameters()
      : skeleton_(KRUSKAL), skeletonWeight_(RANDOM), augmentationWeight_(SKELETON), complexity_(1.0) {}
    virtual ~SubgraphBuilderParameters() {}

    /* for serialization */
    void print() const ;
    virtual void print(std::ostream &os) const ;
    friend std::ostream& operator<<(std::ostream &os, const PreconditionerParameters &p);

    static Skeleton skeletonTranslator(const std::string &s);
    static std::string skeletonTranslator(Skeleton w);
    static SkeletonWeight skeletonWeightTranslator(const std::string &s);
    static std::string skeletonWeightTranslator(SkeletonWeight w);
    static AugmentationWeight augmentationWeightTranslator(const std::string &s);
    static std::string augmentationWeightTranslator(AugmentationWeight w);
  };

  /*****************************************************************************/
  class GTSAM_EXPORT SubgraphBuilder {

  public:
    typedef SubgraphBuilder Base;
    typedef boost::shared_ptr<SubgraphBuilder> shared_ptr;
    typedef std::vector<double> Weights;

    SubgraphBuilder(const SubgraphBuilderParameters &p = SubgraphBuilderParameters())
      : parameters_(p) {}
    virtual ~SubgraphBuilder() {}
    virtual boost::shared_ptr<Subgraph> operator() (const GaussianFactorGraph &jfg) const ;

  private:
    std::vector<size_t> buildTree(const GaussianFactorGraph &gfg, const FastMap<Key, size_t> &ordering, const std::vector<double> &weights) const ;
    std::vector<size_t> unary(const GaussianFactorGraph &gfg) const ;
    std::vector<size_t> natural_chain(const GaussianFactorGraph &gfg) const ;
    std::vector<size_t> bfs(const GaussianFactorGraph &gfg) const ;
    std::vector<size_t> kruskal(const GaussianFactorGraph &gfg, const FastMap<Key, size_t> &ordering, const std::vector<double> &w) const ;
    std::vector<size_t> sample(const std::vector<double> &weights, const size_t t) const ;
    Weights weights(const GaussianFactorGraph &gfg) const;
    SubgraphBuilderParameters parameters_;

  };

  /*******************************************************************************************/
  struct GTSAM_EXPORT SubgraphPreconditionerParameters : public PreconditionerParameters {
    typedef PreconditionerParameters Base;
    typedef boost::shared_ptr<SubgraphPreconditionerParameters> shared_ptr;
    SubgraphPreconditionerParameters(const SubgraphBuilderParameters &p = SubgraphBuilderParameters())
      : Base(), builderParams_(p) {}
    virtual ~SubgraphPreconditionerParameters() {}
    SubgraphBuilderParameters builderParams_;
  };

  /**
   * Subgraph conditioner class, as explained in the RSS 2010 submission.
   * Starting with a graph A*x=b, we split it in two systems A1*x=b1 and A2*x=b2
   * We solve R1*x=c1, and make the substitution y=R1*x-c1.
   * To use the class, give the Bayes Net R1*x=c1 and Graph A2*x=b2.
   * Then solve for yhat using CG, and solve for xhat = system.x(yhat).
   */
  class GTSAM_EXPORT SubgraphPreconditioner : public Preconditioner {

  public:
    typedef boost::shared_ptr<SubgraphPreconditioner> shared_ptr;
    typedef boost::shared_ptr<const GaussianBayesNet> sharedBayesNet;
    typedef boost::shared_ptr<const GaussianFactorGraph> sharedFG;
    typedef boost::shared_ptr<const VectorValues> sharedValues;
    typedef boost::shared_ptr<const Errors> sharedErrors;

  private:
    sharedFG Ab2_;
    sharedBayesNet Rc1_;
    sharedValues xbar_;  ///< A1 \ b1
    sharedErrors b2bar_; ///< A2*xbar - b2

    KeyInfo keyInfo_;
    SubgraphPreconditionerParameters parameters_;

  public:

    SubgraphPreconditioner(const SubgraphPreconditionerParameters &p = SubgraphPreconditionerParameters());

    /**
     * Constructor
     * @param Ab2: the Graph A2*x=b2
     * @param Rc1: the Bayes Net R1*x=c1
     * @param xbar: the solution to R1*x=c1
     */
    SubgraphPreconditioner(const sharedFG& Ab2, const sharedBayesNet& Rc1, const sharedValues& xbar,
                           const SubgraphPreconditionerParameters &p = SubgraphPreconditionerParameters());

    virtual ~SubgraphPreconditioner() {}

    /** print the object */
    void print(const std::string& s = "SubgraphPreconditioner") const;

    /** Access Ab2 */
    const sharedFG& Ab2() const { return Ab2_; }

    /** Access Rc1 */
    const sharedBayesNet& Rc1() const { return Rc1_; }

    /** Access b2bar */
    const sharedErrors b2bar() const { return b2bar_; }

    /**
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */

    /* x = xbar + inv(R1)*y */
    VectorValues x(const VectorValues& y) const;

    /* A zero VectorValues with the structure of xbar */
    VectorValues zero() const {
      VectorValues V(VectorValues::Zero(*xbar_));
      return V ;
    }

    /**
     * Add constraint part of the error only
     * y += alpha*inv(R1')*A2'*e2
     * Takes a range indicating e2 !!!!
     */
    void transposeMultiplyAdd2(double alpha, Errors::const_iterator begin,
        Errors::const_iterator end, VectorValues& y) const;

    /* error, given y */
    double error(const VectorValues& y) const;

    /** gradient = y + inv(R1')*A2'*(A2*inv(R1)*y-b2bar) */
    VectorValues gradient(const VectorValues& y) const;

    /** Apply operator A */
    Errors operator*(const VectorValues& y) const;

    /** Apply operator A in place: needs e allocated already */
    void multiplyInPlace(const VectorValues& y, Errors& e) const;

    /** Apply operator A' */
    VectorValues operator^(const Errors& e) const;

    /**
    * Add A'*e to y
    *  y += alpha*A'*[e1;e2] = [alpha*e1; alpha*inv(R1')*A2'*e2]
    */
    void transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& y) const;

    /*****************************************************************************/
    /* implement virtual functions of Preconditioner */

    /* Computation Interfaces for Vector */
    virtual void solve(const Vector& y, Vector &x) const;
    virtual void transposeSolve(const Vector& y, Vector& x) const ;

    virtual void build(
      const GaussianFactorGraph &gfg,
      const KeyInfo &info,
      const std::map<Key,Vector> &lambda
      ) ;
    /*****************************************************************************/
  };

  /* get subvectors */
  Vector getSubvector(const Vector &src, const KeyInfo &keyInfo, const FastVector<Key> &keys);

  /* set subvectors */
  void setSubvector(const Vector &src, const KeyInfo &keyInfo, const FastVector<Key> &keys, Vector &dst);


  /* build a factor subgraph, which is defined as a set of weighted edges (factors) */
  boost::shared_ptr<GaussianFactorGraph>
  buildFactorSubgraph(const GaussianFactorGraph &gfg, const Subgraph &subgraph, const bool clone);


  /* sort the container and return permutation index with default comparator */
   template <typename Container>
   std::vector<size_t> sort_idx(const Container &src)
   {
     typedef typename Container::value_type T;
     const size_t n = src.size() ;
     std::vector<std::pair<size_t,T> > tmp;
     tmp.reserve(n);
     for ( size_t i = 0 ; i < n ; i++ )
       tmp.push_back(std::make_pair(i, src[i]));

     /* sort */
     std::stable_sort(tmp.begin(), tmp.end()) ;

     /* copy back */
     std::vector<size_t> idx; idx.reserve(n);
     for ( size_t i = 0 ; i < n ; i++ ) {
       idx.push_back(tmp[i].first) ;
     }
     return idx;
   }

} // namespace gtsam
