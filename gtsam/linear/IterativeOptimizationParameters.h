/**
 * @file IterativeOptimizationParameters.h
 * @date Oct 22, 2010
 * @author Yong-Dian Jian
 */

#pragma once

#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace gtsam {

/* parameters for combinatorial preconditioner */
struct CombinatorialParameters {

  enum Group {    /* group of variables */
    BLOCK = 0,    /* utilize the inherent block structure */
    SCALAR        /* break the block variables into scalars */
  } group_;

  enum Type {     /* subgraph specification */
    JACOBI = 0,   /* block diagonal */
    SPTREE,       /* spanning tree */
    GSP,          /* gspn, n = 0: all factors, n = 1: all unary factors, n = 2: spanning tree, etc .. */
    KOUTIS,       /* implement Koutis2011Focs */
  } type_;

  enum Weight {   /* how to weigh the graph edges */
    EQUAL = 0,    /* 0: every block edge has equal weight */
    RHS_2NORM,    /* 1: use the 2-norm of the rhs */
    LHS_FNORM,    /* 2: use the frobenius norm of the lhs */
    RAW,          /* 3: use raw scalar weight for the scalar case */
  } weight_ ;

  int complexity_;/* a parameter for the subgraph complexity */
  int hessian_;   /* 0: do nothing, 1: use whole block hessian */

  CombinatorialParameters(): group_(BLOCK), type_(JACOBI), weight_(EQUAL), complexity_(0), hessian_(1) {}
  CombinatorialParameters(Group group, Type type, Weight weight, int complexity, int hessian)
    : group_(group), type_(type), weight_(weight), complexity_(complexity), hessian_(hessian) {}

  inline Group group() const { return group_; }
  inline Type type() const { return type_; }
  inline Weight weight() const { return weight_; }
  inline int complexity() const { return complexity_; }
  inline int hessian() const { return hessian_; }

  inline bool isScalar() const { return group_ == SCALAR; }
  inline bool isBlock() const { return group_ == BLOCK; }
  inline bool isStatic() const { return (type_ == JACOBI || weight_ == EQUAL) ? true : false;}
  inline bool isDynamic() const { return !isStatic(); }

  inline void print() const {

    const std::string groupStr[2] = {"block", "scalar"};
    const std::string typeStr[4] = {"jacobi", "sptree", "gsp", "koutis"};
    const std::string weightStr[4] = {"equal", "rhs-2norm", "lhs-form", "raw"};

    std::cout << "CombinatorialParameters: "
    << "group = " << groupStr[group_]
    << ", type = " << typeStr[type_]
    << ", weight = " << weightStr[weight_]
    << ", complexity = " << complexity_
    << ", hessian = " << hessian_  << std::endl;
  }
};

/* parameters for the preconditioner */
struct PreconditionerParameters {

  typedef boost::shared_ptr<PreconditionerParameters> shared_ptr;

  enum Kernel { /* Preconditioner Kernel */
    GTSAM = 0,  /* Jacobian Factor Graph of GTSAM */
    CHOLMOD     /* Cholmod Sparse */
  } kernel_ ;

  enum Type {         /* preconditioner type */
    Combinatorial = 0 /* combinatorial preconditioner */
  } type_;

  CombinatorialParameters combinatorial_;

  enum Verbosity { SILENT = 0, COMPLEXITY = 1, ERROR = 2} verbosity_ ;  /* Verbosity */

  PreconditionerParameters(): kernel_(CHOLMOD), type_(Combinatorial), verbosity_(SILENT) {}
  PreconditionerParameters(Kernel kernel, const CombinatorialParameters &combinatorial, Verbosity verbosity)
    : kernel_(kernel), type_(Combinatorial), combinatorial_(combinatorial), verbosity_(verbosity) {}

  /* general interface */
  inline Kernel kernel() const { return kernel_; }
  inline Type type() const { return type_; }
  inline const CombinatorialParameters & combinatorial() const { return combinatorial_; }
  inline Verbosity verbosity() const { return verbosity_; }


  void print() const {
    const std::string kernelStr[2] = {"gtsam", "cholmod"};
    const std::string typeStr[1] = {"combinatorial"};

    std::cout << "PreconditionerParameters: "
              << "kernel = " << kernelStr[kernel_]
              << ", type = " << typeStr[type_] << std::endl;
    combinatorial_.print();
  }
};

/* parameters for the conjugate gradient method */
struct ConjugateGradientParameters {

  size_t minIterations_;  /* minimum number of cg iterations */
  size_t maxIterations_;  /* maximum number of cg iterations */
  size_t reset_;          /* number of iterations before reset */
  double epsilon_rel_;    /* threshold for relative error decrease */
  double epsilon_abs_;    /* threshold for absolute error decrease */

  enum BLASKernel {   /* Matrix Operation Kernel */
    GTSAM = 0,        /* Jacobian Factor Graph of GTSAM */
    SBM,              /* Sparse Block Matrix */
    SM,               /* Sparse Scalar Matrix */
    CHOLMOD           /* Cholmod Sparse */
  } blas_kernel_;

  size_t degree_;         /* the maximum degree of the vertices to be eliminated before doing cg */

  enum Verbosity { SILENT = 0, COMPLEXITY = 1, ERROR = 2} verbosity_ ;  /* Verbosity */

  ConjugateGradientParameters()
  : minIterations_(1), maxIterations_(500), reset_(501), epsilon_rel_(1e-3), epsilon_abs_(1e-3),
    blas_kernel_(GTSAM), degree_(0), verbosity_(SILENT) {}

  ConjugateGradientParameters(size_t minIterations, size_t maxIterations, size_t reset,
    double epsilon_rel, double epsilon_abs, BLASKernel blas = GTSAM, size_t degree = 0, Verbosity verbosity = SILENT)
    : minIterations_(minIterations), maxIterations_(maxIterations), reset_(reset),
      epsilon_rel_(epsilon_rel), epsilon_abs_(epsilon_abs), blas_kernel_(blas), degree_(degree), verbosity_(verbosity) {}

  /* general interface */
  inline size_t minIterations() const { return minIterations_; }
  inline size_t maxIterations() const { return maxIterations_; }
  inline size_t reset() const { return reset_; }
  inline double epsilon() const { return epsilon_rel_; }
  inline double epsilon_rel() const { return epsilon_rel_; }
  inline double epsilon_abs() const { return epsilon_abs_; }
  inline BLASKernel blas_kernel() const { return blas_kernel_; }
  inline size_t degree() const { return degree_; }
  inline Verbosity verbosity() const { return verbosity_; }

  void print() const {
    const std::string blasStr[4] = {"gtsam", "sbm", "sm", "cholmod"};
    std::cout << "ConjugateGradientParameters: "
              << "blas = " << blasStr[blas_kernel_]
              << ", minIter = " << minIterations_
              << ", maxIter = " << maxIterations_
              << ", resetIter = " << reset_
              << ", eps_rel = " << epsilon_rel_
              << ", eps_abs = " << epsilon_abs_
              << ", degree = " << degree_
              << std::endl;
  }
};

/* parameters for iterative linear solvers */
struct IterativeOptimizationParameters {

public:

  typedef boost::shared_ptr<IterativeOptimizationParameters> shared_ptr;

  PreconditionerParameters preconditioner_;
  ConjugateGradientParameters cg_;
  enum Kernel { PCG = 0 /*, PCGPlus*/, LSPCG } kernel_ ;                /* Iterative Method Kernel */
  enum Verbosity { SILENT = 0, COMPLEXITY = 1, ERROR = 2} verbosity_ ;  /* Verbosity */

public:

  IterativeOptimizationParameters()
  : preconditioner_(), cg_(), kernel_(PCG), verbosity_(SILENT) {}

  IterativeOptimizationParameters(const IterativeOptimizationParameters &p) :
    preconditioner_(p.preconditioner_), cg_(p.cg_), kernel_(p.kernel_), verbosity_(p.verbosity_) {}

  IterativeOptimizationParameters(
    const PreconditionerParameters &p, const ConjugateGradientParameters &c, Kernel kernel = PCG, Verbosity verbosity = SILENT)
    : preconditioner_(p), cg_(c), kernel_(kernel), verbosity_(verbosity) {}

  /* general interface */
  inline Kernel kernel() const { return kernel_; }
  inline Verbosity verbosity() const { return verbosity_; }

  /* utility */
  inline const PreconditionerParameters& preconditioner() const { return preconditioner_; }
  inline const ConjugateGradientParameters& cg() const { return cg_; }

  /* interface to preconditioner parameters */
  inline PreconditionerParameters::Kernel preconditioner_kernel() const { return preconditioner_.kernel(); }
  inline PreconditionerParameters::Type type() const { return preconditioner_.type(); }

  /* interface to cg parameters */
  inline size_t minIterations() const { return cg_.minIterations(); }
  inline size_t maxIterations() const { return cg_.maxIterations(); }
  inline size_t reset() const { return cg_.reset(); }
  inline double epsilon() const { return cg_.epsilon_rel(); }
  inline double epsilon_rel() const { return cg_.epsilon_rel(); }
  inline double epsilon_abs() const { return cg_.epsilon_abs(); }
  inline size_t degree() const { return cg_.degree(); }
  inline ConjugateGradientParameters::BLASKernel blas_kernel() const { return cg_.blas_kernel(); }

  void print(const std::string &s="") const {
    const std::string kernelStr[2] = {"pcg", "lspcg"};
    std::cout << s << std::endl
              << "IterativeOptimizationParameters: "
              << "kernel = " << kernelStr[kernel_] << std::endl;
    cg_.print();
    preconditioner_.print();

  }

};

}
