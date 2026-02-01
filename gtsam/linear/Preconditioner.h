/*
 * Preconditioner.h
 *
 *  Created on: Jun 2, 2014
 *      Author: Yong-Dian Jian
 *      Author: Sungtae An
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <memory>
#include <iosfwd>
#include <map>
#include <string>

namespace gtsam {

class GaussianFactorGraph;
class KeyInfo;
class VectorValues;

/* parameters for the preconditioner */
struct GTSAM_EXPORT PreconditionerParameters {

   typedef std::shared_ptr<PreconditionerParameters> shared_ptr;

   enum Kernel { /* Preconditioner Kernel */
     GTSAM = 0,
     CHOLMOD     /* experimental */
   } kernel_ ;

   enum Verbosity {
     SILENT = 0,
     COMPLEXITY = 1,
     ERROR = 2
   } verbosity_ ;

   PreconditionerParameters(): kernel_(GTSAM), verbosity_(SILENT) {}
   PreconditionerParameters(const PreconditionerParameters &p) : kernel_(p.kernel_), verbosity_(p.verbosity_) {}
   virtual ~PreconditionerParameters() {}

   /* general interface */
   inline Kernel kernel() const { return kernel_; }
   inline Verbosity verbosity() const { return verbosity_; }

   void print() const;

   virtual void print(std::ostream &os) const;

   static Kernel kernelTranslator(const std::string &s);
   static Verbosity verbosityTranslator(const std::string &s);
   static std::string kernelTranslator(Kernel k);
   static std::string verbosityTranslator(Verbosity v);

   /* for serialization */
   friend std::ostream& operator<<(std::ostream &os, const PreconditionerParameters &p);
 };

/* PCG aims to solve the problem: A x = b by reparametrizing it as
 * L^{-1} A L^{-T} y = L^{-1} b   or   M^{-1} A x = M^{-1} b,
 * where A \approx L L^{T}, or A \approx M
 * The goal of this class is to provide a general interface to all preconditioners */
class GTSAM_EXPORT Preconditioner {
public:
  typedef std::shared_ptr<Preconditioner> shared_ptr;
  typedef std::vector<size_t> Dimensions;

  /* Generic Constructor and Destructor */
  Preconditioner() {}
  virtual ~Preconditioner() {}

  /* 
  * Abstract interface for raw vectors. VectorValues is a speed bottleneck
  * and Yong-Dian has profiled preconditioners (outside GTSAM) with the the
  * three methods below. In GTSAM, unfortunately, we are still using the
  * VectorValues methods called in iterative-inl.h
  */

  /// implement x = L^{-1} y
  virtual void solve(const Vector& y, Vector &x) const = 0;

  /// implement x = L^{-T} y
  virtual void transposeSolve(const Vector& y, Vector& x) const = 0;

  /// build/factorize the preconditioner
  virtual void build(
    const GaussianFactorGraph &gfg,
    const KeyInfo &info,
    const std::map<Key,Vector> &lambda
    ) = 0;
};

/*******************************************************************************************/
struct GTSAM_EXPORT DummyPreconditionerParameters : public PreconditionerParameters {
  typedef PreconditionerParameters Base;
  typedef std::shared_ptr<DummyPreconditionerParameters> shared_ptr;
  DummyPreconditionerParameters() : Base() {}
  ~DummyPreconditionerParameters() override {}
};

/*******************************************************************************************/
class GTSAM_EXPORT DummyPreconditioner : public Preconditioner {
public:
  typedef Preconditioner Base;
  typedef std::shared_ptr<DummyPreconditioner> shared_ptr;

public:

  DummyPreconditioner() : Base() {}
  ~DummyPreconditioner() override {}

  /* Computation Interfaces for raw vector */
  void solve(const Vector& y, Vector &x) const override { x = y; }
  void transposeSolve(const Vector& y, Vector& x) const  override { x = y; }
  void build(
    const GaussianFactorGraph &gfg,
    const KeyInfo &info,
    const std::map<Key,Vector> &lambda
    ) override {}
};

/*******************************************************************************************/
struct GTSAM_EXPORT BlockJacobiPreconditionerParameters : public PreconditionerParameters {
  typedef PreconditionerParameters Base;
  BlockJacobiPreconditionerParameters() : Base() {}
  ~BlockJacobiPreconditionerParameters() override {}
};

/*******************************************************************************************/
class GTSAM_EXPORT BlockJacobiPreconditioner : public Preconditioner {
public:
  typedef Preconditioner Base;
  BlockJacobiPreconditioner() ;
  ~BlockJacobiPreconditioner() override ;

  /* Computation Interfaces for raw vector */
  void solve(const Vector& y, Vector &x) const override;
  void transposeSolve(const Vector& y, Vector& x) const override;
  void build(
    const GaussianFactorGraph &gfg,
    const KeyInfo &info,
    const std::map<Key,Vector> &lambda
    ) override;

protected:

  void clean() ;

  std::vector<size_t> dims_;
  double *buffer_;
  size_t bufferSize_;
  size_t nnz_;
};

/*********************************************************************************************/
/* factory method to create preconditioners */
std::shared_ptr<Preconditioner> createPreconditioner(const std::shared_ptr<PreconditionerParameters> parameters);

}


