/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VectorMap.h
 * @brief   Factor Graph Valuesuration
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <map>
#include <boost/serialization/map.hpp>

#include <gtsam/base/types.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>

namespace gtsam {

  /** Factor Graph Valuesuration */
  class VectorMap : public Testable<VectorMap> {

  protected:
    /** Map from string indices to values */
    std::map<Index,Vector> values;

  public:
    typedef std::map<Index,Vector>::iterator iterator;
    typedef std::map<Index,Vector>::const_iterator const_iterator;

    VectorMap() {}
    VectorMap(const VectorMap& cfg_in): values(cfg_in.values) {}
    VectorMap(Index j, const Vector& a) { insert(j,a); }
    
    virtual ~VectorMap() {}

    /** return all the nodes in the graph **/
    std::vector<Index> get_names() const;

    /** convert into a single large vector */
    Vector vector() const;

    /** Insert a value into the values structure with a given index */
    VectorMap& insert(Index name, const Vector& v);

    /** Insert or add a value with given index */
    VectorMap& insertAdd(Index j, const Vector& v);

    /** Insert a config into another config */
    void insert(const VectorMap& config);

    /** Insert a config into another config, add if key already exists */
    void insertAdd(const VectorMap& config);

    const_iterator begin() const {return values.begin();}
    const_iterator end()   const {return values.end();}
    iterator begin() {return values.begin();}
    iterator end()   {return values.end();}

    /** Vector access in VectorMap is via a Vector reference */
    Vector& operator[](Index j);
    const Vector& operator[](Index j) const;

    /** [set] and [get] provided for access via MATLAB */
    inline Vector& get(Index j) { return (*this)[j];}
    void set(Index j, const Vector& v) { (*this)[j] = v; }
    inline const Vector& get(Index j) const { return (*this)[j];}

    bool contains(Index name) const {
      const_iterator it = values.find(name);
      return (it!=values.end());
    }

    /** Nr of vectors */
    size_t size() const { return values.size();}

    /** Total dimensionality */
    size_t dim() const;

    /** max of the vectors */
    inline double max() const {
      double m = -std::numeric_limits<double>::infinity();
      for(const_iterator it=begin(); it!=end(); it++)
        m = std::max(m, gtsam::max(it->second));
      return m;
    }

    /** Scale */
    VectorMap scale(double s) const;
    VectorMap operator*(double s) const;

    /** Negation */
    VectorMap operator-() const;

    /** Add in place */
    void operator+=(const VectorMap &b);

    /** Add */
    VectorMap operator+(const VectorMap &b) const;

    /** Subtract */
    VectorMap operator-(const VectorMap &b) const;

    /** print */
    void print(const std::string& name = "") const;

    /** equals, for unit testing */
    bool equals(const VectorMap& expected, double tol=1e-9) const;

    void clear() {values.clear();}

    /** Dot product */
    double dot(const VectorMap& b) const;

    /** Set all vectors to zero */
    VectorMap& zero();

    /** Create a clone of x with exactly same structure, except with zero values */
    static VectorMap zero(const VectorMap& x);

    /**
     * Add a delta config, needed for use in NonlinearOptimizer
     * For VectorMap, this is just addition.
     */
    friend VectorMap expmap(const VectorMap& original, const VectorMap& delta);

    /**
     * Add a delta vector (not a config)
     * Will use the ordering that map uses to loop over vectors
     */
    friend VectorMap expmap(const VectorMap& original, const Vector& delta);

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
      void serialize(Archive & ar, const unsigned int version)
    {
      ar & BOOST_SERIALIZATION_NVP(values);
    }
  }; // VectorMap

  /** scalar product */
  inline VectorMap operator*(double s, const VectorMap& x) {return x*s;}

  /** Dot product */
  double dot(const VectorMap&, const VectorMap&);

  /**
   * BLAS Level 1 scal: x <- alpha*x
   */
  void scal(double alpha, VectorMap& x);

  /**
   * BLAS Level 1 axpy: y <- alpha*x + y
   * UNSAFE !!!! Only works if x and y laid out in exactly same shape
   * Used in internal loop in iterative for fast conjugate gradients
   * Consider using other functions if this is not in hotspot
   */
  void axpy(double alpha, const VectorMap& x, VectorMap& y);

  /** dim function (for iterative::CGD) */
  inline double dim(const VectorMap& x) { return x.dim();}

  /** max of the vectors */
  inline double max(const VectorMap& x) { return x.max();}

  /** print with optional string */
  void print(const VectorMap& v, const std::string& s = "");

} // gtsam
