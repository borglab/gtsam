/**
 * @file    VectorMap.h
 * @brief   Factor Graph Configuration
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

// \callgraph

#pragma once

#include <map>
#include <boost/serialization/map.hpp>

#include "Testable.h"
#include "Vector.h"
#include "Key.h"
#include "SymbolMap.h"

namespace gtsam {
	
  /** Factor Graph Configuration */
  class VectorMap : public Testable<VectorMap> {

  protected:
    /** Map from string indices to values */
    SymbolMap<Vector> values;

  public:
    typedef SymbolMap<Vector>::iterator iterator;
    typedef SymbolMap<Vector>::const_iterator const_iterator;

    VectorMap() {}
    VectorMap(const VectorMap& cfg_in): values(cfg_in.values) {}
    VectorMap(const Symbol& j, const Vector& a) { insert(j,a); }
    
    virtual ~VectorMap() {}

    /** return all the nodes in the graph **/
    std::vector<Symbol> get_names() const;

    /** Insert a value into the configuration with a given index */
    VectorMap& insert(const Symbol& name, const Vector& v);

		/** Insert or add a value with given index */
    VectorMap& insertAdd(const Symbol& j, const Vector& v);

    /** Insert a config into another config */
    void insert(const VectorMap& config);

		/** Insert a config into another config, add if key already exists */
		void insertAdd(const VectorMap& config);

    const_iterator begin() const {return values.begin();}
    const_iterator end()   const {return values.end();}
    iterator begin() {return values.begin();}
    iterator end()   {return values.end();}

		/** Vector access in VectorMap is via a Vector reference */
		Vector& operator[](const Symbol& j);
		const Vector& operator[](const Symbol& j) const;

		/** [set] and [get] provided for access via MATLAB */
		inline Vector& get(const Symbol& j) { return (*this)[j];}
		void set(const Symbol& j, const Vector& v) { (*this)[j] = v; }
		inline const Vector& get(const Symbol& j) const { return (*this)[j];}

		bool contains(const Symbol& name) const {
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
