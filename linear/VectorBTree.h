/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VectorBTree.h
 * @brief   Factor Graph Valuesuration
 * @author Frank Dellaert
 */

// \callgraph

#pragma once

#include <map>
#include <boost/numeric/ublas/storage.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/base/BTree.h>

namespace gtsam {

	/** Factor Graph Valuesuration */
	class VectorBTree: public Testable<VectorBTree> {

	private:

		/** dictionary from Symbol to Range */
		typedef boost::numeric::ublas::range Range;
		typedef BTree<Symbol, Range> Ranges;
		typedef Ranges::value_type Pair;
		Ranges ranges_;

		/** Actual vector */
		Vector values_;

		/** size_ is number of vectors */
		size_t size_;

		/** private get from symbol pair */
		Vector get(const Range& r) const {
			return sub(values_,r.start(),r.start()+r.size());
		}

public:

		/**
		 * Default constructor
		 */
		VectorBTree() :
			size_(0) {
		}

		/**
		 * Copy constructor
		 */
		VectorBTree(const VectorBTree& c) :
			ranges_(c.ranges_), values_(c.values_), size_(c.size_) {
		}

		/**
		 * Construct with a single vector
		 */
		VectorBTree(const Symbol& j, const Vector& a) :
			size_(0) {
			insert(j, a);
		}

		virtual ~VectorBTree() {
		}

		/** print */
		void print(const std::string& name = "") const;

		/** equals, for unit testing */
		bool equals(const VectorBTree& expected, double tol = 1e-9) const;

		/** Insert a value into the values structure with a given index: O(n) */
		VectorBTree& insert(const Symbol& j, const Vector& v);

		/** Insert or add a value with given index: O(n) if does not exist */
		VectorBTree& insertAdd(const Symbol& j, const Vector& v);

		/** Insert a config into another config, replace if key already exists */
		void insert(const VectorBTree& config);

		/** Insert a config into another config, add if key already exists */
		void insertAdd(const VectorBTree& config);

		/** Nr of vectors */
		inline size_t size() const { return size_; }

		/** Total dimensionality */
		inline size_t dim() const { return values_.size(); }

		/** Check whether Symbol j exists in config */
		inline bool contains(const Symbol& j) const { return ranges_.mem(j); }

		/** return all the nodes in the graph **/
		std::vector<Symbol> get_names() const;

		/** Vector access in VectorBtree is via the SubVector type */
		SubVector operator[](const Symbol& j);
		ConstSubVector operator[](const Symbol& j) const;

		/** [set] and [get] provided for access via MATLAB */
		void set(const Symbol& j, const Vector& v) { (*this)[j] = v; }
		inline const Vector get(const Symbol& j) const { return (*this)[j];}

		/** max of the vectors */
		double max() const;

		/**
		 * Check if compatible with other config, which is only
		 * guaranteed if vectors are inserted in exactly the same order,
		 * or if one config was created from the other using assignment.
		 * In the latter case, comparison is O(1), otherwise can be O(n).
		 */
		inline bool compatible(const VectorBTree& other) const {
			return ranges_ == other.ranges_;
		}

		/**
		 * O(1) check if structure of config is *physically* the same.
		 * i.e., configs were created through some assignment chain.
		 */
		inline bool cloned(const VectorBTree& other) const {
			return ranges_.same(other.ranges_);
		}

		/** Math operators */
		VectorBTree scale(double s) const;
		VectorBTree operator*(double s) const;
		VectorBTree operator-() const;
		void operator+=(const VectorBTree &b);
		VectorBTree operator+(const VectorBTree &b) const;
		void operator-=(const VectorBTree &b);
		VectorBTree operator-(const VectorBTree &b) const;
		double dot(const VectorBTree& b) const;

		/** Set all vectors to zero */
		VectorBTree& zero();

		/** Create a clone of x with exactly same structure, except with zero values */
		static VectorBTree zero(const VectorBTree& x);

		/**
		 * Add a delta config, needed for use in NonlinearOptimizer
		 * For VectorBTree, this is just addition.
		 */
		friend VectorBTree expmap(const VectorBTree& original, const VectorBTree& delta);

		/**
		 * Add a delta vector (not a config)
		 * Will use the ordering that map uses to loop over vectors
		 */
		friend VectorBTree expmap(const VectorBTree& original, const Vector& delta);

		/**
		 * BLAS Level 1 scal: x <- alpha*x
		 */
		friend void scal(double alpha, VectorBTree& x);

		/**
		 * BLAS Level 1 axpy: y <- alpha*x + y
		 * UNSAFE !!!! Only works if x and y laid out in exactly same shape
		 * Used in internal loop in iterative for fast conjugate gradients
		 * Consider using other functions if this is not in hotspot
		 */
		friend void axpy(double alpha, const VectorBTree& x, VectorBTree& y);

		/** @brief Const iterator */
		class const_iterator {

		public:

			// traits for playing nice with STL
			typedef ptrdiff_t difference_type; // correct ?
			typedef std::forward_iterator_tag iterator_category;
			typedef std::pair<Symbol,Vector> value_type;
			typedef const value_type* pointer;
			typedef const value_type& reference;

			bool operator==(const const_iterator& __x) const { return it_ == __x.it_;}
			bool operator!=(const const_iterator& __x) const { return it_ != __x.it_;}

			reference operator*()  const { return value_;}
			pointer   operator->() const { return &value_;}

			const_iterator& operator++() { increment(); return *this; }
			const_iterator  operator++(int) {
				const_iterator __tmp = *this; increment(); return __tmp;
			}

		private:

			Ranges::const_iterator it_, end_;
			const VectorBTree& config_;
			value_type value_;

			const_iterator(const VectorBTree& config, const Ranges::const_iterator& it) :
				it_(it), end_(config_.ranges_.end()), config_(config) {
				update();
			}

			void update() {
				if (it_ != end_) value_ = std::make_pair(it_->first, config_.get(it_->second));
			}

			void increment() { it_++; update();}

			friend class VectorBTree;
		}; // const_iterator

		// We do not have a non-const iterator right now
		typedef const_iterator iterator;

		/** return iterators */
		const_iterator begin() const { return const_iterator(*this,ranges_.begin());}
		const_iterator end  () const { return const_iterator(*this,ranges_.end());}

#ifdef UNTESTED

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version)
		{
			ar & BOOST_SERIALIZATION_NVP(values);
		}
	}; // VectorBTree

#endif

	}; // VectorBTree

	/** scalar product */
	inline VectorBTree operator*(double s, const VectorBTree& x) {
		return x * s;
	}

	/** dim function (for iterative::CGD) */
	inline double dim(const VectorBTree& x) {
		return x.dim();
	}

	/** max of the vectors */
	inline double max(const VectorBTree& x) {
		return x.max();
	}

	/* dot product */
	inline double dot(const VectorBTree& a, const VectorBTree& b) {
		return a.dot(b);
	}

	/** print with optional string */
	inline void print(const VectorBTree& v, const std::string& s = "") {
		v.print(s);
	}

} // gtsam
