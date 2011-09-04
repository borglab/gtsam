/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VectorValues.h
 * @brief   Factor Graph Values
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

namespace gtsam {

	/**
	 * The class VectorValues stores a number of Vectors.
	 * Typically, this class is used in back-substitution (Bayes Net solve).
	 *
	 * There are a number of constructors that simply reserve space
	 * (Frank is not a big fan of this imperative style, but it is needed for speed)
	 * and others - safer ones - that actually initialize values.
	 * SameStructure and reserve are two other utility functions that manage storage.
	 *
	 * Access is through the variable index j, and returns a SubVector,
	 * which is a view on the underlying data structure.
	 */
	class VectorValues: public Testable<VectorValues> {
	protected:
		Vector values_;
		std::vector<size_t> varStarts_; // start at 0 with size nVars + 1

	public:
		 // Forward declaration of iterator implementation
		template<class C> class _impl_iterator;
		typedef _impl_iterator<VectorValues> iterator;
		typedef _impl_iterator<const VectorValues> const_iterator;

		// Some other typedefs
		typedef boost::shared_ptr<VectorValues> shared_ptr;
		typedef SubVector value_reference_type;
		typedef ConstSubVector const_value_reference_type;
		typedef SubVector mapped_type;
		typedef ConstSubVector const_mapped_type;

		/** Constructors that simply reserve space */

		/**
		 * Default constructor creates an empty VectorValues.  reserve(...) must be
		 * called to allocate space before any values can be added.  This prevents
		 * slow reallocation of space at runtime.
		 */
		VectorValues() :
				varStarts_(1, 0) {
		}

		/** Construct from a container of variable dimensions (in variable order). */
		template<class CONTAINER>
		VectorValues(const CONTAINER& dimensions);

		/** Construct to hold nVars vectors of varDim dimension each. */
		VectorValues(Index nVars, size_t varDim);

		/** Constructors that actually initialize values */

		/** Construct from a container of variable dimensions in variable order and
		 * a combined Vector of all of the variables in order.*/
		VectorValues(const std::vector<size_t>& dimensions, const Vector& values);

		/** Construct from the variable dimensions in varaible order and a double array that contains actual values */
		VectorValues(const std::vector<size_t>& dimensions, const double* values);

		/** Copy constructor */
		VectorValues(const VectorValues &V) :
				values_(V.values_), varStarts_(V.varStarts_) {
		}

		/** Named constructor to create a VectorValues that matches the structure of
		 * the specified VectorValues, but do not initialize the new values. */
		static VectorValues SameStructure(const VectorValues& otherValues);

		/** Reserve space for a total number of variables and dimensionality */
		void reserve(Index nVars, size_t totalDims) {
			values_.conservativeResize(totalDims);
			varStarts_.reserve(nVars + 1);
		}

		/** print required by Testable for unit testing */
		void print(const std::string& str = "VectorValues: ") const;

		/** equals required by Testable for unit testing */
		bool equals(const VectorValues& x, double tol = 1e-9) const;

		/** Number of elements */
		Index size() const {
			return varStarts_.size() - 1;
		}

		/** dimension of a particular element */
		size_t dim(Index j) const;

		/** Total dimensionality used (could be smaller than what has been allocated
		 * with reserve(...) ).
		 */
		size_t dim() const {
			return varStarts_.back();
		}

		/** Total dimensions capacity allocated */
		size_t dimCapacity() const {
			return values_.size();
		}

		/** Reference the entire solution vector (const version). */
		const Vector& vector() const {
			return values_;
		}

		/** Reference the entire solution vector. */
		Vector& vector() {
			return values_;
		}

		/** Individual element access */
		mapped_type operator[](Index j);
		const_mapped_type operator[](Index j) const;

		/** Iterator access */
		iterator begin() {
			return _impl_iterator<VectorValues>(*this, 0);
		}
		const_iterator begin() const {
			return _impl_iterator<const VectorValues>(*this, 0);
		}
		iterator end() {
			return _impl_iterator<VectorValues>(*this, varStarts_.size() - 1);
		}
		const_iterator end() const {
			return _impl_iterator<const VectorValues>(*this, varStarts_.size() - 1);
		}

		/** access a range of indices (of no particular order) as a single vector */
		template<class ITERATOR>
		Vector range(const ITERATOR& idx_begin, const ITERATOR& idx_end) const;

		/** set a range of indices as a single vector split across the range */
		template<class ITERATOR>
		void range(const ITERATOR& idx_begin, const ITERATOR& idx_end,
				const Vector& v);

		/** Set all elements to zero */
		void makeZero() {
			values_.setZero();
		}

		/** Copy structure of x, but set all values to zero */
		static VectorValues zero(const VectorValues& x);

		/**
		 * Append a variable using the next variable ID, and return that ID.  Space
		 * must have been allocated ahead of time using reserve(...).
		 */
		Index push_back_preallocated(const Vector& vector);

		/* dot product */
		double dot(const VectorValues& V) const {
			return gtsam::dot(this->values_, V.values_);
		}

		/**
		 * + operator simply adds Vectors.  This checks for structural equivalence
		 * when NDEBUG is not defined.
		 */
		VectorValues operator+(const VectorValues& c) const;

		void operator+=(const VectorValues& c);

		/**
		 * Iterator (handles both iterator and const_iterator depending on whether
		 * the template type is const.
		 */
		template<class C>
		class _impl_iterator {
		protected:
			C& config_;
			Index curVariable_;

			_impl_iterator(C& config, Index curVariable) :
					config_(config), curVariable_(curVariable) {
			}
			void checkCompat(const _impl_iterator<C>& r) {
				assert(&config_ == &r.config_);
			}
			friend class VectorValues;

		public:
			typedef typename const_selector<C, VectorValues,
					VectorValues::mapped_type, VectorValues::const_mapped_type>::type value_type;
			_impl_iterator<C>& operator++() {
				++curVariable_;
				return *this;
			}
			_impl_iterator<C>& operator--() {
				--curVariable_;
				return *this;
			}
			_impl_iterator<C>& operator++(int) {
				throw std::runtime_error("Use prefix ++ operator");
			}
			_impl_iterator<C>& operator--(int) {
				throw std::runtime_error("Use prefix -- operator");
			}
			_impl_iterator<C>& operator+=(ptrdiff_t step) {
				curVariable_ += step;
				return *this;
			}
			_impl_iterator<C>& operator-=(ptrdiff_t step) {
				curVariable_ += step;
				return *this;
			}
			ptrdiff_t operator-(const _impl_iterator<C>& r) {
				checkCompat(r);
				return curVariable_ - r.curVariable_;
			}
			bool operator==(const _impl_iterator<C>& r) {
				checkCompat(r);
				return curVariable_ == r.curVariable_;
			}
			bool operator!=(const _impl_iterator<C>& r) {
				checkCompat(r);
				return curVariable_ != r.curVariable_;
			}
			value_type operator*() {
				return config_[curVariable_];
			}
		};

	protected:
		void checkVariable(Index j) const {
			assert(j < varStarts_.size()-1);
		}

	public:
		friend size_t dim(const VectorValues& V) {
			return V.varStarts_.back();
		}
		friend double dot(const VectorValues& V1, const VectorValues& V2) {
			return gtsam::dot(V1.values_, V2.values_);
		}
		friend void scal(double alpha, VectorValues& x) {
			gtsam::scal(alpha, x.values_);
		}
		friend void axpy(double alpha, const VectorValues& x, VectorValues& y) {
			gtsam::axpy(alpha, x.values_, y.values_);
		}
		friend void sqrt(VectorValues &x) {
			Vector y = gtsam::esqrt(x.values_);
			x.values_ = y;
		}

		friend void ediv(const VectorValues& numerator,
				const VectorValues& denominator, VectorValues &result) {
			assert(
					numerator.dim() == denominator.dim() && denominator.dim() == result.dim());
			const size_t sz = result.dim();
			for (size_t i = 0; i < sz; ++i)
				result.values_[i] = numerator.values_[i] / denominator.values_[i];
		}

		friend void edivInPlace(VectorValues& x, const VectorValues& y) {
			assert(x.dim() == y.dim());
			const size_t sz = x.dim();
			for (size_t i = 0; i < sz; ++i)
				x.values_[i] /= y.values_[i];
		}

		// check whether there's a zero in the vector
		friend bool anyZero(const VectorValues& x, double tol = 1e-5) {
			bool flag = false;
			size_t i = 0;
			for (const double *v = x.values_.data(); i < (size_t) x.values_.size();
					++v) {
				if (*v < tol && *v > -tol) {
					flag = true;
					break;
				}
				++i;
			}
			return flag;
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class ARCHIVE>
		void serialize(ARCHIVE & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(values_);
			ar & BOOST_SERIALIZATION_NVP(varStarts_);
		}
	};
	// \class VectorValues definition

/// Implementations of functions

	template<class CONTAINER>
	inline VectorValues::VectorValues(const CONTAINER& dimensions) :
			varStarts_(dimensions.size() + 1) {
		varStarts_[0] = 0;
		size_t varStart = 0;
		Index var = 0;
		BOOST_FOREACH(size_t dim, dimensions)
				{
					varStarts_[++var] = (varStart += dim);
				}
		values_.resize(varStarts_.back());
	}

	template<class ITERATOR>
	inline Vector VectorValues::range(const ITERATOR& idx_begin,
			const ITERATOR& idx_end) const {
		// find the size of the vector to build
		size_t s = 0;
		for (ITERATOR it = idx_begin; it != idx_end; ++it)
			s += dim(*it);

		// assign vector
		Vector result(s);
		size_t start = 0;
		for (ITERATOR it = idx_begin; it != idx_end; ++it) {
			ConstSubVector v = (*this)[*it];
			const size_t d = v.size();
			result.segment(start, d).operator=(v); // This syntax works around what seems to be a bug in clang++
			start += d;
		}
		return result;
	}

	template<class ITERATOR>
	inline void VectorValues::range(const ITERATOR& idx_begin,
			const ITERATOR& idx_end, const Vector& v) {
		size_t start = 0;
		for (ITERATOR it = idx_begin; it != idx_end; ++it) {
			checkVariable(*it);
			const size_t d = dim(*it);
			(*this)[*it] = v.segment(start, d);
			start += d;
		}
	}

	struct DimSpec: public std::vector<size_t> {

		typedef std::vector<size_t> Base;
		typedef boost::shared_ptr<DimSpec> shared_ptr;

		DimSpec() :
				Base() {
		}
		DimSpec(size_t n) :
				Base(n) {
		}
		DimSpec(size_t n, size_t init) :
				Base(n, init) {
		}
		DimSpec(const VectorValues &V) :
				Base(V.size()) {
			const size_t n = V.size();
			for (size_t i = 0; i < n; ++i) {
				(*this)[i] = V[i].size();
			}
		}
	};

} // \namespace gtsam
