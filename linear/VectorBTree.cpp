/**
 * @file    VectorValues.cpp
 * @brief   Factor Graph Values
 * @brief   VectorValues
 * @author Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <gtsam/linear/VectorBTree.h>

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	// Check if two VectorValuess are compatible, throw exception if not
	static void check_compatible(const string& s, const VectorBTree& a,
			const VectorBTree& b) {
		if (!a.compatible(b))
			throw invalid_argument(s + ": incompatible VectorBTrees");
	}

	/* ************************************************************************* */
	// Check if two VectorBTrees are cloned, throw exception if not
	// The 2 configs need exactly the same ranges tree
	static void check_cloned(const string& s, const VectorBTree& a,
			const VectorBTree& b) {
		if (!a.cloned(b))
			throw invalid_argument(s + ": only defined for cloned VectorBTrees");
	}

	/* ************************************************************************* */
	void VectorBTree::print(const string& name) const {
	  odprintf("VectorBTree %s\n", name.c_str());
	  odprintf("size: %d\n", size_);
	  BOOST_FOREACH(const Pair& p, ranges_) {
	  	const Range& r = p.second;
	    odprintf("%s: %d:%d", string(p.first).c_str(), r.start(), r.size());
	    gtsam::print(get(r));
	  }
	}

	/* ************************************************************************* */
	bool VectorBTree::equals(const VectorBTree& expected, double tol) const {
	  BOOST_FOREACH(const Pair& p, ranges_) {
	  	const Symbol& j = p.first;
	  	if (!expected.contains(j)) return false;
	  	if (!equal_with_abs_tol(expected[j], get(p.second), tol))
	  		return false;
	  }
	  return true;
	}

	/* ************************************************************************* */
  VectorBTree& VectorBTree::insert(const Symbol& j, const Vector& v) {
  	if (contains(j)) throw invalid_argument("VectorBTree::insert: key already exists");
  	// calculate new range from current dimensionality and dim(a)
  	size_t n1 = values_.size(), n2 = n1 + v.size();
  	ranges_ = ranges_.add(j,Range(n1,n2));
  	// resize vector and COPY a into it
    values_.resize(n2,true);
    std::copy(v.begin(),v.end(),values_.begin()+n1);
    // increment size
    ++size_;
    return *this;
  }

	/* ************************************************************************* */
  VectorBTree& VectorBTree::insertAdd(const Symbol& j, const Vector& v) {
  	if (!contains(j)) return insert(j,v);
		const Range& r = ranges_.find(j);
	  SubVector(values_,r) += v;
    return *this;
  }

  /* ************************************************************************* */
  void VectorBTree::insert(const VectorBTree& config) {
	  BOOST_FOREACH(const Pair& p, config.ranges_)
	    insert(p.first,config.get(p.second));
  }

  /* ************************************************************************* */
  void VectorBTree::insertAdd(const VectorBTree& config) {
	  BOOST_FOREACH(const Pair& p, config.ranges_)
	    insertAdd(p.first,config.get(p.second));
  }

  /* ************************************************************************* */
  std::vector<Symbol> VectorBTree::get_names() const {
    std::vector<Symbol> names;
	  BOOST_FOREACH(const Pair& p, ranges_)
      names.push_back(p.first);
    return names;
  }

	/* ************************************************************************* */
	SubVector VectorBTree::operator[](const Symbol& j) {
		const Range& r = ranges_.find(j);
	  return SubVector(values_,r);
	}

	/* ************************************************************************* */
	ConstSubVector VectorBTree::operator[](const Symbol& j) const {
		const Range& r = ranges_.find(j);
	  return ConstSubVector(values_,r);
	}

	/* ************************************************************************* */
  double VectorBTree::max() const {
  	double m = -std::numeric_limits<double>::infinity();
	  BOOST_FOREACH(const Pair& p, ranges_)
  		m = std::max(m, gtsam::max(get(p.second)));
  	return m;
  }

		/* ************************************************************************* */
	VectorBTree VectorBTree::scale(double s) const {
		VectorBTree scaled = *this;
		scaled.values_ *= s;
		return scaled;
	}

	/* ************************************************************************* */
	VectorBTree VectorBTree::operator*(double s) const {
		return scale(s);
	}

	/* ************************************************************************* */
	VectorBTree VectorBTree::operator-() const {
		VectorBTree result = *this;
		result.values_ = - values_;
		return result;
	}

	/* ************************************************************************* */
	void VectorBTree::operator+=(const VectorBTree& b) {
		check_compatible("VectorBTree:operator+=", *this, b);
		values_ += b.values_;
	}

	/* ************************************************************************* */
	VectorBTree VectorBTree::operator+(const VectorBTree& b) const {
		check_compatible("VectorBTree:operator+", *this, b);
		VectorBTree result = *this;
		result += b;
		return result;
	}

	/* ************************************************************************* */
	void VectorBTree::operator-=(const VectorBTree& b) {
		check_compatible("VectorBTree:operator-=", *this, b);
		values_ -= b.values_;
	}

	/* ************************************************************************* */
	VectorBTree VectorBTree::operator-(const VectorBTree& b) const {
		VectorBTree result = *this;
		result -= b;
		return result;
	}

	/* ************************************************************************* */
	double VectorBTree::dot(const VectorBTree& b) const {
		check_compatible("VectorBTree:dot", *this, b);
		return gtsam::dot(values_,b.values_);
	}

	/* ************************************************************************* */
	VectorBTree& VectorBTree::zero() {
		std::fill(values_.begin(), values_.end(), 0.0);
		return *this;
	}

	/* ************************************************************************* */
	VectorBTree VectorBTree::zero(const VectorBTree& x) {
		VectorBTree cloned(x);
		return cloned.zero();
	}

	/* ************************************************************************* */
	VectorBTree expmap(const VectorBTree& original, const VectorBTree& delta) {
		check_compatible("VectorBTree:expmap", original, delta);
		return original + delta;
	}

	/* ************************************************************************* */
	VectorBTree expmap(const VectorBTree& original, const Vector& delta)
	{
		VectorBTree result = original;
		result.values_ += delta;
		return result;
	}

	/* ************************************************************************* */
	void scal(double alpha, VectorBTree& x) {
		scal(alpha, x.values_);
	}

	/* ************************************************************************* */
	void axpy(double alpha, const VectorBTree& x, VectorBTree& y) {
		check_cloned("VectorBTree:axpy", x, y);
		axpy(alpha, x.values_, y.values_);
	}

	/* ************************************************************************* */

} // gtsam
