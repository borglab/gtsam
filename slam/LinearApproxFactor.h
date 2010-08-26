/*
 * @file LinearApproxFactor.h
 * @brief A dummy factor that allows a linear factor to act as a nonlinear factor
 * @author Alex Cunningham
 */

#pragma once

#include <vector>
#include <iostream>
#include <boost/foreach.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/linear/VectorConfig.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

/**
 * A dummy factor that takes a linearized factor and inserts it into
 * a nonlinear graph.  This version uses exactly one type of variable.
 */
template <class Config, class Key, class X>
class LinearApproxFactor : public NonlinearFactor<Config> {

public:
	/** base type */
	typedef NonlinearFactor<Config> Base;

	/** shared pointer for convenience */
	typedef boost::shared_ptr<LinearApproxFactor<Config,Key,X> > shared_ptr;

	/** typedefs for key vectors */
	typedef std::vector<Key> KeyVector;

protected:
	/** hold onto the factor itself */
	GaussianFactor::shared_ptr lin_factor_;

	/** linearization points for error calculation */
	Config lin_points_;

	/** keep keys for the factor */
	KeyVector nonlinearKeys_;

	/**
	 * use this for derived classes with keys that don't copy easily
	 */
	LinearApproxFactor(size_t dim, const Config& lin_points)
		: Base(noiseModel::Unit::Create(dim)), lin_points_(lin_points) {}

public:

	/** use this constructor when starting with nonlinear keys */
	LinearApproxFactor(GaussianFactor::shared_ptr lin_factor, const Config& lin_points)
		: Base(noiseModel::Unit::Create(lin_factor->get_model()->dim())),
		  lin_factor_(lin_factor), lin_points_(lin_points)
	{
		// create the keys and store them
		BOOST_FOREACH(Symbol key, lin_factor->keys()) {
			nonlinearKeys_.push_back(Key(key.index()));
			this->keys_.push_back(key);
		}
	}

	virtual ~LinearApproxFactor() {}

	/** Vector of errors, unwhitened ! */
	virtual Vector unwhitenedError(const Config& c) const {
		// extract the points in the new config
		VectorConfig delta;
		BOOST_FOREACH(const Key& key, nonlinearKeys_) {
			X newPt = c[key], linPt = lin_points_[key];
			Vector d = linPt.logmap(newPt);
			delta.insert(key, d);
		}

		return lin_factor_->unweighted_error(delta);
	}

	/**
	 * linearize to a GaussianFactor
	 * Just returns a copy of the existing factor
	 * NOTE: copies to avoid actual factor getting destroyed
	 * during elimination
	 */
	virtual boost::shared_ptr<GaussianFactor>
	linearize(const Config& c) const {
		Vector b = lin_factor_->get_b();
		SharedDiagonal model = lin_factor_->get_model();
		std::vector<std::pair<Symbol, Matrix> > terms;
		BOOST_FOREACH(Symbol key, lin_factor_->keys()) {
			terms.push_back(std::make_pair(key, lin_factor_->get_A(key)));
		}

		return boost::shared_ptr<GaussianFactor>(
				new GaussianFactor(terms, b, model));
	}

	/** get access to nonlinear keys */
	KeyVector nonlinearKeys() const { return nonlinearKeys_; }

	/** override print function */
	virtual void print(const std::string& s="") const {
		Base::print(s);
		lin_factor_->print();
	}

	/** access to b vector of gaussian */
	Vector get_b() const { return lin_factor_->get_b(); }
};

} // \namespace gtsam
