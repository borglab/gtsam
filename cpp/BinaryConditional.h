/**
 * @file    DiscreteConditional.h
 * @brief   Discrete Conditional node for use in Bayes nets
 * @author  Manohar Paluri
 */

// \callgraph

#pragma once

#include <list>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp> // TODO: make cpp file
#include <boost/serialization/list.hpp>
#include "Conditional.h"

namespace gtsam {

	/**
	 * Conditional node for use in a Bayes net
	 */
	class BinaryConditional: public Conditional {

	private:

		std::list<std::string> parents_;

	public:

		/** convenience typename for a shared pointer to this class */
		typedef boost::shared_ptr<BinaryConditional> shared_ptr;

		/**
		 * Empty Constructor to make serialization possible
		 */
		BinaryConditional(){}

		/**
		 * No parents
		 */
		BinaryConditional(const std::string& key, double p) :
			Conditional(key) {
		}

		/**
		 * Single parent
		 */
		BinaryConditional(const std::string& key, const std::string& parent, const std::vector<double>& cpt) :
			Conditional(key) {
			parents_.push_back(parent);
		}

		/** print */
		void print(const std::string& s = "BinaryConditional") const {
			std::cout << s << " P(" << key_;
			if (parents_.size()>0) std::cout << " |";
			BOOST_FOREACH(std::string parent, parents_) std::cout << " " << parent;
			std::cout << ")" << std::endl;
		}

		/** check equality */
		bool equals(const Conditional& c, double tol = 1e-9) const {
			if (!Conditional::equals(c)) return false;
			const BinaryConditional* p = dynamic_cast<const BinaryConditional*> (&c);
			if (p == NULL) return false;
			return parents_ == p->parents_;
		}

		/** return parents */
		std::list<std::string> parents() const { return parents_;}

		/** find the number of parents */
		size_t nrParents() const {
			return parents_.size();
		}

	private:
		/** Serialization function */
		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & boost::serialization::make_nvp("Conditional", boost::serialization::base_object<Conditional>(*this));
			ar & BOOST_SERIALIZATION_NVP(parents_);
		}
	};
} /// namespace gtsam
