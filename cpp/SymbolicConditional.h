/**
 * @file    SymbolicConditional.h
 * @brief   Symbolic Conditional node for use in Bayes nets
 * @author  Frank Dellaert
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
	class SymbolicConditional: public Conditional {

	private:

		std::list<std::string> parents_;

	public:

		/** convenience typename for a shared pointer to this class */
		typedef boost::shared_ptr<SymbolicConditional> shared_ptr;

		/**
		 * Empty Constructor to make serialization possible
		 */
		SymbolicConditional(){}

		/**
		 * No parents
		 */
		SymbolicConditional(const std::string& key) :
			Conditional(key) {
		}

		/**
		 * Single parent
		 */
		SymbolicConditional(const std::string& key, const std::string& parent) :
			Conditional(key) {
			parents_.push_back(parent);
		}

		/**
		 * Two parents
		 */
		SymbolicConditional(const std::string& key, const std::string& parent1,
				const std::string& parent2) :
			Conditional(key) {
			parents_.push_back(parent1);
			parents_.push_back(parent2);
		}

		/**
		 * Three parents
		 */
		SymbolicConditional(const std::string& key, const std::string& parent1,
				const std::string& parent2, const std::string& parent3) :
			Conditional(key) {
			parents_.push_back(parent1);
			parents_.push_back(parent2);
			parents_.push_back(parent3);
		}

		/**
		 * A list
		 */
		SymbolicConditional(const std::string& key,
				const std::list<std::string>& parents) :
			Conditional(key), parents_(parents) {
		}

		/** print */
		void print(const std::string& s = "SymbolicConditional") const {
			std::cout << s << " P(" << key_;
			if (parents_.size()>0) std::cout << " |";
			BOOST_FOREACH(std::string parent, parents_) std::cout << " " << parent;
			std::cout << ")" << std::endl;
		}

		/** check equality */
		bool equals(const Conditional& c, double tol = 1e-9) const {
			if (!Conditional::equals(c)) return false;
			const SymbolicConditional* p = dynamic_cast<const SymbolicConditional*> (&c);
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
