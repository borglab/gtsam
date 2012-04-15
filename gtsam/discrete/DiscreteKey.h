/*
 * DiscreteKey.h
 * @brief specialized key for discrete variables
 * @author Frank Dellaert
 * @date Feb 28, 2011
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/discrete/label_traits.h>

#include <map>
#include <string>
#include <vector>

namespace gtsam {

	typedef std::pair<Index,size_t> DiscreteKey;

	/**
	 * Key type for discrete conditionals
	 * Includes name and cardinality
	 */
	class OldDiscreteKey : std::pair<Index,size_t> {

	private:

		std::string name_;

	public:

		/** Default constructor */
		OldDiscreteKey() :
			std::pair<Index,size_t>(0,0), name_("default") {
		}

		/** Constructor, defaults to binary */
		OldDiscreteKey(Index j, const std::string& name, size_t cardinality = 2) :
			std::pair<Index,size_t>(j,cardinality), name_(name) {
		}

		virtual ~OldDiscreteKey() {
		}

		// Testable
		bool equals(const OldDiscreteKey& other, double tol = 1e-9) const;
		void print(const std::string& s = "") const;

		operator Index() const { return first; }

		const std::string& name() const {
			return name_;
		}

		size_t cardinality() const {
			return second;
		}

		/** compare 2 keys by their name */
		bool operator <(const OldDiscreteKey& other) const {
			return name_ < other.name_;
		}

		/** equality */
		bool operator==(const OldDiscreteKey& other) const {
			return (first == other.first) && (second == other.second) && (name_ == other.name_);
		}

		bool operator!=(const OldDiscreteKey& other) const {
			return !(*this == other);
		}

		/** provide streaming */
		friend std::ostream& operator <<(std::ostream &os, const OldDiscreteKey &key);

	}; // OldDiscreteKey

	/// DiscreteKeys is a set of keys that can be assembled using the & operator
	struct DiscreteKeys: public std::vector<DiscreteKey> {

		/// Default constructor
		DiscreteKeys() {
		}

		/// Construct from a key
		DiscreteKeys(const DiscreteKey& key) {
			push_back(key);
		}

		/// Construct from a vector of keys
		DiscreteKeys(const std::vector<DiscreteKey>& keys) :
			std::vector<DiscreteKey>(keys) {
		}

		/// Construct from cardinalities with default names
		DiscreteKeys(const std::vector<int>& cs);

		/// Return a vector of indices
		std::vector<Index> indices() const;

		/// Return a map from index to cardinality
		std::map<Index,size_t> cardinalities() const;

		/// Add a key (non-const!)
		DiscreteKeys& operator&(const DiscreteKey& key) {
			push_back(key);
			return *this;
		}
	}; // DiscreteKeys

	/// Create a list from two keys
	DiscreteKeys operator&(const DiscreteKey& key1, const DiscreteKey& key2);

	/// traits class for DiscreteKey for use with DecisionTree/DecisionDiagram
	template<>
	struct label_traits<OldDiscreteKey> {
		/** get cardinality from type */
		static size_t cardinality(const OldDiscreteKey& key) {
			return key.cardinality();
		}
		/** compare 2 keys by their name */
		static bool higher(const OldDiscreteKey& a, const OldDiscreteKey& b) {
			return a.name() < b.name();
		}
		/** hash function */
		static size_t hash_value(const OldDiscreteKey& a) {
      boost::hash<std::string> hasher;
      return hasher(a.name());
		}
	};

}
