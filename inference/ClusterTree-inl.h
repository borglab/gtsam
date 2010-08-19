/*
 * ClusterTree-inl.h
 * Created on: July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <boost/foreach.hpp>

#include <gtsam/inference/ClusterTree.h>

namespace gtsam {

	using namespace std;

	/* ************************************************************************* *
	 * Cluster
	 * ************************************************************************* */
	template<class FG>
	ClusterTree<FG>::Cluster::Cluster(const FG& fg, const Symbol& key):FG(fg) {

		// push the one key as frontal
		frontal_.push_back(key);

		// the rest are separator keys...
		BOOST_FOREACH(const Symbol& graphKey, fg.keys())
			if (graphKey != key)
				separator_.insert(graphKey);
	}

	/* ************************************************************************* */
	template<class FG>
	bool ClusterTree<FG>::Cluster::equals(const ClusterTree<FG>::Cluster& other) const {
		if (!frontal_.equals(other.frontal_)) return false;
		if (!separator_.equals(other.separator_)) return false;
		if (children_.size() != other.children_.size()) return false;

		typename vector<shared_ptr>::const_iterator it1 = children_.begin();
		typename vector<shared_ptr>::const_iterator it2 = other.children_.begin();
		for (; it1 != children_.end(); it1++, it2++)
			if (!(*it1)->equals(**it2)) return false;

		return true;
	}

	/* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::print(const string& indent) const {
		cout << indent;
		BOOST_FOREACH(const Symbol& key, frontal_)
						cout << (string) key << " ";
		cout << ":";
		BOOST_FOREACH(const Symbol& key, separator_)
						cout << (string) key << " ";
		cout << endl;
	}

	/* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::printTree(const string& indent) const {
		print(indent);
		BOOST_FOREACH(const shared_ptr& child, children_)
						child->printTree(indent + "  ");
	}

	/* ************************************************************************* *
	 * ClusterTree
	 * ************************************************************************* */
	template<class FG>
	bool ClusterTree<FG>::equals(const ClusterTree<FG>& other, double tol) const {
		if (!root_ && !other.root_) return true;
		if (!root_ || !other.root_) return false;
		return root_->equals(*other.root_);
	}

} //namespace gtsam
