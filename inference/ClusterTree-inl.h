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
	template<class Iterator>
	ClusterTree<FG>::Cluster::Cluster(const FG& fg, varid_t key, Iterator firstSeparator, Iterator lastSeparator) :
	FG(fg), frontal(1, key), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
  template<class FG>
  template<typename FrontalIt, typename SeparatorIt>
  ClusterTree<FG>::Cluster::Cluster(
      const FG& fg, FrontalIt firstFrontal, FrontalIt lastFrontal, SeparatorIt firstSeparator, SeparatorIt lastSeparator) :
      FG(fg), frontal(firstFrontal, lastFrontal), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
  template<class FG>
  template<typename FrontalIt, typename SeparatorIt>
  ClusterTree<FG>::Cluster::Cluster(
      FrontalIt firstFrontal, FrontalIt lastFrontal, SeparatorIt firstSeparator, SeparatorIt lastSeparator) :
      frontal(firstFrontal, lastFrontal), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::addChild(typename ClusterTree<FG>::Cluster::shared_ptr child) {
	  children_.push_back(child);
	}

	/* ************************************************************************* */
	template<class FG>
	bool ClusterTree<FG>::Cluster::equals(const ClusterTree<FG>::Cluster& other) const {
		if (frontal != other.frontal) return false;
		if (separator != other.separator) return false;
		if (children_.size() != other.children_.size()) return false;

		typename list<shared_ptr>::const_iterator it1 = children_.begin();
		typename list<shared_ptr>::const_iterator it2 = other.children_.begin();
		for (; it1 != children_.end(); it1++, it2++)
			if (!(*it1)->equals(**it2)) return false;

		return true;
	}

	/* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::print(const string& indent) const {
		cout << indent;
		BOOST_FOREACH(const varid_t key, frontal)
						cout << key << " ";
		cout << ": ";
		BOOST_FOREACH(const varid_t key, separator)
						cout << key << " ";
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
