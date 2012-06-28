/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ClusterTree-inl.h
 * @date July 13, 2010
 * @author Kai Ni
 * @author Frank Dellaert
 * @brief: Collects factorgraph fragments defined on variable clusters, arranged in a tree
 */

#pragma once

#include <iostream>
#include <boost/foreach.hpp>

#include <gtsam/inference/ClusterTree.h>

namespace gtsam {

	/* ************************************************************************* *
	 * Cluster
	 * ************************************************************************* */
	template<class FG>
	template<class Iterator>
	ClusterTree<FG>::Cluster::Cluster(const FG& fg, Index key, Iterator firstSeparator, Iterator lastSeparator) :
	FG(fg), frontal(1, key), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
  template<class FG>
  template<typename FRONTALIT, typename SEPARATORIT>
  ClusterTree<FG>::Cluster::Cluster(
      const FG& fg, FRONTALIT firstFrontal, FRONTALIT lastFrontal, SEPARATORIT firstSeparator, SEPARATORIT lastSeparator) :
      FG(fg), frontal(firstFrontal, lastFrontal), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
  template<class FG>
  template<typename FRONTALIT, typename SEPARATORIT>
  ClusterTree<FG>::Cluster::Cluster(
      FRONTALIT firstFrontal, FRONTALIT lastFrontal, SEPARATORIT firstSeparator, SEPARATORIT lastSeparator) :
      frontal(firstFrontal, lastFrontal), separator(firstSeparator, lastSeparator) {}

  /* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::addChild(typename ClusterTree<FG>::Cluster::shared_ptr child) {
	  children_.push_back(child);
	}

	/* ************************************************************************* */
	template<class FG>
	bool ClusterTree<FG>::Cluster::equals(const Cluster& other) const {
		if (frontal != other.frontal) return false;
		if (separator != other.separator) return false;
		if (children_.size() != other.children_.size()) return false;

		typename std::list<shared_ptr>::const_iterator it1 = children_.begin();
		typename std::list<shared_ptr>::const_iterator it2 = other.children_.begin();
		for (; it1 != children_.end(); it1++, it2++)
			if (!(*it1)->equals(**it2)) return false;

		return true;
	}

	/* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::print(const std::string& indent,
			const IndexFormatter& formatter) const {
		std::cout << indent;
		BOOST_FOREACH(const Index key, frontal)
						std::cout << formatter(key) << " ";
		std::cout << ": ";
		BOOST_FOREACH(const Index key, separator)
						std::cout << key << " ";
		std::cout << std::endl;
	}

	/* ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::Cluster::printTree(const std::string& indent,
			const IndexFormatter& formatter) const {
		print(indent, formatter);
		BOOST_FOREACH(const shared_ptr& child, children_)
						child->printTree(indent + "  ", formatter);
	}

	/* ************************************************************************* *
	 * ClusterTree
	 * ************************************************************************* */
	template<class FG>
	void ClusterTree<FG>::print(const std::string& str,
			const IndexFormatter& formatter) const {
		std::cout << str << std::endl;
		if (root_) root_->printTree("", formatter);
	}

	/* ************************************************************************* */
	template<class FG>
	bool ClusterTree<FG>::equals(const ClusterTree<FG>& other, double tol) const {
		if (!root_ && !other.root_) return true;
		if (!root_ || !other.root_) return false;
		return root_->equals(*other.root_);
	}

} //namespace gtsam
