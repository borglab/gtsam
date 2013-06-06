/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    treeTraversal-inst.h
* @author  Richard Roberts
* @date    April 9, 2013
*/
#pragma once

#include <gtsam/base/FastList.h>

#include <stack>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

namespace gtsam {

  /** Internal functions used for traversing trees */
  namespace treeTraversal {

    /* ************************************************************************* */
    namespace {
      // Internal node used in DFS preorder stack
      template<typename NODE, typename DATA>
      struct TraversalNode {
        bool expanded;
        const boost::shared_ptr<NODE>& treeNode;
        DATA data;
        TraversalNode(const boost::shared_ptr<NODE>& _treeNode, const DATA& _data) :
          expanded(false), treeNode(_treeNode), data(_data) {}
      };

      /// Do nothing - default argument for post-visitor for tree traversal
      template<typename NODE, typename DATA>
      void no_op(const boost::shared_ptr<NODE>& node, const DATA& data) {}
    }

    /** Traverse a forest depth-first with pre-order and post-order visits.
     *  @param forest The forest of trees to traverse.  The method \c forest.roots() should exist
     *         and return a collection of (shared) pointers to \c FOREST::Node.
     *  @param visitorPre \c visitorPre(node, parentData) will be called at every node, before
     *         visiting its children, and will be passed, by reference, the \c DATA object returned
     *         by the visit to its parent.  Likewise, \c visitorPre should return the \c DATA object
     *         to pass to the children.  The returned \c DATA object will be copy-constructed only
     *         upon returning to store internally, thus may be modified by visiting the children.
     *         Regarding efficiency, this copy-on-return is usually optimized out by the compiler.
     *  @param visitorPost \c visitorPost(node, data) will be called at every node, after visiting
     *         its children, and will be passed, by reference, the \c DATA object returned by the
     *         call to \c visitorPre (the \c DATA object may be modified by visiting the children).  
     *  @param rootData The data to pass by reference to \c visitorPre when it is called on each
     *         root node. */
    template<class FOREST, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
    void DepthFirstForest(FOREST& forest, DATA& rootData, VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost)
    {
      // Typedefs
      typedef typename FOREST::Node Node;
      typedef boost::shared_ptr<Node> sharedNode;

      // Depth first traversal stack
      typedef TraversalNode<typename FOREST::Node, DATA> TraversalNode;
      typedef std::stack<TraversalNode, FastList<TraversalNode> > Stack;
      Stack stack;

      // Add roots to stack (use reverse iterators so children are processed in the order they
      // appear)
      BOOST_REVERSE_FOREACH(const sharedNode& root, forest.roots())
        stack.push(TraversalNode(root, visitorPre(root, rootData)));

      // Traverse
      while(!stack.empty())
      {
        // Get next node
        TraversalNode& node = stack.top();

        if(node.expanded) {
          // If already expanded, then the data stored in the node is no longer needed, so visit
          // then delete it.
          (void) visitorPost(node.treeNode, node.data);
          stack.pop();
        } else {
          // If not already visited, visit the node and add its children (use reverse iterators so
          // children are processed in the order they appear)
          BOOST_REVERSE_FOREACH(const sharedNode& child, node.treeNode->children)
            stack.push(TraversalNode(child, visitorPre(child, node.data)));
          node.expanded = true;
        }
      }
    }
    
    /** Traverse a forest depth-first, with a pre-order visit but no post-order visit.
     *  @param forest The forest of trees to traverse.  The method \c forest.roots() should exist
     *         and return a collection of (shared) pointers to \c FOREST::Node.
     *  @param visitorPre \c visitorPre(node, parentData) will be called at every node, before
     *         visiting its children, and will be passed, by reference, the \c DATA object returned
     *         by the visit to its parent.  Likewise, \c visitorPre should return the \c DATA object
     *         to pass to the children.  The returned \c DATA object will be copy-constructed only
     *         upon returning to store internally, thus may be modified by visiting the children.
     *         Regarding efficiency, this copy-on-return is usually optimized out by the compiler.
     *  @param rootData The data to pass by reference to \c visitorPre when it is called on each
     *         root node. */
    template<class FOREST, typename DATA, typename VISITOR_PRE>
    void DepthFirstForest(FOREST& forest, DATA& rootData, VISITOR_PRE& visitorPre)
    {
      DepthFirstForest(forest, rootData, visitorPre, no_op<typename FOREST::Node, DATA>);
    }
    

    /* ************************************************************************* */
    /** Traversal function for CloneForest */
    namespace {
      template<typename NODE>
      boost::shared_ptr<NODE>
        CloneForestVisitorPre(const boost::shared_ptr<NODE>& node, const boost::shared_ptr<NODE>& parentPointer)
      {
        // Clone the current node and add it to its cloned parent
        boost::shared_ptr<NODE> clone = boost::make_shared<NODE>(*node);
        parentPointer->children.push_back(clone);
        return clone;
      }
    }

    /** Clone a tree, copy-constructing new nodes (calling boost::make_shared) and setting up child
     *  pointers for a clone of the original tree.
     *  @param forest The forest of trees to clone.  The method \c forest.roots() should exist and
     *         return a collection of shared pointers to \c FOREST::Node.
     *  @return The new collection of roots. */
    template<class FOREST>
    std::vector<boost::shared_ptr<typename FOREST::Node> > CloneForest(const FOREST& forest)
    {
      typedef typename FOREST::Node Node;
      boost::shared_ptr<Node> rootContainer = boost::make_shared<Node>();
      DepthFirstForest(forest, rootContainer, CloneForestVisitorPre<Node>);
      return std::vector<boost::shared_ptr<Node> >(rootContainer->children.begin(), rootContainer->children.end());
    }


    /* ************************************************************************* */
    /** Traversal function for PrintForest */
    namespace {
      template<typename NODE>
      std::string
        PrintForestVisitorPre(const boost::shared_ptr<NODE>& node, const std::string& parentString, const KeyFormatter& formatter)
      {
        // Print the current node
        node->print(parentString + "-", formatter);
        // Increment the indentation
        return parentString + "| ";
      }
    }

    /** Print a tree, prefixing each line with \c str, and formatting keys using \c keyFormatter.
     *  To print each node, this function calls the \c print function of the tree nodes. */
    template<class FOREST>
    void PrintForest(const FOREST& forest, const std::string& str, const KeyFormatter& keyFormatter) {
      typedef typename FOREST::Node Node;
      DepthFirstForest(forest, str, boost::bind(PrintForestVisitorPre<Node>, _1, _2, keyFormatter));
    }
  }

}