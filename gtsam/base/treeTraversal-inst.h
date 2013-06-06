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

namespace gtsam {

  /** Internal functions used for traversing trees */
  namespace treeTraversal {

    namespace {
      // Internal node used in DFS preorder stack
      template<typename NODE, typename DATA>
      struct TraversalNode {
        bool expanded;
        NODE& const treeNode;
        DATA data;
        TraversalNode(NODE* _treeNode, const DATA& _data) :
          expanded(false), treeNode(_treeNode), data(_data) {}
      };

      // Functional object to visit a node and then add it to the traversal stack
      template<typename NODE, typename VISITOR, typename DATA, typename STACK>
      struct PreOrderExpand {
        VISITOR& visitor_;
        DATA& parentData_;
        STACK& stack_;
        PreOrderExpand(VISITOR& visitor, DATA& parentData, STACK& stack) :
          visitor_(visitor), parentData_(parentData), stack_(stack) {}
        template<typename P>
        void operator()(const P& nodePointer) {
          // Add node
          stack_.push(TraversalNode<NODE,DATA>(*nodePointer, visitor_(nodePointer, parentData_)));
        }
      };

      /// Do nothing - default argument for post-visitor for tree traversal
      template<typename NODE, typename DATA>
      void no_op(const NODE& node, const DATA& data) {}
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
      // Depth first traversal stack
      typedef TraversalNode<typename FOREST::Node, DATA> TraversalNode;
      typedef std::stack<TraversalNode, FastList<TraversalNode> > Stack;
      typedef PreOrderExpand<typename FOREST::Node, VISITOR_PRE, DATA, Stack> Expander;
      Stack stack;

      // Add roots to stack (use reverse iterators so children are processed in the order they
      // appear)
      (void) std::for_each(forest.roots().rbegin(), forest.roots().rend(),
        Expander(visitorPre, rootData, stack));

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
          (void) std::for_each(node.treeNode->children.rbegin(), node.treeNode->children.rend(),
            Expander(visitorPre, node.data, stack));
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
      DepthFirstForest<FOREST, DATA, VISITOR_PRE, void(&)(const typename FOREST::Node&, const DATA&)>(
        forest, rootData, visitorPre, no_op<typename FOREST::Node, DATA>);
    }
    
    /** Traversal function for CloneForest */
    namespace {
      template<typename NODE>
      boost::shared_ptr<NODE> CloneForestVisitorPre(const NODE& node, const boost::shared_ptr<NODE>& parentPointer)
      {
        // Clone the current node and add it to its cloned parent
        boost::shared_ptr<NODE> clone = boost::make_shared<NODE>(node);
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

  }

}