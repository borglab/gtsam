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
#include <gtsam/inference/Key.h>

#include <stack>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>

#include <tbb/tbb.h>
#undef max // TBB seems to include windows.h and we don't want these macros
#undef min

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
        DATA& parentData;
        typename FastList<DATA>::iterator dataPointer;
        TraversalNode(const boost::shared_ptr<NODE>& _treeNode, DATA& _parentData) :
          expanded(false), treeNode(_treeNode), parentData(_parentData) {}
      };

      // Do nothing - default argument for post-visitor for tree traversal
      template<typename NODE, typename DATA>
      void no_op(const boost::shared_ptr<NODE>& node, const DATA& data) {}

      // Internal node used in parallel traversal stack
      template<typename NODE, typename DATA>
      struct ParallelTraversalNode {
        const boost::shared_ptr<NODE>& treeNode;
        DATA myData;
        ParallelTraversalNode(const boost::shared_ptr<NODE>& treeNode, const DATA& myData) :
          treeNode(treeNode), myData(myData) {}
      };

      template<typename NODE, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
      class PreOrderTask : public tbb::task
      {
      public:
        const boost::shared_ptr<NODE>& treeNode;
        DATA myData;
        VISITOR_PRE& visitorPre;
        VISITOR_POST& visitorPost;
        int problemSizeThreshold;
        bool makeNewTasks;
        PreOrderTask(const boost::shared_ptr<NODE>& treeNode, const DATA& myData,
          VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost, int problemSizeThreshold,
          bool makeNewTasks = true) :
          treeNode(treeNode), myData(myData), visitorPre(visitorPre), visitorPost(visitorPost),
          problemSizeThreshold(problemSizeThreshold), makeNewTasks(makeNewTasks) {}

        typedef ParallelTraversalNode<NODE, DATA> ParallelTraversalNode;

        tbb::task* execute()
        {
          // Process this node and its children
          processNode(treeNode, myData);

          // Return NULL
          return NULL;
        }

        void processNode(const boost::shared_ptr<NODE>& node, DATA& myData)
        {
          if(makeNewTasks)
          {
            bool overThreshold = (node->problemSize() >= problemSizeThreshold);

            tbb::task_list childTasks;
            BOOST_FOREACH(const boost::shared_ptr<NODE>& child, node->children)
            {
              // Process child in a subtask
              childTasks.push_back(*new(allocate_child())
                PreOrderTask(child, visitorPre(child, myData), visitorPre, visitorPost,
                problemSizeThreshold, overThreshold));
            }

            // If we have child tasks, start subtasks and wait for them to complete
            set_ref_count(1 + node->children.size());
            spawn(childTasks);
            wait_for_all();
          }
          else
          {
            BOOST_FOREACH(const boost::shared_ptr<NODE>& child, node->children)
            {
              processNode(child, visitorPre(child, myData));
            }
          }

          // Run the post-order visitor
          (void) visitorPost(node, myData);
        }
      };

      template<typename ROOTS, typename NODE, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
      class RootTask : public tbb::task
      {
      public:
        const ROOTS& roots;
        DATA& myData;
        VISITOR_PRE& visitorPre;
        VISITOR_POST& visitorPost;
        int problemSizeThreshold;
        RootTask(const ROOTS& roots, DATA& myData, VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost,
          int problemSizeThreshold) :
          roots(roots), myData(myData), visitorPre(visitorPre), visitorPost(visitorPost),
            problemSizeThreshold(problemSizeThreshold) {}

        tbb::task* execute()
        {
          typedef PreOrderTask<NODE, DATA, VISITOR_PRE, VISITOR_POST> PreOrderTask;
          // Set TBB ref count
          set_ref_count(1 + (int)roots.size());
          // Create data and tasks for our children
          std::vector<PreOrderTask*> tasks;
          tasks.reserve(roots.size());
          BOOST_FOREACH(const boost::shared_ptr<NODE>& root, roots)
          {
            tasks.push_back(new(allocate_child())
              PreOrderTask(root, visitorPre(root, myData), visitorPre, visitorPost, problemSizeThreshold));
          }
          // Spawn tasks
          BOOST_FOREACH(PreOrderTask* task, tasks)
            spawn(*task);
          // Wait for tasks to finish
          wait_for_all();
          // Return NULL
          return NULL;
        }
      };

      template<typename NODE, typename ROOTS, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
      RootTask<ROOTS, NODE, DATA, VISITOR_PRE, VISITOR_POST>&
        CreateRootTask(const ROOTS& roots, DATA& rootData, VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost,
        int problemSizeThreshold)
      {
        typedef RootTask<ROOTS, NODE, DATA, VISITOR_PRE, VISITOR_POST> RootTask;
        return *new(tbb::task::allocate_root()) RootTask(roots, rootData, visitorPre, visitorPost, problemSizeThreshold);
      }

      /* ************************************************************************* */
      //template<class NODE, typename DATA>
      //struct ParallelDFSData {
      //  DATA myData;
      //  FastList<ParallelTraversalNode<NODE,DATA> >& 
      //};
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
      typedef FastList<TraversalNode> Stack;
      Stack stack;
      FastList<DATA> dataList; // List to store node data as it is returned from the pre-order visitor

      // Add roots to stack (insert such that they are visited and processed in order
      {
        Stack::iterator insertLocation = stack.begin();
        BOOST_FOREACH(const sharedNode& root, forest.roots())
          stack.insert(insertLocation, TraversalNode(root, rootData));
      }

      // Traverse
      while(!stack.empty())
      {
        // Get next node
        TraversalNode& node = stack.front();

        if(node.expanded) {
          // If already expanded, then the data stored in the node is no longer needed, so visit
          // then delete it.
          (void) visitorPost(node.treeNode, *node.dataPointer);
          dataList.erase(node.dataPointer);
          stack.pop_front();
        } else {
          // If not already visited, visit the node and add its children (use reverse iterators so
          // children are processed in the order they appear)
          node.dataPointer = dataList.insert(dataList.end(), visitorPre(node.treeNode, node.parentData));
          Stack::iterator insertLocation = stack.begin();
          BOOST_FOREACH(const sharedNode& child, node.treeNode->children)
            stack.insert(insertLocation, TraversalNode(child, *node.dataPointer));
          node.expanded = true;
        }
      }
      assert(dataList.empty());
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
    void DepthFirstForestParallel(FOREST& forest, DATA& rootData, VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost,
      int problemSizeThreshold = 10)
    {
      // Typedefs
      typedef typename FOREST::Node Node;
      typedef boost::shared_ptr<Node> sharedNode;

      tbb::task::spawn_root_and_wait(CreateRootTask<Node>(
        forest.roots(), rootData, visitorPre, visitorPost, problemSizeThreshold));
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
        clone->children.clear();
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
    void PrintForest(const FOREST& forest, std::string str, const KeyFormatter& keyFormatter) {
      typedef typename FOREST::Node Node;
      DepthFirstForest(forest, str, boost::bind(PrintForestVisitorPre<Node>, _1, _2, keyFormatter));
    }
  }

}