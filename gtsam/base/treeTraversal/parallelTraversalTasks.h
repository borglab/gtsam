/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
* @file    parallelTraversalTasks.h
* @author  Richard Roberts
* @date    April 9, 2013
*/
#pragma once

#include <gtsam/global_includes.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task.h>               // tbb::task, tbb::task_list
#include <tbb/scalable_allocator.h> // tbb::scalable_allocator

namespace gtsam {

  /** Internal functions used for traversing trees */
  namespace treeTraversal {

    namespace internal {

      /* ************************************************************************* */
      template<typename NODE, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
      class PreOrderTask : public tbb::task
      {
      public:
        const boost::shared_ptr<NODE>& treeNode;
        boost::shared_ptr<DATA> myData;
        VISITOR_PRE& visitorPre;
        VISITOR_POST& visitorPost;
        int problemSizeThreshold;
        bool makeNewTasks;

        bool isPostOrderPhase;

        PreOrderTask(const boost::shared_ptr<NODE>& treeNode, const boost::shared_ptr<DATA>& myData,
                     VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost, int problemSizeThreshold,
                     bool makeNewTasks = true)
            : treeNode(treeNode),
              myData(myData),
              visitorPre(visitorPre),
              visitorPost(visitorPost),
              problemSizeThreshold(problemSizeThreshold),
              makeNewTasks(makeNewTasks),
              isPostOrderPhase(false) {}

        tbb::task* execute()
        {
          if(isPostOrderPhase)
          {
            // Run the post-order visitor since this task was recycled to run the post-order visitor
            (void) visitorPost(treeNode, *myData);
            return nullptr;
          }
          else
          {
            if(makeNewTasks)
            {
              if(!treeNode->children.empty())
              {
                // Allocate post-order task as a continuation
                isPostOrderPhase = true;
                recycle_as_continuation();

                bool overThreshold = (treeNode->problemSize() >= problemSizeThreshold);

                tbb::task* firstChild = 0;
                tbb::task_list childTasks;
                for(const boost::shared_ptr<NODE>& child: treeNode->children)
                {
                  // Process child in a subtask.  Important:  Run visitorPre before calling
                  // allocate_child so that if visitorPre throws an exception, we will not have
                  // allocated an extra child, this causes a TBB error.
                  boost::shared_ptr<DATA> childData = boost::allocate_shared<DATA>(
                      tbb::scalable_allocator<DATA>(), visitorPre(child, *myData));
                  tbb::task* childTask =
                      new (allocate_child()) PreOrderTask(child, childData, visitorPre, visitorPost,
                                                          problemSizeThreshold, overThreshold);
                  if (firstChild)
                    childTasks.push_back(*childTask);
                  else
                    firstChild = childTask;
                }

                // If we have child tasks, start subtasks and wait for them to complete
                set_ref_count((int)treeNode->children.size());
                spawn(childTasks);
                return firstChild;
              }
              else
              {
                // Run the post-order visitor in this task if we have no children
                (void) visitorPost(treeNode, *myData);
                return nullptr;
              }
            }
            else
            {
              // Process this node and its children in this task
              processNodeRecursively(treeNode, *myData);
              return nullptr;
            }
          }
        }

        void processNodeRecursively(const boost::shared_ptr<NODE>& node, DATA& myData)
        {
          for(const boost::shared_ptr<NODE>& child: node->children)
          {
            DATA childData = visitorPre(child, myData);
            processNodeRecursively(child, childData);
          }

          // Run the post-order visitor
          (void) visitorPost(node, myData);
        }
      };

      /* ************************************************************************* */
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
          // Create data and tasks for our children
          tbb::task_list tasks;
          for(const boost::shared_ptr<NODE>& root: roots)
          {
            boost::shared_ptr<DATA> rootData = boost::allocate_shared<DATA>(tbb::scalable_allocator<DATA>(), visitorPre(root, myData));
            tasks.push_back(*new(allocate_child())
              PreOrderTask(root, rootData, visitorPre, visitorPost, problemSizeThreshold));
          }
          // Set TBB ref count
          set_ref_count(1 + (int) roots.size());
          // Spawn tasks
          spawn_and_wait_for_all(tasks);
          // Return nullptr
          return nullptr;
        }
      };

      template<typename NODE, typename ROOTS, typename DATA, typename VISITOR_PRE, typename VISITOR_POST>
      RootTask<ROOTS, NODE, DATA, VISITOR_PRE, VISITOR_POST>&
        CreateRootTask(const ROOTS& roots, DATA& rootData, VISITOR_PRE& visitorPre, VISITOR_POST& visitorPost, int problemSizeThreshold)
      {
          typedef RootTask<ROOTS, NODE, DATA, VISITOR_PRE, VISITOR_POST> RootTask;
          return *new(tbb::task::allocate_root()) RootTask(roots, rootData, visitorPre, visitorPost, problemSizeThreshold);
        }

    }

  }

}

#endif
