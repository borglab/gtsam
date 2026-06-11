/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ForestTraversal.h
 * @brief Forest traversal helpers with optional TBB acceleration.
 *
 * @details
 * Provides top-down and bottom-up traversal helpers that either enqueue work
 * on an internal `TaskScheduler` (for builds without TBB) or call the
 * `treeTraversal` parallel helpers when `GTSAM_USE_TBB` is enabled.
 *
 * @note `Forest::roots()` or `Forest::roots` must return a range of
 * pointer-like `Node` roots.
 * @note `Node::children()` or `Node::children` must return a range of
 * pointer-like `Node` children.
 *
 * @author Frank Dellaert
 * @date May, 2025
 */

#pragma once

#include <gtsam/config.h>
#ifdef GTSAM_USE_TBB
#include <gtsam/base/treeTraversal-inst.h>
#include <gtsam/base/types.h>
#include <tbb/global_control.h>
#else
#include <gtsam/base/TaskScheduler.h>
#endif

#include <atomic>
#include <cassert>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>

namespace gtsam {

/**
 * @brief Mixin that provides depth-based top-down or bottom-up traversal.
 *
 * @details
 * When TBB is present, the traversal delegates directly to
 * `treeTraversal::DepthFirstForestParallel` and
 * `treeTraversal::PostOrderForestParallel` with the configured parallel
 * thresholds. Otherwise it falls back to the priority-queued implementation
 * that mirrors `TaskMixin`.
 */
template <typename Forest, typename Node>
class ForestTraversal {
 private:
  size_t threadCount_;
  using SharedNode = std::shared_ptr<Node>;

 public:
  /// Construct a helper with a fixed thread budget (used by TBB when enabled).
  explicit ForestTraversal(
      size_t numThreads = std::thread::hardware_concurrency())
      : threadCount_(numThreads == 0 ? 1 : numThreads)
#ifndef GTSAM_USE_TBB
        ,
        scheduler_(threadCount_)
#endif
  {
  }

#ifdef GTSAM_USE_TBB
  template <typename Fn>
  /// Depth-first traversal using a pre-order visitor (TBB path).
  void runTopDown(Fn fn, int parallelThreshold = 10) {
    withTbbTraversalControl([&] {
      // The pre-order visitor runs before visiting children; parallel task
      // scheduling is handled by treeTraversal helpers.
      struct VisitorPre {
        Fn* fn;
        int operator()(const SharedNode& node, int&) const {
          if (node) std::invoke(*fn, *node);
          return 0;
        }
      };

      int rootData = 0;
      VisitorPre visitor{&fn};
      auto visitorPost = [](const SharedNode&, int) {};
      treeTraversal::DepthFirstForestParallel(static_cast<Forest&>(*this),
                                              rootData, visitor, visitorPost,
                                              parallelThreshold);
    });
  }

  template <typename Fn>
  /// Post-order traversal using a bottom-up visitor (TBB path).
  void runBottomUp(Fn fn, int parallelThreshold = 10) {
    withTbbTraversalControl([&] {
      // The bottom-up visitor runs after all children are processed;
      // treeTraversal helpers orchestrate the parallelism.
      struct VisitorPost {
        Fn* fn;
        void operator()(const SharedNode& node) const {
          if (node) std::invoke(*fn, *node);
        }
      };

      VisitorPost visitor{&fn};
      treeTraversal::PostOrderForestParallel(static_cast<Forest&>(*this),
                                             visitor, parallelThreshold);
    });
  }

 private:
  /// Run a traversal with TBB and OpenMP concurrency limited to `threadCount_`.
  template <typename Body>
  void withTbbTraversalControl(Body&& body) {
    // Set a cap on TBB threads and enter an OpenMP-compatible scope,
    // then execute the provided traversal body.
    tbb::global_control control(tbb::global_control::max_allowed_parallelism,
                                static_cast<int>(threadCount_));
    TbbOpenMPMixedScope threadLimiter;
    std::forward<Body>(body)();
  }

#else

  /// Scheduler-based top-down traversal.
  template <typename Fn>
  void runTopDown(Fn fn, int parallelThreshold = 10) {
    const auto& roots = getRoots();
    if (roots.empty()) return;
    // Create shared traversal state and run from all roots.
    State state;
    state.runTraversal([&]() {
      for (const auto& root : roots) {
        assert(root);
        Frame<Fn> frame{&scheduler_, *root, 0, fn, parallelThreshold, &state};
        frame.topDownDispatch();
      }
    });
  }

  /// Scheduler-based bottom-up traversal.
  template <typename Fn>
  void runBottomUp(Fn fn, int parallelThreshold = 10) {
    const auto& roots = getRoots();
    if (roots.empty()) return;
    // Create shared traversal state and run from all roots.
    State state;
    state.runTraversal([&]() {
      for (const auto& root : roots) {
        assert(root);
        Frame<Fn> frame{&scheduler_, *root, 0, fn, parallelThreshold, &state};
        frame.bottomUpAsync([] {});
      }
    });
  }

 private:
  TaskScheduler<void> scheduler_;

  /// Return forest roots via method or field.
  decltype(auto) getRoots() const {
    const Forest& forest = static_cast<const Forest&>(*this);
    if constexpr (std::is_member_function_pointer_v<decltype(&Forest::roots)>) {
      return forest.roots();
    } else {
      return (forest.roots);
    }
  }

  /// Shared traversal state across scheduled tasks.
  struct State {
    std::atomic<int> pending{0};
    std::atomic_flag exceptionClaim = ATOMIC_FLAG_INIT;
    std::atomic<bool> hasException{false};
    std::exception_ptr exception;
    std::promise<void> done;

    /// Increment the pending counter.
    inline int incrementPending() {
      return pending.fetch_add(1, std::memory_order_relaxed);
    }

    /// Decrement the pending counter.
    inline int decrementPending() {
      return pending.fetch_sub(1, std::memory_order_relaxed);
    }

    /// Run a scheduler-based traversal with a fresh state and roots.
    template <typename Body>
    void runTraversal(Body&& body) {
      std::future<void> future = done.get_future();

      // Seed pending count for the overall traversal scope, invoke body,
      // then mark this seed work item as finished.
      incrementPending();
      std::forward<Body>(body)();
      maybeFinish();

      // Block until traversal resolves (success or exception propagation).
      future.get();
    }

    /// Record the first exception.
    /// Uses an atomic flag to win a single claim and set shared exception.
    void recordException(std::exception_ptr e) {
      if (!exceptionClaim.test_and_set(std::memory_order_acq_rel)) {
        exception = e;
        hasException.store(true, std::memory_order_release);
      }
    }

    /// Resolve traversal completion once pending reaches zero.
    /// When the last task finishes, fulfill the promise or set the exception.
    void maybeFinish() {
      if (decrementPending() == 1) {
        if (hasException.load(std::memory_order_acquire)) {
          try {
            done.set_exception(exception);
          } catch (...) { /* ignore */
          }
        } else {
          try {
            done.set_value();
          } catch (...) { /* ignore */
          }
        }
      }
    }
  };
  using DoneFn = std::function<void()>;

  /// RAII guard: marks one scheduled task as finished on scope exit.
  struct MaybeFinish {
    State* state;
    ~MaybeFinish() { state->maybeFinish(); }
  };

  /// Per-node traversal frame to avoid threading many parameters.
  template <typename Fn>
  struct Frame {
    TaskScheduler<void>* scheduler;
    Node& node;
    int depth;
    const Fn& fn;
    int threshold;
    State* state;

    /// Return node children via method or field.
    auto& getChildren() const {
      if constexpr (std::is_member_function_pointer<
                        decltype(&Node::children)>::value) {
        return node.children();
      } else {
        return node.children;
      }
    }

    /// Return true iff node should be processed as a scheduled "parallel" task.
    /// Decide scheduling based on `node.problemSize()` vs. `threshold`.
    bool shouldParallelize() const {
      if (threshold <= 0) {
        return true;
      } else {
        return static_cast<int>(node.problemSize()) >= threshold;
      }
    }

    /// Top-down dispatch: choose async or inline based on threshold.
    inline void topDownDispatch() const {
      if (shouldParallelize()) {
        topDownAsync();
      } else {
        topDownTraverse();
      }
    }

    /// Enqueue node work; upon completion, dispatch children traversal.
    inline void topDownAsync() const {
      auto task = [frame = *this]() {
        MaybeFinish finish{frame.state};
        frame.topDownTraverse();
      };
      /// Schedule a task and increment the pending counter.
      state->incrementPending();
      scheduler->enqueue(std::function<void()>(std::move(task)));
    }

    /// Inline node work followed by traversal of children.
    inline void topDownTraverse() const {
      if (state->hasException.load(std::memory_order_acquire))
        return;  // Keep draining without doing new work.
      try {
        std::invoke(fn, node);
        if (!state->hasException.load(std::memory_order_acquire)) {
          auto&& children = getChildren();
          for (const auto& child : children) {
            assert(child);
            Frame childFrame{scheduler, *child,    depth + 1,
                             fn,        threshold, state};
            childFrame.topDownDispatch();
          }
        }
      } catch (...) {
        state->recordException(std::current_exception());
      }
    }

    /// Recurse children; complete this node after the last child finishes.
    inline void bottomUpAsync(const DoneFn& onDone) const {
      auto&& children = getChildren();
      if (children.empty()) {
        completeBottomUpNode(onDone);
      } else {
        auto remaining = std::make_shared<std::atomic<int> >(
            static_cast<int>(children.size()));
        std::function<void()> childDone = [frame = *this, remaining,
                                           onDone]() mutable {
          if (remaining->fetch_sub(1, std::memory_order_relaxed) == 1) {
            frame.completeBottomUpNode(onDone);
          }
        };

        for (const auto& child : children) {
          assert(child);
          Frame childFrame{scheduler, *child, depth + 1, fn, threshold, state};
          childFrame.bottomUpAsync(childDone);
        }
      }
    }

    /// Execute node work after children; schedule if above threshold.
    inline void completeBottomUpNode(const DoneFn& onDone) const {
      if (state->hasException.load(std::memory_order_acquire)) {
        callOnDone(onDone);
      } else if (shouldParallelize()) {
        scheduleBottomUpNode(onDone);
      } else {
        bottomUpWork(onDone);
      }
    }

    /// Enqueue bottom-up node work, then invoke the continuation.
    inline void scheduleBottomUpNode(const DoneFn& onDone) const {
      auto task = [frame = *this, onDone]() mutable {
        MaybeFinish finish{frame.state};
        if (frame.state->hasException.load(std::memory_order_acquire)) {
          frame.callOnDone(onDone);
        } else {
          frame.bottomUpWork(onDone);
        }
      };

      // Each scheduled task increments the pending counter.
      state->incrementPending();

      // Schedule a continuation or run it inline if already on a worker thread.
      scheduler->enqueueOrRunInline(std::function<void()>(std::move(task)));
    }

    /// Do bottom-up node work, then invoke the continuation.
    inline void bottomUpWork(const DoneFn& onDone) const {
      try {
        std::invoke(fn, node);
      } catch (...) {
        state->recordException(std::current_exception());
      }
      callOnDone(onDone);
    }

    /// Invoke an `onDone` callback and record any exception it throws.
    void callOnDone(const DoneFn& onDone) const {
      try {
        onDone();
      } catch (...) {
        state->recordException(std::current_exception());
      }
    }

  };  // end Frame

#endif
};

}  // namespace gtsam
