/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Ordering.h
 * @brief   Variable ordering for the elimination algorithm
 * @author  Richard Roberts
 * @author  Andrew Melim
 * @author  Frank Dellaert
 * @date    Sep 2, 2010
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/MetisIndex.h>
#include <gtsam/base/FastSet.h>

#include <algorithm>
#include <vector>

namespace gtsam {

class Ordering: public KeyVector {
protected:
  typedef KeyVector Base;

public:

  /// Type of ordering to use
  enum OrderingType {
    COLAMD, METIS, NATURAL, CUSTOM
  };

  typedef Ordering This; ///< Typedef to this class
  typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

  /// Create an empty ordering
  GTSAM_EXPORT
  Ordering() {
  }

  using KeyVector::KeyVector;  // Inherit the KeyVector's constructors

  /// Create from a container
  template<typename KEYS>
  explicit Ordering(const KEYS& keys) :
      Base(keys.begin(), keys.end()) {
  }

  /// Add new variables to the ordering as
  /// `ordering += key1, key2, ...`.
  This& operator+=(Key key);

  /// Overloading the comma operator allows for chaining appends
  // e.g. keys += key1, key2
  This& operator,(Key key);

  /**
   * @brief Append new keys to the ordering as `ordering += keys`.
   *
   * @param keys The key vector to append to this ordering.
   * @return The ordering variable with appended keys.
   */
  This& operator+=(KeyVector& keys);

  /// Check if key exists in ordering.
  bool contains(const Key& key) const;

  /**
   * @brief Invert (not reverse) the ordering - returns a map from key to order
   * position.
   *
   * @return FastMap<Key, size_t>
   */
  FastMap<Key, size_t> invert() const;

  /// @name Fill-reducing Orderings
  /// @{

  /// Compute a fill-reducing ordering using COLAMD from a factor graph (see details for note on
  /// performance). This internally builds a VariableIndex so if you already have a VariableIndex,
  /// it is faster to use COLAMD(const VariableIndex&)
  template<class FACTOR_GRAPH>
  static Ordering Colamd(const FACTOR_GRAPH& graph) {
    if (graph.empty())
      return Ordering();
    else
      return Colamd(VariableIndex(graph));
  }

  /// Compute a fill-reducing ordering using COLAMD from a VariableIndex.
  static GTSAM_EXPORT Ordering Colamd(const VariableIndex& variableIndex);

  /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
  /// for note on performance).  This internally builds a VariableIndex so if you already have a
  /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  This function constrains
  /// the variables in \c constrainLast to the end of the ordering, and orders all other variables
  /// before in a fill-reducing ordering.  If \c forceOrder is true, the variables in \c
  /// constrainLast will be ordered in the same order specified in the KeyVector \c
  /// constrainLast.   If \c forceOrder is false, the variables in \c constrainLast will be
  /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
  template<class FACTOR_GRAPH>
  static Ordering ColamdConstrainedLast(const FACTOR_GRAPH& graph,
      const KeyVector& constrainLast, bool forceOrder = false) {
    if (graph.empty())
      return Ordering();
    else
      return ColamdConstrainedLast(VariableIndex(graph), constrainLast, forceOrder);
  }

  /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  This
  /// function constrains the variables in \c constrainLast to the end of the ordering, and orders
  /// all other variables before in a fill-reducing ordering.  If \c forceOrder is true, the
  /// variables in \c constrainLast will be ordered in the same order specified in the KeyVector
  /// \c constrainLast.   If \c forceOrder is false, the variables in \c constrainLast will be
  /// ordered after all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
  static GTSAM_EXPORT Ordering ColamdConstrainedLast(
      const VariableIndex& variableIndex, const KeyVector& constrainLast,
      bool forceOrder = false);

  /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
  /// for note on performance).  This internally builds a VariableIndex so if you already have a
  /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  This function constrains
  /// the variables in \c constrainLast to the end of the ordering, and orders all other variables
  /// before in a fill-reducing ordering.  If \c forceOrder is true, the variables in \c
  /// constrainFirst will be ordered in the same order specified in the KeyVector \c
  /// constrainFirst.   If \c forceOrder is false, the variables in \c constrainFirst will be
  /// ordered before all the others, but will be rearranged by CCOLAMD to reduce fill-in as well.
  template<class FACTOR_GRAPH>
  static Ordering ColamdConstrainedFirst(const FACTOR_GRAPH& graph,
      const KeyVector& constrainFirst, bool forceOrder = false) {
    if (graph.empty())
      return Ordering();
    else
      return ColamdConstrainedFirst(VariableIndex(graph), constrainFirst, forceOrder);
  }

  /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  This
  /// function constrains the variables in \c constrainFirst to the front of the ordering, and
  /// orders all other variables after in a fill-reducing ordering.  If \c forceOrder is true, the
  /// variables in \c constrainFirst will be ordered in the same order specified in the
  /// KeyVector \c constrainFirst.   If \c forceOrder is false, the variables in \c
  /// constrainFirst will be ordered before all the others, but will be rearranged by CCOLAMD to
  /// reduce fill-in as well.
  static GTSAM_EXPORT Ordering ColamdConstrainedFirst(
      const VariableIndex& variableIndex,
      const KeyVector& constrainFirst, bool forceOrder = false);

  /// Compute a fill-reducing ordering using constrained COLAMD from a factor graph (see details
  /// for note on performance).  This internally builds a VariableIndex so if you already have a
  /// VariableIndex, it is faster to use COLAMD(const VariableIndex&).  In this function, a group
  /// for each variable should be specified in \c groups, and each group of variables will appear
  /// in the ordering in group index order.  \c groups should be a map from Key to group index.
  /// The group indices used should be consecutive starting at 0, but may appear in \c groups in
  /// arbitrary order.  Any variables not present in \c groups will be assigned to group 0.  This
  /// function simply fills the \c cmember argument to CCOLAMD with the supplied indices, see the
  /// CCOLAMD documentation for more information.
  template<class FACTOR_GRAPH>
  static Ordering ColamdConstrained(const FACTOR_GRAPH& graph,
      const FastMap<Key, int>& groups) {
    if (graph.empty())
      return Ordering();
    else
      return ColamdConstrained(VariableIndex(graph), groups);
  }

  /// Compute a fill-reducing ordering using constrained COLAMD from a VariableIndex.  In this
  /// function, a group for each variable should be specified in \c groups, and each group of
  /// variables will appear in the ordering in group index order.  \c groups should be a map from
  /// Key to group index. The group indices used should be consecutive starting at 0, but may
  /// appear in \c groups in arbitrary order.  Any variables not present in \c groups will be
  /// assigned to group 0.  This function simply fills the \c cmember argument to CCOLAMD with the
  /// supplied indices, see the CCOLAMD documentation for more information.
  static GTSAM_EXPORT Ordering ColamdConstrained(
      const VariableIndex& variableIndex, const FastMap<Key, int>& groups);

  /// Return a natural Ordering. Typically used by iterative solvers
  template<class FACTOR_GRAPH>
  static Ordering Natural(const FACTOR_GRAPH &fg) {
    KeySet src = fg.keys();
    KeyVector keys(src.begin(), src.end());
    std::stable_sort(keys.begin(), keys.end());
    return Ordering(keys.begin(), keys.end());
  }

  /// METIS Formatting function
  template<class FACTOR_GRAPH>
  static GTSAM_EXPORT void CSRFormat(std::vector<int>& xadj,
      std::vector<int>& adj, const FACTOR_GRAPH& graph);

  /// Compute an ordering determined by METIS from a VariableIndex
  static GTSAM_EXPORT Ordering Metis(const MetisIndex& met);

  template<class FACTOR_GRAPH>
  static Ordering Metis(const FACTOR_GRAPH& graph) {
    if (graph.empty())
      return Ordering();
    else
      return Metis(MetisIndex(graph));
  }

  /// @}

  /// @name Named Constructors
  /// @{

  template<class FACTOR_GRAPH>
  static Ordering Create(OrderingType orderingType,
      const FACTOR_GRAPH& graph) {
    if (graph.empty())
      return Ordering();

    switch (orderingType) {
    case COLAMD:
      return Colamd(graph);
    case METIS:
      return Metis(graph);
    case NATURAL:
      return Natural(graph);
    case CUSTOM:
      throw std::runtime_error(
          "Ordering::Create error: called with CUSTOM ordering type.");
    default:
      throw std::runtime_error(
          "Ordering::Create error: called with unknown ordering type.");
    }
  }

  /// @}

  /// @name Testable
  /// @{

  GTSAM_EXPORT
  void print(const std::string& str = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  GTSAM_EXPORT
  bool equals(const Ordering& other, double tol = 1e-9) const;

  /// @}

private:
  /// Internal COLAMD function
  static GTSAM_EXPORT Ordering ColamdConstrained(
      const VariableIndex& variableIndex, std::vector<int>& cmember);

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

/// traits
template<> struct traits<Ordering> : public Testable<Ordering> {
};

}

