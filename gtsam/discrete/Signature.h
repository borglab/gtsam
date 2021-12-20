/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Signature.h
 * @brief signatures for conditional densities
 * @author Frank Dellaert
 * @date Feb 27, 2011
 */

#pragma once
#include <string>
#include <vector>
#include <boost/optional.hpp>
#include <gtsam/discrete/DiscreteKey.h>

namespace gtsam {

  /**
   * Signature for a discrete conditional density, used to construct conditionals.
   *
   * The format is (Key % string) for nodes with no parents,
   * and (Key | Key, Key = string) for nodes with parents.
   *
   * The string specifies a conditional probability spec in the 00 01 10 11 order.
   * For three-valued, it would be 00 01 02 10 11 12 20 21 22, etc...
   *
   * For example, given the following keys
   *
   *   Key A("Asia"), S("Smoking"), T("Tuberculosis"), L("LungCancer"),
   *   B("Bronchitis"), E("Either"), X("XRay"), D("Dyspnoea");
   *
   * These are all valid signatures (Asia network example):
   *
   *   A % "99/1"
   *   S % "50/50"
   *   T|A = "99/1  95/5"
   *   L|S = "99/1  90/10"
   *   B|S = "70/30  40/60"
   *   E|T,L = "F F F 1"
   *   X|E = "95/5 2/98"
   *   D|E,B = "9/1 2/8 3/7 1/9"
   */
  class GTSAM_EXPORT Signature {

  public:

    /** Data type for the CPT */
    typedef std::vector<double> Row;
    typedef std::vector<Row> Table;

  private:

    /** the variable key */
    DiscreteKey key_;

    /** the parent keys */
    DiscreteKeys parents_;

    // the given CPT specification string
    boost::optional<std::string> spec_;

    // the CPT as parsed, if successful
    boost::optional<Table> table_;

  public:

    /** Constructor from DiscreteKey */
    Signature(const DiscreteKey& key);

    /** the variable key */
    const DiscreteKey& key() const {
      return key_;
    }

    /** the parent keys */
    const DiscreteKeys& parents() const {
      return parents_;
    }

    /** All keys, with variable key first */
    DiscreteKeys discreteKeys() const;

    /** All key indices, with variable key first */
    KeyVector indices() const;

    // the CPT as parsed, if successful
    const boost::optional<Table>& table() const {
      return table_;
    }

    // the CPT as a vector of doubles, with key's values most rapidly changing
    std::vector<double> cpt() const;

    /** Add a parent */
    Signature& operator,(const DiscreteKey& parent);

    /** Add the CPT spec - Fails in boost 1.40 */
    Signature& operator=(const std::string& spec);

    /** Add the CPT spec directly as a table */
    Signature& operator=(const Table& table);

    /** provide streaming */
    GTSAM_EXPORT friend std::ostream& operator <<(std::ostream &os, const Signature &s);
  };

  /**
   * Helper function to create Signature objects
   * example: Signature s = D | E;
   */
  GTSAM_EXPORT Signature operator|(const DiscreteKey& key, const DiscreteKey& parent);

  /**
   * Helper function to create Signature objects
   * example: Signature s(D % "99/1");
   * Uses string parser, which requires BOOST 1.42 or higher
   */
  GTSAM_EXPORT Signature operator%(const DiscreteKey& key, const std::string& parent);

  /**
   * Helper function to create Signature objects, using table construction directly
   * example: Signature s(D % table);
   */
  GTSAM_EXPORT Signature operator%(const DiscreteKey& key, const Signature::Table& parent);

}
