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
   * The string specifies a conditional probability table in 00 01 10 11 order.
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
   *   (E|T,L) = "F F F 1"
   *   X|E = "95/5 2/98"
   *   (D|E,B) = "9/1 2/8 3/7 1/9"
   *
   * @ingroup discrete
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
   /**
    * Construct from key, parents, and a Signature::Table specifying the
    * conditional probability table (CPT) in 00 01 10 11 order. For
    * three-valued, it would be 00 01 02 10 11 12 20 21 22, etc....
    *
    * The first string is parsed to add a key and parents.
    *
    * Example:
    *   Signature::Table table{{0.9, 0.1}, {0.2, 0.8}, {0.3, 0.7}, {0.1, 0.9}};
    *   Signature sig(D, {E, B}, table);
    */
   Signature(const DiscreteKey& key, const DiscreteKeys& parents,
             const Table& table);

   /**
    * Construct from key, parents, and a string specifying the conditional
    * probability table (CPT) in 00 01 10 11 order. For three-valued, it would
    * be 00 01 02 10 11 12 20 21 22, etc....
    *
    * The first string is parsed to add a key and parents. The second string
    * parses into a table.
    *
    * Example (same CPT as above):
    *   Signature sig(D, {B,E}, "9/1 2/8 3/7 1/9");
    */
   Signature(const DiscreteKey& key, const DiscreteKeys& parents,
             const std::string& spec);

   /**
    * Construct from a single DiscreteKey.
    *
    * The resulting signature has no parents or CPT table. Typical use then
    * either adds parents with | and , operators below, or assigns a table with
    * operator=().
    */
   Signature(const DiscreteKey& key);

   /** the variable key */
   const DiscreteKey& key() const { return key_; }

   /** the parent keys */
   const DiscreteKeys& parents() const { return parents_; }

   /** All keys, with variable key first */
   DiscreteKeys discreteKeys() const;

   /** All key indices, with variable key first */
   KeyVector indices() const;

   // the CPT as parsed, if successful
   const boost::optional<Table>& table() const { return table_; }

   // the CPT as a vector of doubles, with key's values most rapidly changing
   std::vector<double> cpt() const;

   /** Add a parent */
   Signature& operator,(const DiscreteKey& parent);

   /** Add the CPT spec */
   Signature& operator=(const std::string& spec);

   /** Add the CPT spec directly as a table */
   Signature& operator=(const Table& table);

   /** provide streaming */
   GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                                const Signature& s);
  };

  /**
   * Helper function to create Signature objects
   * example: Signature s = D | E;
   */
  GTSAM_EXPORT Signature operator|(const DiscreteKey& key, const DiscreteKey& parent);

  /**
   * Helper function to create Signature objects
   * example: Signature s(D % "99/1");
   */
  GTSAM_EXPORT Signature operator%(const DiscreteKey& key, const std::string& parent);

  /**
   * Helper function to create Signature objects, using table construction directly
   * example: Signature s(D % table);
   */
  GTSAM_EXPORT Signature operator%(const DiscreteKey& key, const Signature::Table& parent);

}
