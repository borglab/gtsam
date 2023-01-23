/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Signature.cpp
 * @brief signatures for conditional densities
 * @author Frank Dellaert
 * @date Feb 27, 2011
 */

#include <sstream>

#include "Signature.h"

#include <boost/spirit/include/qi.hpp> // for parsing
#include <boost/spirit/include/phoenix.hpp> // for qi::_val

namespace gtsam {

  using namespace std;

  namespace qi = boost::spirit::qi;
  namespace ph = boost::phoenix;

  // parser for strings of form "99/1 80/20" etc...
  namespace parser {
    typedef string::const_iterator It;
    using boost::phoenix::val;
    using boost::phoenix::ref;
    using boost::phoenix::push_back;

    // Special rows, true and false
    Signature::Row F{1, 0}, T{0, 1};  

    // Special tables (inefficient, but do we care for user input?)
    Signature::Table logic(bool ff, bool ft, bool tf, bool tt) {
      Signature::Table t(4);
      t[0] = ff ? T : F;
      t[1] = ft ? T : F;
      t[2] = tf ? T : F;
      t[3] = tt ? T : F;
      return t;
    }

    struct Grammar {
      qi::rule<It, qi::space_type, Signature::Table()> table, or_, and_, rows;
      qi::rule<It, Signature::Row()> true_, false_, row;
      Grammar() {
        table = or_ | and_ | rows;
        or_ = qi::lit("OR")[qi::_val = logic(false, true, true, true)];
        and_ = qi::lit("AND")[qi::_val = logic(false, false, false, true)];
        rows = +(row | true_ | false_);
        row = qi::double_ >> +("/" >> qi::double_);
        true_ = qi::lit("T")[qi::_val = T];
        false_ = qi::lit("F")[qi::_val = F];
      }
    } grammar;

  } // \namespace parser

  ostream& operator <<(ostream &os, const Signature::Row &row) {
    os << row[0];
    for (size_t i = 1; i < row.size(); i++)
      os << " " << row[i];
    return os;
  }

  ostream& operator <<(ostream &os, const Signature::Table &table) {
    for (size_t i = 0; i < table.size(); i++)
      os << table[i] << endl;
    return os;
  }

  Signature::Signature(const DiscreteKey& key, const DiscreteKeys& parents,
                       const Table& table)
      : key_(key), parents_(parents) {
    operator=(table);
  }

  Signature::Signature(const DiscreteKey& key, const DiscreteKeys& parents,
                       const std::string& spec)
      : key_(key), parents_(parents) {
    operator=(spec);
  }

  Signature::Signature(const DiscreteKey& key) :
    key_(key) {
  }

  DiscreteKeys Signature::discreteKeys() const {
    DiscreteKeys keys;
    keys.push_back(key_);
    for (const DiscreteKey& key : parents_) keys.push_back(key);
    return keys;
  }

  KeyVector Signature::indices() const {
    KeyVector js;
    js.push_back(key_.first);
    for (const DiscreteKey& key : parents_) js.push_back(key.first);
    return js;
  }

  vector<double> Signature::cpt() const {
    vector<double> cpt;
    if (table_) {
      const size_t nrStates = table_->at(0).size();
      for (size_t j = 0; j < nrStates; j++) {
        for (const Row& row : *table_) {
          assert(row.size() == nrStates);
          cpt.push_back(row[j]);
        }
      }
    }
    return cpt;
  }

  Signature& Signature::operator,(const DiscreteKey& parent) {
    parents_.push_back(parent);
    return *this;
  }

  static void normalize(Signature::Row& row) {
    double sum = 0;
    for (size_t i = 0; i < row.size(); i++)
      sum += row[i];
    for (size_t i = 0; i < row.size(); i++)
      row[i] /= sum;
  }

  Signature& Signature::operator=(const string& spec) {
    spec_ = spec;
    Table table;
    parser::It f = spec.begin(), l = spec.end();
    bool success =
        qi::phrase_parse(f, l, parser::grammar.table, qi::space, table);
    if (success) {
      for (Row& row : table) normalize(row);
      table_ = table;
    }
    return *this;
  }

  Signature& Signature::operator=(const Table& t) {
    Table table = t;
    for(Row& row: table)
      normalize(row);
    table_ = table;
    return *this;
  }

  ostream& operator <<(ostream &os, const Signature &s) {
    os << s.key_.first;
    if (s.parents_.empty()) {
      os << " % ";
    } else {
      os << " | " << s.parents_[0].first;
      for (size_t i = 1; i < s.parents_.size(); i++)
        os << " && " << s.parents_[i].first;
      os << " = ";
    }
    os << (s.spec_ ? *s.spec_ : "no spec") << endl;
    if (s.table_)
      os << (*s.table_);
    else
      os << "spec could not be parsed" << endl;
    return os;
  }

  Signature operator|(const DiscreteKey& key, const DiscreteKey& parent) {
    Signature s(key);
    return s, parent;
  }

  Signature operator%(const DiscreteKey& key, const string& parent) {
    Signature s(key);
    return s = parent;
  }

  Signature operator%(const DiscreteKey& key, const Signature::Table& parent) {
    Signature s(key);
    return s = parent;
  }

} // namespace gtsam
