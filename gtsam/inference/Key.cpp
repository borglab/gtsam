/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Key.h
 * @brief
 * @author Richard Roberts
 * @author Alex Cunningham
 * @date Feb 20, 2012
 */

#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>

#include <boost/lexical_cast.hpp>
#include <iostream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
string _defaultKeyFormatter(Key key) {
  const Symbol asSymbol(key);
  if (asSymbol.chr() > 0)
    return (string) asSymbol;
  else
    return boost::lexical_cast<string>(key);
}

/* ************************************************************************* */
void PrintKey(Key key, const string& s, const KeyFormatter& keyFormatter) {
  cout << s << keyFormatter(key);
}

/* ************************************************************************* */
string _multirobotKeyFormatter(Key key) {
  const LabeledSymbol asLabeledSymbol(key);
  if (asLabeledSymbol.chr() > 0 && asLabeledSymbol.label() > 0)
    return (string) asLabeledSymbol;

  const Symbol asSymbol(key);
  if (asLabeledSymbol.chr() > 0)
    return (string) asSymbol;
  else
    return boost::lexical_cast<string>(key);
}

/* ************************************************************************* */
template<class CONTAINER>
static void Print(const CONTAINER& keys, const string& s,
    const KeyFormatter& keyFormatter) {
  cout << s << " ";
  if (keys.empty())
    cout << "(none)" << endl;
  else {
    for(const Key& key: keys)
      cout << keyFormatter(key) << " ";
    cout << endl;
  }
}

/* ************************************************************************* */
void PrintKeyList(const KeyList& keys, const string& s,
    const KeyFormatter& keyFormatter) {
  Print(keys, s, keyFormatter);
}
/* ************************************************************************* */
void PrintKeyVector(const KeyVector& keys, const string& s,
    const KeyFormatter& keyFormatter) {
  Print(keys, s, keyFormatter);
}
/* ************************************************************************* */
void PrintKeySet(const KeySet& keys, const string& s,
    const KeyFormatter& keyFormatter) {
  Print(keys, s, keyFormatter);
}
/* ************************************************************************* */

} // \namespace gtsam
