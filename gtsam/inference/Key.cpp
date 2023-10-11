/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Key.cpp
 * @brief
 * @author Richard Roberts
 * @author Alex Cunningham
 * @date Feb 20, 2012
 */

#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>

#include <iostream>

using namespace std;

namespace gtsam {

/// Assign default key formatter
KeyFormatter DefaultKeyFormatter = &_defaultKeyFormatter;

/* ************************************************************************* */
string _defaultKeyFormatter(Key key) {
  const Symbol asSymbol(key);
  if (asSymbol.chr() > 0) {
    return (string) asSymbol;
  }
  else {
    return std::to_string(key);
  }
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
  if (asLabeledSymbol.chr() > 0) {
    return (string) asSymbol;
  }
  else {
    return std::to_string(key);
  }
}

/* ************************************************************************* */
template<class CONTAINER>
void Print(const CONTAINER& keys, const string& s,
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
// Access to custom stream property.
void *&key_formatter::property(ios_base &s) {
  static int kUniqueIndex = ios_base::xalloc();
  return s.pword(kUniqueIndex);
}

/* ************************************************************************* */
// Store pointer to formatter in property.
void key_formatter::set_property(ios_base &s, const KeyFormatter &f) {
  property(s) = (void *)(&f);
}

/* ************************************************************************* */
// Get pointer to formatter from property.
KeyFormatter *key_formatter::get_property(ios_base &s) {
  return (KeyFormatter *)(property(s));
}

/* ************************************************************************* */
// Stream operator that will take a key_formatter and set the stream property.
ostream &operator<<(ostream &os, const key_formatter &m) {
  key_formatter::set_property(os, m.formatter_);
  return os;
}

/* ************************************************************************* */
// Stream operator that takes a StreamedKey and properly formats it
ostream &operator<<(ostream &os, const StreamedKey &streamedKey) {
  const KeyFormatter *formatter = key_formatter::get_property(os);
  if (formatter == nullptr) {
    formatter = &DefaultKeyFormatter;
  }
  os << (*formatter)(streamedKey.key_);
  return (os);
}

/* ************************************************************************* */

} // \namespace gtsam
