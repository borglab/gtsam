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

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>

namespace gtsam {

/* ************************************************************************* */
std::string _multirobotKeyFormatter(Key key) {
  const LabeledSymbol asLabeledSymbol(key);
  if (asLabeledSymbol.chr() > 0 && asLabeledSymbol.label() > 0)
    return (std::string) asLabeledSymbol;

  const Symbol asSymbol(key);
  if (asLabeledSymbol.chr() > 0)
    return (std::string) asSymbol;
  else
    return boost::lexical_cast<std::string>(key);
}

/* ************************************************************************* */
template<class CONTAINER>
static void print(const CONTAINER& keys, const std::string& s,
    const KeyFormatter& keyFormatter) {
  std::cout << s << " ";
  if (keys.empty())
    std::cout << "(none)" << std::endl;
  else {
    BOOST_FOREACH(const Key& key, keys)
      std::cout << keyFormatter(key) << " ";
    std::cout << std::endl;
  }
}

/* ************************************************************************* */
void printKeyList(const KeyList& keys, const std::string& s,
    const KeyFormatter& keyFormatter) {
  print(keys, s, keyFormatter);
}
/* ************************************************************************* */
void printKeyVector(const KeyVector& keys, const std::string& s,
    const KeyFormatter& keyFormatter) {
  print(keys, s, keyFormatter);
}
/* ************************************************************************* */
void printKeySet(const KeySet& keys, const std::string& s,
    const KeyFormatter& keyFormatter) {
  print(keys, s, keyFormatter);
}
/* ************************************************************************* */

} // \namespace gtsam
