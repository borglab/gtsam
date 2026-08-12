/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LabeledSymbol.h
 * @date Jan 12, 2010
 * @author: Alex Cunningham
 */

#include <gtsam/inference/LabeledSymbol.h>

#include <iostream>
#include <limits>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol()
:  c_(0), label_(0), j_(0) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(const LabeledSymbol& key)
: c_(key.c_), label_(key.label_), j_(key.j_) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(unsigned char c, unsigned char label, std::uint64_t j)
: c_(c), label_(label), j_(j) {}

/* ************************************************************************* */
LabeledSymbol::LabeledSymbol(gtsam::Key key) {
  const size_t keyBits = sizeof(gtsam::Key) * 8;
  const size_t chrBits = sizeof(unsigned char) * 8;
  const size_t lblBits = sizeof(unsigned char) * 8;
  const size_t indexBits = keyBits - chrBits - lblBits;
  const gtsam::Key chrMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << (indexBits + lblBits);
  const gtsam::Key lblMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << indexBits;
  const gtsam::Key indexMask = ~(chrMask | lblMask);
  c_ = (unsigned char)((key & chrMask) >> (indexBits + lblBits));
  label_ = (unsigned char)((key & lblMask) >> indexBits);
  j_ = key & indexMask;
}

/* ************************************************************************* */
LabeledSymbol::operator gtsam::Key() const {
  const size_t keyBits = sizeof(gtsam::Key) * 8;
  const size_t chrBits = sizeof(unsigned char) * 8;
  const size_t lblBits = sizeof(unsigned char) * 8;
  const size_t indexBits = keyBits - chrBits - lblBits;
  const gtsam::Key chrMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << (indexBits + lblBits);
  const gtsam::Key lblMask = gtsam::Key(std::numeric_limits<unsigned char>::max()) << indexBits;
  const gtsam::Key indexMask = ~(chrMask | lblMask);
  if(j_ > indexMask)
    throw std::invalid_argument("Symbol index is too large");
  gtsam::Key key = (gtsam::Key(c_) << (indexBits + lblBits)) | (gtsam::Key(label_) << indexBits) | j_;
  return key;
}

/* ************************************************************************* */
void LabeledSymbol::print(const std::string& s) const {
  std::cout << s << ": " << (std::string) (*this) << std::endl;
}

/* ************************************************************************* */
LabeledSymbol::operator std::string() const {
  char buffer[100];
  snprintf(buffer, 100, "%c%c%llu", c_, label_,
           static_cast<unsigned long long>(j_));
  return std::string(buffer);
}

/* ************************************************************************* */
bool LabeledSymbol::operator<(const LabeledSymbol& comp) const {
  return c_ < comp.c_
      || (comp.c_ == c_ && label_ < comp.label_)
      || (comp.c_ == c_ && comp.label_ == label_ && j_ < comp.j_);
}

/* ************************************************************************* */
bool LabeledSymbol::operator==(const LabeledSymbol& comp) const {
  return comp.c_ == c_ && comp.label_ == label_ && comp.j_ == j_;
}

/* ************************************************************************* */
bool LabeledSymbol::operator!=(const LabeledSymbol& comp) const {
  return comp.c_ != c_ || comp.label_ != label_ || comp.j_ != j_;
}

/* ************************************************************************* */
bool LabeledSymbol::operator==(gtsam::Key comp) const {
  return comp == (gtsam::Key)(*this);
}

/* ************************************************************************* */
bool LabeledSymbol::operator!=(gtsam::Key comp) const {
  return comp != (gtsam::Key)(*this);
}

/* ************************************************************************* */
static LabeledSymbol make(gtsam::Key key) { return LabeledSymbol(key);}

std::function<bool(gtsam::Key)> LabeledSymbol::TypeTest(unsigned char c) {
  // Use lambda function to check equality
  auto equals = [](unsigned char s, unsigned char c) { return s == c; };
  return std::bind(
      equals,
      std::bind(&LabeledSymbol::chr, std::bind(make, std::placeholders::_1)),
      c);
}

std::function<bool(gtsam::Key)> LabeledSymbol::LabelTest(unsigned char label) {
  // Use lambda function to check equality
  auto equals = [](unsigned char s, unsigned char c) { return s == c; };
  return std::bind(
      equals,
      std::bind(&LabeledSymbol::label, std::bind(make, std::placeholders::_1)),
      label);
}

std::function<bool(gtsam::Key)> LabeledSymbol::TypeLabelTest(unsigned char c, unsigned char label) {
  // Use lambda functions for && and ==
  auto logical_and = [](bool is_type, bool is_label) { return is_type == is_label; };
  auto equals = [](unsigned char s, unsigned char c) { return s == c; };
  return std::bind(logical_and,
                   std::bind(equals,
                             std::bind(&LabeledSymbol::chr,
                                       std::bind(make, std::placeholders::_1)),
                             c),
                   std::bind(equals,
                             std::bind(&LabeledSymbol::label,
                                       std::bind(make, std::placeholders::_1)),
                             label));
}

/* ************************************************************************* */
GTSAM_EXPORT std::ostream &operator<<(std::ostream &os, const LabeledSymbol &symbol) {
  os << StreamedKey(symbol);
  return os;
}

/* ************************************************************************* */

} // \namespace gtsam

