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

#include <iostream>

#include <boost/format.hpp>
#include <boost/bind.hpp>

#include <boost/lexical_cast.hpp>

#include <gtsam/inference/LabeledSymbol.h>

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
  return str(boost::format("%c%c%d") % c_ % label_ % j_);
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

boost::function<bool(gtsam::Key)> LabeledSymbol::TypeTest(unsigned char c) {
  return boost::bind(&LabeledSymbol::chr, boost::bind(make, _1)) == c;
}

boost::function<bool(gtsam::Key)> LabeledSymbol::LabelTest(unsigned char label) {
  return boost::bind(&LabeledSymbol::label, boost::bind(make, _1)) == label;
}

boost::function<bool(gtsam::Key)> LabeledSymbol::TypeLabelTest(unsigned char c, unsigned char label) {
  return boost::bind(&LabeledSymbol::chr,   boost::bind(make, _1)) == c &&
      boost::bind(&LabeledSymbol::label, boost::bind(make, _1)) == label;
}
/* ************************************************************************* */

} // \namespace gtsam

