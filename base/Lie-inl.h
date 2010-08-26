/*
 * LieBaseImplementations.h
 *
 *  Created on: Jan 9, 2010
 *      Author: richard
 */

#pragma once

#include <gtsam/base/Lie.h>

#define INSTANTIATE_LIE(T) \
  template T between_default(const T&, const T&); \
  template Vector logmap_default(const T&, const T&); \
  template T expmap_default(const T&, const Vector&); \
  template bool equal(const T&, const T&, double); \
  template bool equal(const T&, const T&); \
  template class Lie<T>;

