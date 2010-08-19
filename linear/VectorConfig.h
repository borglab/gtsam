/**
 * @file    VectorConfig.h
 * @brief   Factor Graph Configuration
 * @author Frank Dellaert
 */

#pragma once

/*
 * There are two interchangeable implementations of VectorConfig.
 *
 * VectorMap uses a straightforward stl::map of Vectors. It has O(log n)
 * insert and access, and is fairly fast at both. However, it has high overhead
 * for arithmetic operations such as +, scale, axpy etc...
 *
 * VectorBTree uses a functional BTree as a way to access SubVectors
 * in an ordinary Vector. Inserting is O(n) and much slower, but accessing,
 * is O(log n) and might be a bit slower than VectorMap. Arithmetic operations
 * are blindingly fast, however. The cost is it is not as KISS as VectorMap.
 *
 * Access to vectors is now exclusively via operator[]
 * Vector access in VectorMap is via a Vector reference
 * Vector access in VectorBtree is via the SubVector type (see Vector.h)
 *
 * Feb 16 2010: FD: I made VectorMap the default, because I decided to try
 * and speed up conjugate gradients by using Sparse FactorGraphs all the way.
 */

// we use define and not typedefs as typdefs cannot be forward declared
#ifdef VECTORBTREE

#include <gtsam/linear/VectorBTree.h>
#define VectorConfig VectorBTree

#else

#include <gtsam/linear/VectorMap.h>
#define VectorConfig VectorMap

#endif

