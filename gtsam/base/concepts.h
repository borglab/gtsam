/*
 * concepts.h
 *
 * @date Dec 4, 2014
 * @author Mike Bosse
 * @author Frank Dellaert
 */

#pragma once

#ifdef GTSAM_USE_BOOST_FEATURES
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>
#include <boost/concept_check.hpp>
#define GTSAM_CONCEPT_ASSERT(concept) BOOST_CONCEPT_ASSERT((concept))
#define GTSAM_CONCEPT_REQUIRES(concept, return_type) BOOST_CONCEPT_REQUIRES(((concept)), (return_type))
#else 
// This does something sensible:
#define BOOST_CONCEPT_USAGE(concept) void check##concept()
// These just ignore the concept checking for now:
#define GTSAM_CONCEPT_ASSERT(concept) static_assert(true, "")
#define GTSAM_CONCEPT_REQUIRES(concept, return_type) return_type
#endif

