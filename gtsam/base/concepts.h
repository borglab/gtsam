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
#define GTSAM_CONCEPT_USAGE(concept) BOOST_CONCEPT_USAGE((concept))
#define GTSAM_CONCEPT_ASSERT(concept) BOOST_CONCEPT_ASSERT((concept))
#define GTSAM_CONCEPT_ASSERT1(concept) BOOST_CONCEPT_ASSERT((concept))
#define GTSAM_CONCEPT_ASSERT2(concept) BOOST_CONCEPT_ASSERT((concept))
#define GTSAM_CONCEPT_ASSERT3(concept) BOOST_CONCEPT_ASSERT((concept))
#define GTSAM_CONCEPT_REQUIRES(concept, return_type) BOOST_CONCEPT_REQUIRES((concept), (return_type))
#else 
// These do something sensible:
#define GTSAM_CONCEPT_USAGE(concept) void check##concept()
#define GTSAM_CONCEPT_ASSERT(concept) concept checkConcept [[maybe_unused]];
#define GTSAM_CONCEPT_ASSERT1(concept) concept checkConcept1 [[maybe_unused]];
#define GTSAM_CONCEPT_ASSERT2(concept) concept checkConcept2 [[maybe_unused]];
#define GTSAM_CONCEPT_ASSERT3(concept) concept checkConcept3 [[maybe_unused]];
// This one just ignores concept for now:
#define GTSAM_CONCEPT_REQUIRES(concept, return_type) return_type
#endif

