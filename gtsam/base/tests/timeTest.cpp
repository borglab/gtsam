/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    timing.h
 * @author  Richard Roberts (extracted from Michael Kaess' timing functions)
 * @date    Oct 5, 2010
 */

#include <gtsam/base/timing.h>

int main(int argc, char *argv[]) {

  // FIXME: ticPush_ does not exist
  ticPush_("1", "top 1");
  ticPush_("1", "sub 1");
  gttic_("sub sub a");
  gttoc_("sub sub a");
  ticPop_("1", "sub 1");
  ticPush_("2", "sub 2");
  gttic_("sub sub b");
  gttoc_("sub sub b");
  ticPop_("2", "sub 2");
  ticPop_("1", "top 1");

  ticPush_("2", "top 2");
  ticPush_("1", "sub 1");
  gttic_("sub sub a");
  gttoc_("sub sub a");
  ticPop_("1", "sub 1");
  ticPush_("2", "sub 2");
  gttic_("sub sub b");
  gttoc_("sub sub b");
  ticPop_("2", "sub 2");
  ticPop_("2", "top 2");

  for(size_t i=0; i<1000000; ++i) {
    ticPush_("3", "overhead");
    ticPush_("1", "overhead");
    ticPop_("1", "overhead");
    ticPop_("3", "overhead");
  }

  for(size_t i=0; i<1000000; ++i) {
    gttic(overhead_a);
    gttic(overhead_b);
    gttoc(overhead_b);
    gttoc(overhead_a);
  }

  tictoc_print_();

  return 0;
}
