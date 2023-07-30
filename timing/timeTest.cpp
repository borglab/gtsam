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

using namespace gtsam;

int main(int argc, char *argv[]) {

  {
    gttic_(top1);
    gttic_(sub1);
    gttic_(sub_sub_a);
    gttoc_(sub_sub_a);
    gttoc_(sub1);
    gttic_(sub2);
    gttic_(sub_sub_b);
    gttoc_(sub_sub_b);
    gttoc_(sub2);
    gttoc_(top1);
  }

  {
    gttic_(top2);
    gttic_(sub1);
    gttic_(sub_sub_a);
    gttoc_(sub_sub_a);
    gttoc_(sub1);
    gttic_(sub2);
    gttic_(sub_sub_b);
    gttoc_(sub_sub_b);
    gttoc_(sub2);
    gttoc_(top2);
  }

  gttic_(top3);
  for(size_t i=0; i<1000000; ++i) {
    gttic_(overhead);
    gttic_(sub_overhead);
    gttoc_(sub_overhead);
    gttoc_(overhead);
    tictoc_finishedIteration_();
  }
  gttoc_(top3);

  gttic_(top4);
  for(size_t i=0; i<1000000; ++i) {
    gttic(overhead_a);
    gttic(overhead_b);
    gttoc(overhead_b);
    gttoc(overhead_a);
    tictoc_finishedIteration();
  }
  gttoc_(top4);

  tictoc_print_();

  return 0;
}
