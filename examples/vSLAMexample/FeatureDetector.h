#ifndef FEATUREDETECTOR_H
#define FEATUREDETECTOR_H

#include "Feature2D.h"
#include <vector>
#include "cv.h"
#include "landmarkUtils.h"


/**
  * A simple Feature Detector to detect color-coded features in the image.
  * A list of colors must be provided. Feature id is the id of its associated color.
  */
class FeatureDetector
{
private:
    std::vector<Feature2D> vFeatures_;

public:
    std::vector<Feature2D>& read(const char* filename);
};

#endif // FEATUREDETECTOR_H
