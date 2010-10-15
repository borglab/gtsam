#include "FeatureDetector.h"
#include "landmarkUtils.h"
#include <fstream>
#include <stdio.h>

using namespace cv;
using namespace std;
using namespace gtsam;

FeatureDetector::FeatureDetector(const char* colorDescFile)
{
    readLandMarkColors(colorDescFile, vColors_);
}

int FeatureDetector::findColorId(const RGBColor& color)
{
    for (size_t i = 0; i<vColors_.size(); i++)
        if (vColors_[i] == color)
            return i;
    return -1;
}

std::vector<Feature2D> FeatureDetector::detect(const cv::Mat& image)
{
    vFeatures_.clear();

    RGBColor BLACK(0,0,0);
    map<int, int> numPixels; // <colorId, numpixel found>
    for (int y = 0; y<image.rows; y++) {
        for (int x = 0; x<image.cols; x++)
        {
            if (image.at<RGBColor>(y,x) == BLACK)
                continue;

            RGBColor color = image.at<RGBColor>(y,x);
            int t = color[2]; color[2] = color[0]; color[0] = t;
            int colorId = findColorId(color);

            if (colorId == -1)
                continue;

            map<int, int>::iterator it = numPixels.find(colorId)  ;
            if (it == numPixels.end())
            {
                numPixels[colorId] = 1;
                vFeatures_.push_back(Feature2D(colorId, Point2(x,y)));
            }
            else
            {
                size_t k = 0;
                for (; k<vFeatures_.size(); k++)
                    if (vFeatures_[k].m_id == colorId)
                        break;
                vFeatures_[k].m_p = (vFeatures_[k].m_p*numPixels[colorId] + Point2(x,y))/(numPixels[colorId]+1);
                numPixels[colorId]++;
            }
        }
    }

    return vFeatures_;
}

void FeatureDetector::write(const char* filename) const
{
    ofstream file(filename);
    file << vFeatures_.size() << endl;
    for (size_t i = 0; i<vFeatures_.size(); i++)
        file << vFeatures_[i].m_id << " " << vFeatures_[i].m_p.x() << " " << vFeatures_[i].m_p.y() << endl;
    file.close();
}

std::vector<Feature2D>& FeatureDetector::read(const char* filename)
{
    ifstream file(filename);
    int numFeatures;
    file >> numFeatures ;

    vFeatures_.clear();
    for (size_t i = 0; i < numFeatures; i++)
    {
        int id; double x, y;
        file >> id >> x >> y;
        vFeatures_.push_back(Feature2D(id, Point2(x, y)));
    }

    file.close();
    return vFeatures_;
}
