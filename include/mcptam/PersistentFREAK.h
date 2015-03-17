#ifndef PERSISTENT_FREAK_H
#define PERSISTENT_FREAK_H

#include <opencv2/features2d/features2d.hpp>

namespace cv
{

class CV_EXPORTS PersistentFREAK : public FREAK
{
public:

	PersistentFREAK(const Mat& img, bool orientationNormalized = true,
           bool scaleNormalized = true,
           float patternScale = 22.0f,
           int nOctaves = 4,
           const vector<int>& selectedPairs = vector<int>());
           
  void compute(vector<KeyPoint>& keypoints, Mat& descriptors ) const;
           
protected:
    
  Mat image;
  Mat imgIntegral;
};



}  // end namespace cv


#endif
