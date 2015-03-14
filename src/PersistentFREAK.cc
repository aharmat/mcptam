#include <mcptam/PersistentFREAK.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/assert.h>
#include <bitset>

namespace cv
{

static const double FREAK_SQRT2 = 1.4142135623731;
static const double FREAK_INV_SQRT2 = 1.0 / FREAK_SQRT2;
static const double FREAK_LOG2 = 0.693147180559945;
static const int FREAK_NB_ORIENTATION = 256;
static const int FREAK_NB_POINTS = 43;
static const int FREAK_SMALLEST_KP_SIZE = 7; // smallest size of keypoints
static const int FREAK_NB_SCALES = FREAK::NB_SCALES;
static const int FREAK_NB_PAIRS = FREAK::NB_PAIRS;
static const int FREAK_NB_ORIENPAIRS = FREAK::NB_ORIENPAIRS;

// default pairs
static const int FREAK_DEF_PAIRS[FREAK::NB_PAIRS] =
{
     404,431,818,511,181,52,311,874,774,543,719,230,417,205,11,
     560,149,265,39,306,165,857,250,8,61,15,55,717,44,412,
     592,134,761,695,660,782,625,487,549,516,271,665,762,392,178,
     796,773,31,672,845,548,794,677,654,241,831,225,238,849,83,
     691,484,826,707,122,517,583,731,328,339,571,475,394,472,580,
     381,137,93,380,327,619,729,808,218,213,459,141,806,341,95,
     382,568,124,750,193,749,706,843,79,199,317,329,768,198,100,
     466,613,78,562,783,689,136,838,94,142,164,679,219,419,366,
     418,423,77,89,523,259,683,312,555,20,470,684,123,458,453,833,
     72,113,253,108,313,25,153,648,411,607,618,128,305,232,301,84,
     56,264,371,46,407,360,38,99,176,710,114,578,66,372,653,
     129,359,424,159,821,10,323,393,5,340,891,9,790,47,0,175,346,
     236,26,172,147,574,561,32,294,429,724,755,398,787,288,299,
     769,565,767,722,757,224,465,723,498,467,235,127,802,446,233,
     544,482,800,318,16,532,801,441,554,173,60,530,713,469,30,
     212,630,899,170,266,799,88,49,512,399,23,500,107,524,90,
     194,143,135,192,206,345,148,71,119,101,563,870,158,254,214,
     276,464,332,725,188,385,24,476,40,231,620,171,258,67,109,
     844,244,187,388,701,690,50,7,850,479,48,522,22,154,12,659,
     736,655,577,737,830,811,174,21,237,335,353,234,53,270,62,
     182,45,177,245,812,673,355,556,612,166,204,54,248,365,226,
     242,452,700,685,573,14,842,481,468,781,564,416,179,405,35,
     819,608,624,367,98,643,448,2,460,676,440,240,130,146,184,
     185,430,65,807,377,82,121,708,239,310,138,596,730,575,477,
     851,797,247,27,85,586,307,779,326,494,856,324,827,96,748,
     13,397,125,688,702,92,293,716,277,140,112,4,80,855,839,1,
     413,347,584,493,289,696,19,751,379,76,73,115,6,590,183,734,
     197,483,217,344,330,400,186,243,587,220,780,200,793,246,824,
     41,735,579,81,703,322,760,720,139,480,490,91,814,813,163,
     152,488,763,263,425,410,576,120,319,668,150,160,302,491,515,
     260,145,428,97,251,395,272,252,18,106,358,854,485,144,550,
     131,133,378,68,102,104,58,361,275,209,697,582,338,742,589,
     325,408,229,28,304,191,189,110,126,486,211,547,533,70,215,
     670,249,36,581,389,605,331,518,442,822
};

// used to sort pairs during pairs selection
struct PairStat
{
    double mean;
    int idx;
};

struct sortMean
{
    bool operator()( const PairStat& a, const PairStat& b ) const
    {
        return a.mean < b.mean;
    }
};

PersistentFREAK::PersistentFREAK(const Mat& img, bool orientationNormalized,
                                 bool scaleNormalized,
                                 float patternScale,
                                 int nOctaves,
                                 const vector<int>& selectedPairs)
: FREAK(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs)
, image(img)
{
  ROS_ASSERT(!image.empty());
  
  buildPattern();
  integral(image, imgIntegral);
}

// This function is directly copied from OpenCV's freak.cpp, it just uses the pattern
// built in the constructor and the saved integral image rather than doing those every
// time the compute function is called
void PersistentFREAK::compute(vector<KeyPoint>& keypoints, Mat& descriptors ) const
{
    if( keypoints.empty() )
        return;

    std::vector<int> kpScaleIdx(keypoints.size()); // used to save pattern scale index corresponding to each keypoints
    const std::vector<int>::iterator ScaleIdxBegin = kpScaleIdx.begin(); // used in std::vector erase function
    const std::vector<cv::KeyPoint>::iterator kpBegin = keypoints.begin(); // used in std::vector erase function
    const float sizeCst = static_cast<float>(FREAK_NB_SCALES/(FREAK_LOG2* nOctaves));
    uchar pointsValue[FREAK_NB_POINTS];
    int thetaIdx = 0;
    int direction0;
    int direction1;

    // compute the scale index corresponding to the keypoint size and remove keypoints close to the border
    if( scaleNormalized )
    {
        for( size_t k = keypoints.size(); k--; )
        {
            //Is k non-zero? If so, decrement it and continue"
            kpScaleIdx[k] = max( (int)(log(keypoints[k].size/FREAK_SMALLEST_KP_SIZE)*sizeCst+0.5) ,0);
            if( kpScaleIdx[k] >= FREAK_NB_SCALES )
                kpScaleIdx[k] = FREAK_NB_SCALES-1;

            if( keypoints[k].pt.x <= patternSizes[kpScaleIdx[k]] || //check if the description at this specific position and scale fits inside the image
                 keypoints[k].pt.y <= patternSizes[kpScaleIdx[k]] ||
                 keypoints[k].pt.x >= image.cols-patternSizes[kpScaleIdx[k]] ||
                 keypoints[k].pt.y >= image.rows-patternSizes[kpScaleIdx[k]]
               )
            {
                keypoints.erase(kpBegin+k);
                kpScaleIdx.erase(ScaleIdxBegin+k);
            }
        }
    }
    else
    {
        const int scIdx = max( (int)(1.0986122886681*sizeCst+0.5) ,0);
        for( size_t k = keypoints.size(); k--; )
        {
            kpScaleIdx[k] = scIdx; // equivalent to the formule when the scale is normalized with a constant size of keypoints[k].size=3*SMALLEST_KP_SIZE
            if( kpScaleIdx[k] >= FREAK_NB_SCALES )
            {
                kpScaleIdx[k] = FREAK_NB_SCALES-1;
            }
            if( keypoints[k].pt.x <= patternSizes[kpScaleIdx[k]] ||
                keypoints[k].pt.y <= patternSizes[kpScaleIdx[k]] ||
                keypoints[k].pt.x >= image.cols-patternSizes[kpScaleIdx[k]] ||
                keypoints[k].pt.y >= image.rows-patternSizes[kpScaleIdx[k]]
               )
            {
                keypoints.erase(kpBegin+k);
                kpScaleIdx.erase(ScaleIdxBegin+k);
            }
        }
    }

    // allocate descriptor memory, estimate orientations, extract descriptors
    if( !extAll )
    {
        // extract the best comparisons only
        descriptors = cv::Mat::zeros((int)keypoints.size(), FREAK_NB_PAIRS/8, CV_8U);
#if CV_SSE2
        __m128i* ptr= (__m128i*) (descriptors.data+(keypoints.size()-1)*descriptors.step[0]);
#else
        std::bitset<FREAK_NB_PAIRS>* ptr = (std::bitset<FREAK_NB_PAIRS>*) (descriptors.data+(keypoints.size()-1)*descriptors.step[0]);
#endif
        for( size_t k = keypoints.size(); k--; )
        {
            // estimate orientation (gradient)
            if( !orientationNormalized )
            {
                thetaIdx = 0; // assign 0° to all keypoints
                keypoints[k].angle = 0.0;
            }
            else
            {
                // get the points intensity value in the un-rotated pattern
                for( int i = FREAK_NB_POINTS; i--; )
                {
                    pointsValue[i] = meanIntensity(image, imgIntegral, keypoints[k].pt.x,keypoints[k].pt.y, kpScaleIdx[k], 0, i);
                }
                direction0 = 0;
                direction1 = 0;
                for( int m = 45; m--; )
                {
                    //iterate through the orientation pairs
                    const int delta = (pointsValue[ orientationPairs[m].i ]-pointsValue[ orientationPairs[m].j ]);
                    direction0 += delta*(orientationPairs[m].weight_dx)/2048;
                    direction1 += delta*(orientationPairs[m].weight_dy)/2048;
                }

                keypoints[k].angle = static_cast<float>(atan2((float)direction1,(float)direction0)*(180.0/CV_PI));//estimate orientation
                thetaIdx = int(FREAK_NB_ORIENTATION*keypoints[k].angle*(1/360.0)+0.5);
                if( thetaIdx < 0 )
                    thetaIdx += FREAK_NB_ORIENTATION;

                if( thetaIdx >= FREAK_NB_ORIENTATION )
                    thetaIdx -= FREAK_NB_ORIENTATION;
            }
            // extract descriptor at the computed orientation
            for( int i = FREAK_NB_POINTS; i--; )
            {
                pointsValue[i] = meanIntensity(image, imgIntegral, keypoints[k].pt.x,keypoints[k].pt.y, kpScaleIdx[k], thetaIdx, i);
            }
#if CV_SSE2
            // note that comparisons order is modified in each block (but first 128 comparisons remain globally the same-->does not affect the 128,384 bits segmanted matching strategy)
            int cnt = 0;
            for( int n = FREAK_NB_PAIRS/128; n-- ; )
            {
                __m128i result128 = _mm_setzero_si128();
                for( int m = 128/16; m--; cnt += 16 )
                {
                    __m128i operand1 = _mm_set_epi8(
                        pointsValue[descriptionPairs[cnt+0].i],
                        pointsValue[descriptionPairs[cnt+1].i],
                        pointsValue[descriptionPairs[cnt+2].i],
                        pointsValue[descriptionPairs[cnt+3].i],
                        pointsValue[descriptionPairs[cnt+4].i],
                        pointsValue[descriptionPairs[cnt+5].i],
                        pointsValue[descriptionPairs[cnt+6].i],
                        pointsValue[descriptionPairs[cnt+7].i],
                        pointsValue[descriptionPairs[cnt+8].i],
                        pointsValue[descriptionPairs[cnt+9].i],
                        pointsValue[descriptionPairs[cnt+10].i],
                        pointsValue[descriptionPairs[cnt+11].i],
                        pointsValue[descriptionPairs[cnt+12].i],
                        pointsValue[descriptionPairs[cnt+13].i],
                        pointsValue[descriptionPairs[cnt+14].i],
                        pointsValue[descriptionPairs[cnt+15].i]);

                    __m128i operand2 = _mm_set_epi8(
                        pointsValue[descriptionPairs[cnt+0].j],
                        pointsValue[descriptionPairs[cnt+1].j],
                        pointsValue[descriptionPairs[cnt+2].j],
                        pointsValue[descriptionPairs[cnt+3].j],
                        pointsValue[descriptionPairs[cnt+4].j],
                        pointsValue[descriptionPairs[cnt+5].j],
                        pointsValue[descriptionPairs[cnt+6].j],
                        pointsValue[descriptionPairs[cnt+7].j],
                        pointsValue[descriptionPairs[cnt+8].j],
                        pointsValue[descriptionPairs[cnt+9].j],
                        pointsValue[descriptionPairs[cnt+10].j],
                        pointsValue[descriptionPairs[cnt+11].j],
                        pointsValue[descriptionPairs[cnt+12].j],
                        pointsValue[descriptionPairs[cnt+13].j],
                        pointsValue[descriptionPairs[cnt+14].j],
                        pointsValue[descriptionPairs[cnt+15].j]);

                    __m128i workReg = _mm_min_epu8(operand1, operand2); // emulated "not less than" for 8-bit UNSIGNED integers
                    workReg = _mm_cmpeq_epi8(workReg, operand2);        // emulated "not less than" for 8-bit UNSIGNED integers

                    workReg = _mm_and_si128(_mm_set1_epi16(short(0x8080 >> m)), workReg); // merge the last 16 bits with the 128bits std::vector until full
                    result128 = _mm_or_si128(result128, workReg);
                }
                (*ptr) = result128;
                ++ptr;
            }
            ptr -= 8;
#else
            // extracting descriptor preserving the order of SSE version
            int cnt = 0;
            for( int n = 7; n < FREAK_NB_PAIRS; n += 128)
            {
                for( int m = 8; m--; )
                {
                    int nm = n-m;
                    for(int kk = nm+15*8; kk >= nm; kk-=8, ++cnt)
                    {
                        ptr->set(kk, pointsValue[descriptionPairs[cnt].i] >= pointsValue[descriptionPairs[cnt].j]);
                    }
                }
            }
            --ptr;
#endif
        }
    }
    else // extract all possible comparisons for selection
    {
        descriptors = cv::Mat::zeros((int)keypoints.size(), 128, CV_8U);
        std::bitset<1024>* ptr = (std::bitset<1024>*) (descriptors.data+(keypoints.size()-1)*descriptors.step[0]);

        for( size_t k = keypoints.size(); k--; )
        {
            //estimate orientation (gradient)
            if( !orientationNormalized )
            {
                thetaIdx = 0;//assign 0° to all keypoints
                keypoints[k].angle = 0.0;
            }
            else
            {
                //get the points intensity value in the un-rotated pattern
                for( int i = FREAK_NB_POINTS;i--; )
                    pointsValue[i] = meanIntensity(image, imgIntegral, keypoints[k].pt.x,keypoints[k].pt.y, kpScaleIdx[k], 0, i);

                direction0 = 0;
                direction1 = 0;
                for( int m = 45; m--; )
                {
                    //iterate through the orientation pairs
                    const int delta = (pointsValue[ orientationPairs[m].i ]-pointsValue[ orientationPairs[m].j ]);
                    direction0 += delta*(orientationPairs[m].weight_dx)/2048;
                    direction1 += delta*(orientationPairs[m].weight_dy)/2048;
                }

                keypoints[k].angle = static_cast<float>(atan2((float)direction1,(float)direction0)*(180.0/CV_PI)); //estimate orientation
                thetaIdx = int(FREAK_NB_ORIENTATION*keypoints[k].angle*(1/360.0)+0.5);

                if( thetaIdx < 0 )
                    thetaIdx += FREAK_NB_ORIENTATION;

                if( thetaIdx >= FREAK_NB_ORIENTATION )
                    thetaIdx -= FREAK_NB_ORIENTATION;
            }
            // get the points intensity value in the rotated pattern
            for( int i = FREAK_NB_POINTS; i--; )
            {
                pointsValue[i] = meanIntensity(image, imgIntegral, keypoints[k].pt.x,
                                             keypoints[k].pt.y, kpScaleIdx[k], thetaIdx, i);
            }

            int cnt(0);
            for( int i = 1; i < FREAK_NB_POINTS; ++i )
            {
                //(generate all the pairs)
                for( int j = 0; j < i; ++j )
                {
                    ptr->set(cnt, pointsValue[i] >= pointsValue[j] );
                    ++cnt;
                }
            }
            --ptr;
        }
    }
  
}

} // end namespace cv
