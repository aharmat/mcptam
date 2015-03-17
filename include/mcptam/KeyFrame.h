/*************************************************************************
 *  
 *  
 *  Copyright 2014  Adam Harmat (McGill University) 
 *                      [adam.harmat@mail.mcgill.ca]
 *                  Michael Tribou (University of Waterloo)
 *                      [mjtribou@uwaterloo.ca]
 *
 *  Multi-Camera Parallel Tracking and Mapping (MCPTAM) is free software:
 *  you can redistribute it and/or modify it under the terms of the GNU 
 *  General Public License as published by the Free Software Foundation,
 *  either version 3 of the License, or (at your option) any later
 *  version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *  
 *  MCPTAM is based on the Parallel Tracking and Mapping (PTAM) software.
 *  Copyright 2008 Isis Innovation Limited
 *  
 *  
 ************************************************************************/


/****************************************************************************************
 *
 * \file KeyFrame.h
 * \brief Declares the KeyFrame and MultiKeyFrame classes
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo (mjtribou@uwaterloo.ca)
 * 
 *   The class MultiKeyFrame contains a map of "camera name" => "KeyFrame"
 *   as well as a base pose. Individual KeyFrames are fixed with respect to 
 *   its parent MultiKeyFrame, and it is the MultiKeyFrame's pose that is
 *   optimized during bundle adjustment during normal operation. During calibration,
 *   the KeyFrame poses are treated differently, see calibration documentation for details. 
 *   
 *   Some functions have been moved from other parts of PTAM into the KeyFrame
 *   class: refreshing scene depth, calculating the distance to another KeyFrame,
 *   and handling the erasure of measurements. The MultiKeyFrame class also
 *   contains these functions and simply calls the corresponding function 
 *   in each of the KeyFrames it owns.
 * 
 *
 * Copyright 2008 Isis Innovation Limited
 *   This header declares the data structures to do with keyframes:
 *   structs KeyFrame, Level, Measurement, Candidate.
 *   
 *   A KeyFrame contains an image pyramid stored as array of Level;
 *   A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
 *   Each individual Level contains an image, corner points, and special corner points
 *   which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
 *   
 *   KeyFrames are stored in the Map class and manipulated by the MapMaker.
 *   However, the tracker also stores its current frame as a half-populated
 *   KeyFrame struct.
 *
 ****************************************************************************************/

#ifndef __KEY_FRAME_H
#define __KEY_FRAME_H

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <map>
#include <list>
#include <atomic>
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <opencv2/core/core.hpp>

class MapPoint;
class SmallBlurryImage;

namespace cv{
  class PersistentFREAK;
}

// Don't change LEVELS without understanding the code! 
// Lots of things are (unfortunately) hard coded for 4 levels
#define LEVELS 4  
#define RELOC_LEVEL 1  // don't put this on level 0, too heavy computationally
#define MAX_DEPTH 10000
#define MAX_SIGMA 10000
#define MIN_FAST_THRESH 5
#define MAX_FAST_THRESH 30


/// A feature in an image which could be made into a map point
struct Candidate
{
  CVD::ImageRef irLevelPos;       ///< ImageRef of the position at the level of its parent
  double dSTScore;                ///< Shi-Tomasi score
};

/// A 2D image measurement of a map point, each keyframe stores a bunch of these.
struct Measurement
{  
  inline Measurement()
  : bTransferred(false)
  , bDeleted(false)
  { }
  
  int nLevel;                 ///< Which image level?
  bool bSubPix;               ///< Has this measurement been refined to sub-pixel level?
  TooN::Vector<2> v2RootPos;  ///< Position of the measurement at level zero
  enum Src {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} eSource; ///< Where has this measurement come from?
  bool bTransferred;          ///< Has this measurement been transferred over the network? Meaningless in standalone application
  bool bDeleted;              ///< Used only during map editing
  
  // For finding matches after relocalization
  cv::Mat matDescriptor;
  
  int nID;  ///< Debugging ID
  
  //for testing
  CVD::ImageRef irOffset;
  bool bSelected;
};

/// Contains image data and corner points. Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
struct Level
{
  /** @brief Default constructor */
  inline Level() 
  : imagePrev(Level::snNumPrev)
  , vCornersPrev(Level::snNumPrev)
//  , bImsphereCornersCached(false)
  { }
  
  // Disable the assignment operator to make sure Levels aren't copied
  Level& operator=(const Level &rhs) = delete;
  
  CVD::Image<CVD::byte> lastMask;          ///< Used to draw last combined internal + glare mask by Tracker
  CVD::Image<CVD::byte> mask;              ///< The loaded mask
  CVD::Image<CVD::byte> image;             ///< The pyramid level pixels
  std::vector<CVD::ImageRef> vCorners;     ///< All FAST corners on this level
  cv::Mat matBoW;                          ///< Bag of words descriptor for each FAST corner (same number of rows as vCorners has elements)
  std::vector<int> vCornerRowLUT;          ///< Row-index into the FAST corners, speeds up access
  std::vector<Candidate> vCandidates;      ///< Potential locations of new map points
  std::vector<std::pair<double, CVD::ImageRef> > vScoresAndMaxCorners;  ///< The best scoring points and their scores, used to generate candidates
  
  //bool bImsphereCornersCached;             ///< Have the FAST corners been projected onto the camera sphere?
  //std::vector<TooN::Vector<3> > vImsphereCorners; ///< Corner points un-projected into r=1 sphere coordinates, used to speed up epipolar search

  TooN::Vector<MAX_FAST_THRESH+1, int> vFastFrequency;  ///< Histogram of number of corners at different thresholds
  int nFastThresh;  ///< The threshold that was selected
  
  // Circular buffers to hold previous images and corners, used to figure out which candidate corners are stable
  boost::circular_buffer<CVD::Image<CVD::byte> > imagePrev;
  boost::circular_buffer<std::vector<CVD::ImageRef> > vCornersPrev;
  
  static int snNumPrev;  ///< Size of circular buffers
};


class MultiKeyFrame;
typedef std::map<MapPoint*, Measurement*> MeasPtrMap;  

/** @brief Contains all keyframe-related data, such as pose relative to the world, the image pyramid, candidate FAST corners, etc.
 * 
 *  The map contains of a bunch of these, embedded in their parent MultiKeyFrames. Every incoming image is turned into
 *  a KeyFrame before tracking, grouped together into MultiKeyFrames. Most of these are then overwritten
 *  on the next frame, but sometimes they're added to the map and the tracker regenerates its keyframes.
 */
class KeyFrame
{
public:
  
  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param pMKF Pointer to the MultiKeyFrame that will be the owner of this KeyFrame
   *  @param name The name of the camera whose image this KeyFrame will hold
   */
  KeyFrame(MultiKeyFrame* pMKF, std::string name);
  /// Deletes the small blurry image and clears all measurements
  ~KeyFrame();
  
  // Operators
  /// Deleted to disallow copy construction
  KeyFrame(const KeyFrame& other) = delete;
  /// Deleted to disallow assignment operation
  KeyFrame& operator= (const KeyFrame & other) = delete;
  
  /// Takes an image and calculates pyramid levels etc to fill the keyframe data structures with everything that's needed by the tracker.
  /** @param im The raw image from the camera
   *  @param bDeepCopy Should the function make a deep copy of the image or rely on reference counting?
   *  @param bGlareMasking Should a mask be created from the portions of the image with large values (ie white)?
   *  @return A tuple of timing values, used mostly for debugging */
  std::tuple<double,double,double> MakeKeyFrame_Lite(CVD::Image<CVD::byte> &im, bool bDeepCopy = true, bool bGlareMasking = true);   
  //void MakeKeyFrame_Lite(CVD::Image<CVD::byte> &im, bool bDeepCopy = true);
    
  /// Calculates the rest of the data which the mapmaker needs but the tracker doesn't.
  void MakeKeyFrame_Rest();
  
  /// Set the fixed mask to a given image. For example, landing gear or other aircraft parts might be in the image and
  /// we don't want to use points that fall in those areas
  void SetMask(CVD::Image<CVD::byte> &m);
  
  // Utility functions
  /// Refreshes the mean scene depth and the scene depth variance with a robust M-estimator using the measurements stored in the KeyFrame
  void RefreshSceneDepthRobust();
  
  /// Refreshes the mean scene depth and the scene depth varianace with a robust M-estimator using the specified point depths and weights
  /** @param vDepthsAndWeights Vector of pairs containing <depth, weight> */
  void RefreshSceneDepthRobust(std::vector<std::pair<double, double> >& vDepthsAndWeights);
  
  /// Refreshes the mean scene depth and the scene depth varianace with a weighted mean using the measurements stored in the KeyFrame
  void RefreshSceneDepthMean();
  
  /// Refershes the mean scene depth and the scene depth variance with a weighted mean using the specified point depths and weights
  /** @param vDepthsAndWeights Vector of pairs containing <depth, weight> */
  void RefreshSceneDepthMean(std::vector<std::pair<double, double> >& vDepthsAndWeights);
  
  /// Calculates the Euclidean linear distance between the current keyframe and the argument
  /** @param other The other KeyFrame 
   *  @return The calculated distance */
  double LinearDist(KeyFrame &other);
  
  /// Calculates a distance based on the Euclidean distance separating the KeyFrames as well as the points defined by their mean scene depths
  /** @param other The other KeyFrame 
   *  @return The calculated distance */
  double Distance(KeyFrame &other);
  
  /// Erase the measurement object corresponding to its argument and remove it from the map of measurements
  /** @param pPoint Pointer to the MapPoint to erase */
  void EraseMeasurementOfPoint(MapPoint* pPoint);
  
  /// Erase, from all points measured, the pointer back to this KeyFrame
  void EraseBackLinksFromPoints();
  
  /// Erase all measurements
  void ClearMeasurements();
  
  void AddMeasurement(MapPoint* pPoint, Measurement* pMeas, bool bExtractDescriptor);
  
  void CreateMeasurementDescriptor(Measurement& meas);
  
  /// Make the small blurry image
  void MakeSBI();
  
  void MakeExtractor();
  
  bool NoImage();
  
  void RemoveImage();
  
  // Variables
  static double sdDistanceMeanDiffFraction;  ///< fraction of distance between mean scene depth points that is used in overall distance computation
//  static double saThreshDerivs[LEVELS];  ///< derivatives that determine FAST threshold
  static std::string ssCandidateType; ///< decide scoring type ("fast, "shi")
  static std::string ssCandidateCriterion;  ///< decide scoring criterion ("percent", "thresh")
  static double sdCandidateThresh; ///< when using "thresh" criterion
  static double sdCandidateTopFraction; ///< when using "percent" criterion
  static bool sbAdaptiveThresh;  ///< should we use an adaptive computation of the feature detection threshold?
  
  TooN::SE3<> mse3CamFromBase;   ///< The current pose in a base frame, which is the pose of the parent MultiKeyFrame
  TooN::SE3<> mse3CamFromWorld;  ///< The current pose in the world frame, a product of mse3CamFromBase and the parent's mse3BaseFromWorld

  Level maLevels[LEVELS];  ///< Images, corners, etc lives in this array of pyramid levels
  
  cv::PersistentFREAK* mpExtractor;
  
  MeasPtrMap mmpMeasurements;   ///< All the measurements associated with the keyframe as a map of MapPoint pointers to Measurement pointers  
  //MeasPtrMap mmpDeletedMeas;  ///< Queue of deleted measurements' map points, used to send information in client/server mode
  //MeasPtrMap mmpClearedMeas;
  //MeasPtrMap mmpMeasurementsPerma;
  
  boost::mutex mMeasMutex;   ///< To allow multi-threaded operation safely
  
  //testing
  //MeasPtrMap mmpBlobMeasurements;
  
  SmallBlurryImage* mpSBI; ///< Extremely downsampled version of the stored image, the relocaliser uses this

  double mdSceneDepthMean;  ///< The mean z-axis value of all the points visible from this keyframe
  double mdSceneDepthSigma; ///< The variance of the z-axis values of the points visible from this keyframe
  
  const std::string mCamName;  ///< The camera name corresponding to this keyframe

  MultiKeyFrame* mpParent;  ///< Pointer to the MultiKeyFrame that owns this keyframe
  
  bool mbActive;  ///< The tracker uses this to indicate which keyframes have been updated with new measurements, the MapMaker culls inactive keyframes when it receives a new MultiKeyFrame
  
private:

  /// Generate a vector of <depth, weight> pairs from points that this keyframe measures, used in the RefreshSceneDepth functions
  /** @param vDepthsAndWeights The vector of pairs to fill */
  void GetPointDepthsAndWeights(std::vector<std::pair<double, double> >& vDepthsAndWeights);
  
};

typedef std::map<std::string, KeyFrame*> KeyFramePtrMap;

/** @brief Contains a collection of KeyFrames, as well as a pose relative to the world and utility functions to deal with the owned KeyFrames
 */
class MultiKeyFrame
{  
public:

  MultiKeyFrame();
  
  /// Deletes owned KeyFrames
  ~MultiKeyFrame();
  
  // Operators
  /// Deleted to disallow copy construction
  MultiKeyFrame(const MultiKeyFrame& other) = delete;
  /// Deleted to disallow assignment operation
  MultiKeyFrame& operator= (const MultiKeyFrame & other) = delete;
  
  /// Call EraseBackLinksFromPoints on all owned KeyFrames
  void EraseBackLinksFromPoints();
  
  /// Call RefreshSceneDepthRobust on all owned KeyFrames
  void RefreshSceneDepthRobust();
  
  /// Call RefreshSceneDepthMean on all owned KeyFrames
  void RefreshSceneDepthMean();
  
  /** @brief Calculates the Euclidean linear distance between this MultiKeyFrame and the argument
   * 
   *  Computes the distance between the base poses of both MultiKeyFrames
   *  @param other The other MultiKeyFrame 
   *  @return The calculated distance */
  double LinearDist(MultiKeyFrame &other);
  
  /** @brief Calculates the distance between this MultiKeyFrame and the argument as the minimum distance between any of their KeyFrames
   * 
   *  Uses the KeyFrame::Distance function to compute pairwise distances
   *  @param other The other MultiKeyFrame 
   *  @return The calculated distance */
  double Distance(MultiKeyFrame &other);
  
  /// Update the camera-from-world poses of the KeyFrames, based on the MultiKeyFrame's pose and the fixed relative transforms
  void UpdateCamsFromWorld();
  
  /// Call ClearMeasurements on all owned KeyFrames
  void ClearMeasurements();
  
  /// Returns the number of measurements of all owned KeyFrames
  int NumMeasurements();
  
  bool NoImages();
  
  void RemoveImages();
  
  // Variables
  TooN::SE3<> mse3BaseFromWorld;  ///< The current pose in the world reference frame
  bool mbFixed; ///< Is the pose fixed? Generally only true for the first MultiKeyFrame added to the map
  bool mbBad;  ///< Is it a dud? In that case it'll be moved to the trash soon.
  bool mbDeleted; ///< Similar to mbBad, but used only in client/server code to allow immediate deletion of received MKF
  
  /// Atomic counter that indicates the number of TrackerData structures in Tracker currently referencing this MKF through
  /// KF measurements
  std::atomic<unsigned int> mnUsing;  
  
  KeyFramePtrMap mmpKeyFrames;  ///< %Map of camera names to KeyFrame pointers
  
  double mdTotalDepthMean;  ///< The mean of all owned KeyFrames' mdSceneDepthMean values
  int mnID;      ///< Used for identifying MultiKeyFrame when writing map to file          
};


#endif

