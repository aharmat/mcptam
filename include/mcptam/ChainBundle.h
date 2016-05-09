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
 * \file ChainBundle.h
 * \brief Declaration of ChainBundle class
 *
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * ChainBundle has the same function as the Bundle class included with PTAM, but
 *achieves
 * it quite differently. Whereas the Bundle class used a straight implementation
 *of Hartley
 * and Zisserman's bundle adjustment code from the book "Multiple View
 *Geometry",
 * ChainBundle uses the G2O generic graph optimizer
 *(http://openslam.org/g2o.html).
 *
 * A single measurement is a function of a map point and an observing camera.
 *However,
 * unlike traditional BA, these are not specified in the world frame. Rather,
 *they are
 * specified in the frame of their parent entity, whose pose is likewise
 *specified in ITS
 * parent entity's frame, etc, forming pose chains. Therefore, each measurement
 *has two
 * pose chains, one that defines the location of the map point, and one that
 *defines the
 * location of the observing camera.
 *
 * In mcptam, where MapPoints are defined relative to KeyFrames, which
 * belong to a MultiKeyFrame that is located in the world frame, we only have
 *pose
 * chains of length 3 at most: world ===== MKF ===== KF ===== point, but pose
 *chains could be of any length.
 * For example, they could implement Relative Bundle Adjustment (by Sibley et
 *al.) where
 * the pose chains are very long. However, since the code was not designed with
 *relative
 * bundle adjustment in mind, it might not be very efficient at performing that
 *task.
 * Another example is during camera extrinsic calibration, where the pose chains
 *share
 * the link that defines the relative pose between KFs so that there is only a
 *single transformation
 * being estimated between any two KFs.
 *
 * The point features are parameterized with respect to their anchor KF and MKF.
 * Therefore, when giving a point to ChainBundle it needs not only the 3-vector
 * spherical coordinates for the point but also the source KeyFrame that the
 *point is
 * defined in.
 *
 ****************************************************************************************/

#ifndef MCPTAM_CHAINBUNDLE_H
#define MCPTAM_CHAINBUNDLE_H

#include <mcptam/Types.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <tuple>
#include <map>
#include <string>
#include <vector>

// Forward declarations
namespace g2o
{
class SparseOptimizer;
}

class UpdateSigmaSquaredAction;
class CheckConvergedUpdateMagAction;
class CheckConvergedResidualAction;
class UpdateHelpersAction;
class PoseChainHelper;
class RobustKernelAdaptive;
class RobustKernelData;
class UpdateTotalIterationsAction;

typedef std::map<std::vector<int>, PoseChainHelper *> HelperMap;

/** @brief Performs bundle adjustment on a chain of poses where the final pose
 *is linked to map points through measurements.
 *
 *  Unlike the Bundle class in PTAM, ChainBundle supports multiple camera models
 *and multiple measurements between
 *  one point and one camera (which is the movable base).
 */
class ChainBundle
{
public:
  /** @brief Creates the bundle adjuster object
   *  @param cameraModels The camera models are needed for point reprojection
   *  @param bUseRobust Should we do robustification of errors during
   * optimization? Uses Huber M-estimator.
   *  @param bUseTukey Should we do a round of Tukey robustification AFTER
   * optimization? If you want to generate outliers, you have to do this.
   *  @param bVerbose Verbose flag for the g2o optimizer */
  ChainBundle(TaylorCameraMap &cameraModels, bool bUseRobust, bool bUseTukey, bool bVerbose);

  /// Destructor
  ~ChainBundle();

  // Functions for adding data to the system
  /** @brief Add a pose to the system.
   *
   *  @param se3PoseFromRef The pose to add
   *  @param bFixed Can this pose be modified during adjustment?
   *  @return An index by which the pose is referenced inside ChainBundle */
  int AddPose(TooN::SE3<> se3PoseFromRef, bool bFixed);

  /** @brief Add a map point to the system.
   *  @param v3PointInCam The coordinates of the point in the parent camera's
   * frame
   *  @param vCams The chain of pose indices that this point is defined in
   *  @param bFixed Can this point be moved during adjustment?
   *  @return An index by which the point is referenced inside ChainBundle */
  int AddPoint(TooN::Vector<3> v3PointInCam, std::vector<int> vCams, bool bFixed);

  /** @brief Add a measurement connecting one point to the end of one pose chain
   *
   *  @param vCams The pose chain of the measuring camera
   *  @param nPointIdx The index of the point
   *  @param v2Pos The pixel coordinates of the image of the point
   *  @param dNoiseSigmaSquared The measurement noise
   *  @param cameraName The name of the camera that made this measurement */
  void AddMeas(std::vector<int> vCams, int nPointIdx, TooN::Vector<2> v2Pos, double dNoiseSigmaSquared,
               std::string cameraName);

  /** @brief Initialize internal variables and the g2o optimizer */
  void Initialize();

  /** @brief Perform bundle adjustment.
   *
   * Initializes everything and performs a given number of iterations.
   * @param pAbortSignal Pointer to a boolean abort signal.
   * @param nNumIter Number of steps to take
   * @param dUserLambda User specified initial lambda value for
   *Levenberg-Marquardt algorithm
   * @return Number of accepted update iterations, or negative on error. */
  int Compute(bool *pAbortSignal, int nNumIter = ChainBundle::snMaxIterations, double dUserLambda = -1);

  /// Has bundle adjustment converged?
  inline bool Converged()
  {
    return mbConverged;
  }

  inline int TotalIterations()
  {
    return mnTotalIterations;
  }

  /// Get point position after adjustment
  /** @param n The index of the point
   *  @return The position of the point (in the parent camera frame!) */
  TooN::Vector<3> GetPoint(int n);

  /// Get pose after adjustment
  /** @param n The index of the pose
   *  @return The pose */
  TooN::SE3<> GetPose(int n);

  /** @brief Get measurements flagged as outliers
   *
   * The measurements are encoded as a tuple of <point idx, pose idx, camera
   *name>, which
   * can be used to convert to a MapPoint, KeyFrame pair by the caller
   * @return A vector of measurements encoded as described above */
  std::vector<std::tuple<int, int, std::string>> GetOutlierMeasurements();

  /** @brief Get the sigma squared value for the optimization */
  double GetSigmaSquared();

  /** @brief Get the chi2 value (sum of errors squared weighted by information
   * and robust kernel) */
  double GetMeanChiSquared();

  /** @brief Get the maximum covariance of the point feature depth */
  double GetMaxCov()
  {
    return mdLastMaxCov;
  }

  /** @brief Get the last value of the LM lambda parameter */
  double GetLambda();

  /** @brief Get the depth covariance of the point */
  double GetPointDepthCovariance(int n);
  
  // Static members
  static int snMaxIterations;                     ///< Maximum number of iterations allowed before bailing
  static int snMaxTrialsAfterFailure;             ///< Maximum number of times the solver will
  static double sdUpdatePercentConvergenceLimit;  ///< If residual error changes
  /// less than this, consider
  /// problem converged
  static double sdUpdateRMSConvergenceLimit;  ///< If update magnitude RMS less
  /// than this, consider problem
  /// converged
  static double sdMinMEstimatorSigma;  ///< The minimum allowed sigma squared
  /// value to use with MEstimator

protected:
  g2o::SparseOptimizer *mpOptimizer;     ///< The actual optimizer
  RobustKernelData *mpRobustKernelData;  ///< Data for the robust kernel

  // The following are "actions" that are added to the optimizer to be performed
  // at various points in the minimization loop
  UpdateSigmaSquaredAction *mpUpdateSigmaSquaredAction;  ///< Update the sigma
  /// squared used for
  /// robustifying
  /// measurements
  CheckConvergedUpdateMagAction *mpConvergedUpdateMagAction;  ///< Check if
  /// optimizer
  /// converged based
  /// on magnitude of
  /// the update
  /// vector
  CheckConvergedResidualAction *mpConvergedResidualAction;  ///< Check if
  /// optimizer
  /// converged based on
  /// change in residual
  /// error
  UpdateHelpersAction *mpUpdateHelpersAction;  ///< Update the pose chain helper transforms
  UpdateTotalIterationsAction *mpUpdateTotalIterationsAction;

  std::vector<std::tuple<int, int, std::string>> mvOutlierMeasurementIdx;  ///< Holds outlier measurements, encoded as
  /// described in GetOutlierMeasurements()

  /// A map of pose chain => helper class, used to calculate pose chain
  /// transforms only once per chain rather than inside the error function of
  /// each measurement.
  HelperMap mmHelpers;

  TaylorCameraMap mmCameraModels;  ///< The map of camera models

  bool mbConverged;         ///< Is the solution converged?
  bool mbHitMaxIterations;  ///< Did the algorithm hit an interation ceiling
  double mdLastMaxCov;      ///< The maximum point depth covariance from the last run

  int mnCurrId;  ///< Used to give IDs to poses and points, incremented each time
  /// AddPose() or AddPoint() is called
  bool mbUseRobust;  ///< Use robustification of errors?
  bool mbUseTukey;   ///< Use the Tukey test for outliers
  bool mbVerbose;    ///< Print lots of stuff?

  int mnTotalIterations;
};

#endif  // MCPTAM_CHAINBUNDLE_H
