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
 * \file SmallBlurryImage.h
 * \brief Declaration of SmallBlurryImage class
 *
 * Large parts of this code are from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * SmallBlurryImage - A small and blurry representation of an image.
 * used by the relocaliser.
 *
 *
 * Modifications
 * Copyright 2014   Adam Harmat, McGill University (adam.harmat@mail.mcgill.ca)
 *                  Michael Tribou, University of Waterloo
 *(mjtribou@uwaterloo.ca)
 *
 * The only modification made was changing the camera model to TaylorCamera and
 *adding
 * another camera argument to SE3fromSE2 so that different source and target
 *cameras can
 * be used.
 *
 ****************************************************************************************/

#ifndef MCPTAM_SMALLBLURRYIMAGE_H
#define MCPTAM_SMALLBLURRYIMAGE_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <TooN/se2.h>
#include <TooN/se3.h>
#include <utility>

class TaylorCamera;
class KeyFrame;

/** @brief A small and blurry representation of an image, used by the
 *Relocalizer.
 *
 *  I made minimial changes to the code, and don't understand most of it in
 *detail, so most of
 *  the Doxygen documentation is made up of comments from the original PTAM
 *
 *  All images are resized to sirSize which is 40x30 by default. However, images
 *from very
 *  different lens types will not match too nicely.
 *   */
class SmallBlurryImage
{
public:
  /// Default constructor
  SmallBlurryImage();

  /** @brief Construct and fill internal variables immediately from KeyFrame
   *  @param kf The KeyFrame whose image will be used to make the small blurry
   * image
   *  @param dBlur The width of the gaussian convolution window used to blur */
  explicit SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);

  /** @brief Fill internal variables from KeyFrame
   *  @param kf The KeyFrame whose image will be used to make the small blurry
   * image
   *  @param dBlur The width of the gaussian convolution window used to blur */
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);

  /// Make the jacobians (actually, no more than a gradient image) of the
  /// blurred template
  void MakeJacs();

  /** @brief Calculate the zero-mean SSD between one image and the next.
   *
   * Since both are zero mean already, just calculate the SSD. Both small blurry
   *images need to be
   * the same size for this to work.
   * @param other The other image
   * @return Sum of squared differences */
  double ZMSSD(SmallBlurryImage &other);

  /** @brief Find an SE2 which best aligns an SBI to a target
   *
   * Do this by ESM-tracking a la Benhimane & Malis. Both small blurry images
   *need to be the same
   * size for this work.
   * @param other The small blurry image we're comparing to
   * @param nIterations The number of iterations of the algorithm after which we
   *call it converged
   * @return The found transform and its final score (lower is better) */
  std::pair<TooN::SE2<>, double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);

  /** @brief What is the 3D camera rotation (zero trans) SE3<> which causes an
   *input image SO2 rotation?
   *
   * Do this by projecting two points, and then iterating the SE3<> (SO3
   *actually) until convergence.
   * It might seem stupid doing this so precisely when the whole SE2-finding is
   *one big hack, but hey.
   * @param se2 The SE2 transform we need to find an SE3 for
   * @param cameraSrc The first image's camera model
   * @param cameraTarget The second image's camera model
   * @return The SE3 transformation that best explains the input and the inverse
   *covariance matrix of the minimization */
  static TooN::SE3<> SE3fromSE2(TooN::SE2<> se2, TaylorCamera &cameraSrc, TaylorCamera &cameraTarget);
  // static void SE3fromSE2(TooN::SE2<> se2, TaylorCamera& cameraSrc,
  // TaylorCamera& cameraTarget, TooN::SE3<>& se3, TooN::Matrix<6>& m6Cov);

  static CVD::ImageRef sirSize;  ///< The common small image size used by all
  /// SBIs, please read SmallBlurryImage
  /// documentation to see why this is important

protected:
  CVD::Image<CVD::byte> mimSmall;  ///< The downsampled small image
  CVD::Image<float> mimTemplate;   ///< Same as mimSmall but with the mean image
  /// intensity subtracted, used in all image
  /// operations
  CVD::Image<TooN::Vector<2>> mimImageJacs;  ///< The gradient image of mimTemplate
  bool mbMadeJacs;                           ///< Have we made the gradient image?
};

#endif  // MCPTAM_SMALLBLURRYIMAGE_H
