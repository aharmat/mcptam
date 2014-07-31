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


// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//=========================================================================================
//
// Modifications
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Minor renaming and reshuffling of variables. Addition of two friend
// functions that allow a TrackerData object controlled through a boost
// intrusive pointer to increment/decrement the map point's and MKF's "using" counter.
//
//=========================================================================================

#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H

#include <mcptam/PatchFinder.h>
#include <mcptam/TaylorCamera.h>
#include <mcptam/MapPoint.h>
#include <TooN/TooN.h>

// This is very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

/** @brief Contains all the intermediate results associated with a map-point that the tracker keeps up-to-date. 
 * 
 * TrackerData basically handles all the tracker's point-projection jobs, and also contains the PatchFinder which does the image search.
 */
class TrackerData
{

public:

  /// Only a parameterized constructor is available to ensure requirements are met
  /** @param pPoint Pointer to the MapPoint that will be worked on
   *  @param irImageSize The image size to use for checking projection inliers
   */
  TrackerData(MapPoint *pPoint, CVD::ImageRef irImageSize) 
  : mPoint(*pPoint)
  , mirImageSize(irImageSize)
    {};
  
  MapPoint &mPoint;  ///< The MapPoint being worked on
  PatchFinder mFinder;  ///< The PatchFinder
  
  // Projection itermediates:
  TooN::Vector<3> mv3Cam;        ///< Coordinates in current camera frame
  TooN::Vector<2> mv2Image;      ///< Pixel coords in level zero
  TooN::Matrix<2> mm2CamDerivs;  ///< Camera projection derivatives
  bool mbInImage;                ///< Is the projection of the point within the image boundaries?
  
  int mnSearchLevel;    ///< The pyramid level that the PatchFinder determined should hold the point
  bool mbSearched;      ///< Has the PatchFinder searched for this point?
  bool mbFound;         ///< Did the PatchFinder find the point?
  bool mbDidSubPix;     ///< Did the PatchFinder do sub-pixel iterations?
  TooN::Vector<2> mv2Found;      ///< Pixel coordinatess of found patch at level zero
  double mdSqrtInvNoise;   ///< Approximate inverse noise based on search level
  
  CVD::ImageRef mirImageSize;  ///< The image size used to check for projection inliers
  
  // Stuff for pose update:
  TooN::Vector<2> mv2Error;            ///< The projection error
  TooN::Vector<2> mv2Error_CovScaled;  ///< The projection error scaled by the inverse noise term
  TooN::Matrix<2,6> mm26Jacobian;   ///< Jacobian of projection with respect to camera position
  TooN::Matrix<2,3> mm23Jacobian;   ///< Jacobian of projection with respect to point position
  
  /** @brief Project point into image given certain pose and camera.
   * 
   * This can bail out at several stages if the point will not be properly in the image.
   * @param se3CamFromWorld Pose of camera in world frame
   * @param camera The camera model used for projection
   */
  inline void Project(const TooN::SE3<> &se3CamFromWorld, TaylorCamera &camera)
  {
    mbInImage = false;
    mv3Cam = se3CamFromWorld * mPoint.mv3WorldPos;
    
    if(!TooN::isfinite(mv3Cam))
    {
      std::cout<<"se3CamFromWorld: "<<std::endl<<se3CamFromWorld<<std::endl;
      std::cout<<"mPoint.mv3WorldPos: "<<mPoint.mv3WorldPos<<std::endl;
      std::cout<<"mv3Cam: "<<mv3Cam<<std::endl;
      ROS_BREAK();
    }
    
    mv2Image = camera.Project(mv3Cam);
    
    if(camera.Invalid())
    {
      return;
    }
    
    if(mv2Image[0] < 0 || mv2Image[1] < 0 || mv2Image[0] > mirImageSize[0] || mv2Image[1] > mirImageSize[1])
    {
      return;
    }
    mbInImage = true;
  }
  
  /** @brief Get the projection derivatives (depends only on the camera model.)
   * 
   * This is called Unsafe because it depends on the camera caching 
   * results from the previous projection: Only do this right after the same point has been projected!
   * @param camera The camera model whose derivatives will be stored
   */
  inline void GetDerivsUnsafe(TaylorCamera &camera) 
  {
    mm2CamDerivs = camera.GetProjectionDerivs();
  }
  
  /// Does projection and gets camera derivatives all in one.
  /** @param se3CamFromWorld The camera pose in world frame
   *  @param camera The camera model
   */
  inline void ProjectAndDerivs(const TooN::SE3<>& se3CamFromWorld, TaylorCamera &camera)
  {
    Project(se3CamFromWorld, camera);
    if(mbFound)
      GetDerivsUnsafe(camera);
  }
  
  /** @brief Jacobian of projection W.R.T. the camera position
   * 
   * This function returns the derivative of the pixel (x,y) location of a projected
   * point  with respect to the six degrees of freedom that make up a camera's pose.
   * In a multi-camera setup where the pose of a camera is fixed with respect to a base
   * pose, but the base itself can move, getting the Jacobian is a bit trickier than the
   * single camera case. In essence we need to calculate the derivative of the projected point
   * with respect to the pose of the base, which is the only thing that can move. See
   * source code for details.
   */
  inline void CalcJacobian(const TooN::SE3<>& se3BaseFromWorld, const TooN::SE3<>& se3CamFromBase)
  {
    const TooN::SE3<> se3CamFromWorld = se3CamFromBase * se3BaseFromWorld;
    const TooN::Vector<3> v3Cam = se3CamFromWorld * mPoint.mv3WorldPos;  // The point in the camera coordinate frame
    const TooN::Vector<3> v3Base = se3BaseFromWorld * mPoint.mv3WorldPos;  // The point in the base (MultiKeyFrame) coordinate frame
    const TooN::Vector<4> v4Base = unproject(v3Base);  // The SE3 generator field function works on 4-vectors
    
    TooN::SE3<> se3CamFromBase_only_rot = se3CamFromBase;  // Only the rotational component of the camera-from-base transform is needed
    se3CamFromBase_only_rot.get_translation() = TooN::Zeros;
    
    TooN::Vector<3> v3_dTheta, v3_dPhi;
    TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);
    
    // For each of six degrees of freedom...
    for(int m=0; m<6; m++)
    {
      // Get the motion of the point in the base frame when the pose of the base changes by one of the degrees of freedom
      const TooN::Vector<4> v4Motion_Base = TooN::SE3<>::generator_field(m, v4Base); 
      // Then the motion of the point in the camera frame only depends on the relative rotation between camera and base, not the translation
      const TooN::Vector<4> v4Motion_Cam = se3CamFromBase_only_rot * v4Motion_Base;  // CHECK!! GOOD
      
      TooN::Vector<2> v2CamSphereMotion;
      v2CamSphereMotion[0] = v3_dTheta * v4Motion_Cam.slice<0,3>();  // theta component
      v2CamSphereMotion[1] = v3_dPhi * v4Motion_Cam.slice<0,3>();  // phi component
    
      // The camera derivatives are motion of pixel relative to motion on unit sphere, so a multiplication gets the desired Jacobian entry
      mm26Jacobian.T()[m] = mm2CamDerivs * v2CamSphereMotion;
    }
    
    // For each of three degrees of freedom...
    for(int m=0;m<3;m++)
    {
      // Get the motion of the point in the camera frame when the position of the point changes by one of the degrees of freedom
      const TooN::Vector<3> v3Motion = se3CamFromWorld.get_rotation().get_matrix().T()[m];  // CHECK!! GOOD
      
      // Convert to motion on the sphere
      TooN::Vector<2> v2CamSphereMotion;
      v2CamSphereMotion[0] = v3_dTheta * v3Motion;  // theta component
      v2CamSphereMotion[1] = v3_dPhi * v3Motion;  // phi component
      
      // Convert to motion on the image
      mm23Jacobian.T()[m] = mm2CamDerivs * v2CamSphereMotion;
    }
  }
  
  /// Allows updating the the error linearly instead of reprojecting the point, saves time
  /** @param v6 The update vector */
  inline void LinearUpdate(const TooN::Vector<6> &v6)
  {
    mv2Image += mm26Jacobian * v6;
  }
  
  /// Allows a TrackerData object accessed through a boost intrusive_ptr to increment the "using" count of its MapPoint
  friend void intrusive_ptr_add_ref(TrackerData const* op)
  {
    ++op->mPoint.mnUsing;
    ++op->mPoint.mpPatchSourceKF->mpParent->mnUsing;
  }

  /// Allows a TrackerData object accessed through a boost intrusive_ptr to decrement the "using" count of its MapPoint
  friend void intrusive_ptr_release(TrackerData const* op)
  {
    --op->mPoint.mnUsing;
    --op->mPoint.mpPatchSourceKF->mpParent->mnUsing;
  }
  
};


#endif
