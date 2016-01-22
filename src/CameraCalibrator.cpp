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

//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Parts of the code are from PTAM, which is
// Copyright 2008 Isis Innovation Limited
//
//=========================================================================================

#include <mcptam/CameraCalibrator.h>
#include <mcptam/TaylorCamera.h>
#include <mcptam/CalibImageTaylor.h>
#include <mcptam/OpenGL.h>
#include <gvars3/instances.h>
#include <TooN/SVD.h>
#include <TooN/Cholesky.h>
#include <cvd/image_io.h>
#include <ros/ros.h>

using namespace TooN;
using namespace GVars3;

CameraCalibrator::CameraCalibrator()
  : mVideoSource(false), mpGLWindow(InitWindow("CameraCalibrator")), mpCamera(NULL), mNodeHandlePriv("~")
{
  GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Remove", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Optimize", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.GrabMore", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("MouseDown", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);

  GUI.ParseLine("UseExistingCalib=0");
  GUI.ParseLine("Draw3DGrid=1");
  GUI.ParseLine("DrawMask=0");

  GUI.ParseLine("GLWindow.AddMenu CalibMenu");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize CameraCalibrator.Optimize");
  GUI.ParseLine("CalibMenu.AddMenuToggle Live \"Use Existing\" UseExistingCalib Live");
  GUI.ParseLine("CalibMenu.AddMenuToggle Live \"Draw Mask\" DrawMask Live");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.GrabMore ");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Remove CameraCalibrator.Remove");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Save CameraCalibrator.SaveCalib");
  GUI.ParseLine("CalibMenu.AddMenuToggle Opti \"3D Grid\" Draw3DGrid Opti");

  mCamName = mVideoSource.GetCamName();
  mirSize = mVideoSource.GetSize();
  CVD::ImageRef irFullScaleSize = mVideoSource.GetFullScaleSize();

  if (mirSize != irFullScaleSize)
  {
    ROS_ERROR("CameraCalibrator: Can't calibrate camera with binning turned on, relaunch camera with binning off");
    ROS_ERROR_STREAM("CameraCalibrator: Got image size of " << mirSize.x << "," << mirSize.y
                     << "  and full scale size of " << irFullScaleSize.x << ","
                     << irFullScaleSize.y);
    ros::shutdown();
    return;
  }

  std::string maskFile;
  mNodeHandlePriv.getParam("mask", maskFile);

  if (!maskFile.empty())
  {
    mimMask = CVD::img_load(maskFile);
  }
  else
  {
    mimMask = CVD::Image<CVD::byte>(mirSize, 255);
  }

  Reset();
  mbDone = false;
}

CameraCalibrator::~CameraCalibrator()
{
  if (mpGLWindow)
    delete mpGLWindow;

  if (mpCamera)
    delete mpCamera;

  for (unsigned i = 0; i < mvpCalibImgs.size(); ++i)
    delete mvpCalibImgs[i];
}

void CameraCalibrator::Run()
{
  static gvar3<int> gvnUseExistingCalib("UseExistingCalib", 0, HIDDEN | SILENT);
  static gvar3<int> gvnDraw3DGrid("Draw3DGrid", 1, HIDDEN | SILENT);
  static gvar3<int> gvnDrawMask("DrawMask", 0, HIDDEN | SILENT);

  int nPrevUseExisting = *gvnUseExistingCalib;

  while (!mbDone)
  {
    if (*gvnUseExistingCalib != nPrevUseExisting)
    {
      Reset();
    }

    nPrevUseExisting = *gvnUseExistingCalib;

    // Set up openGL
    mpGLWindow->SetupViewport();
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();

    CVD::Image<CVD::byte> imFrameBW;
    bool bGotNewFrame = mVideoSource.GetAndFillFrameBW(ros::WallDuration(0.1), imFrameBW);

    if (!mbOptimizing)
    {
      // Show the capturing menu
      GUI.ParseLine("CalibMenu.ShowMenu Live");

      if (*gvnDrawMask)
      {
        glDrawPixels(mimMask);
      }
      else if (bGotNewFrame)
      {
        // Clear screen with black
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);

        glDrawPixels(imFrameBW);

        TaylorCamera* pCamera = *gvnUseExistingCalib ? mpCamera : NULL;
        CalibImageTaylor* pCalibImg = new CalibImageTaylor(CVD::ImageRef(), mpGLWindow, pCamera, mimMask);

        if (pCalibImg->MakeFromImage(imFrameBW, CVD::ImageRef(), false))
        {
          if (mbGrabNextFrame)
          {
            mvpCalibImgs.push_back(pCalibImg);
            // mvpCalibImgs.back()->Draw3DGrid(*mpCamera, false);
            mbGrabNextFrame = false;

            // Flash screen white to indicate capture
            glClearColor(1, 1, 1, 0.5);
            glClear(GL_COLOR_BUFFER_BIT);
          }
        }
      }
    }
    else
    {
      // Show the optimization menu
      GUI.ParseLine("CalibMenu.ShowMenu Opti");

      if (!mbInit)
      {
        InitOptimization(mbFindCenter);
        mbInit = true;
      }

      OptimizeOneStepLM();

      // Draw the currently selected image and the 3D grid overlay
      glDrawPixels(mvpCalibImgs[mnImageToShow]->mImage);

      if (*gvnDraw3DGrid)
        mvpCalibImgs[mnImageToShow]->Draw3DGrid(*mpCamera, true);
      else
        mvpCalibImgs[mnImageToShow]->DrawImageGrid();
    }

    std::ostringstream ost;
    ost << "Camera Calibration: Grabbed " << mvpCalibImgs.size() << " images." << std::endl;
    if (!mbOptimizing)
    {
      ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << std::endl;
      ost << "and then press \"Optimize\"." << std::endl;
      ost << "Take enough shots (4+) at different angles to get points " << std::endl;
      ost << "into all parts of the image (corners too.) The whole grid " << std::endl;
      ost << "doesn't need to be visible so feel free to zoom in." << std::endl;
    }
    else
    {
      ost << "Current image: " << mnImageToShow + 1 << " / " << mvpCalibImgs.size() << std::endl;
      ost << "Current RMS pixel error is " << mdMeanPixelError << std::endl;
      ost << "Current camera params are  " << mpCamera->GetParams() << std::endl;
      ost << "(That would be a pixel aspect ratio of " << mpCamera->PixelAspectRatio() << ")" << std::endl;
      ost << "Check fit by looking through the grabbed images." << std::endl;
      ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << std::endl;
      ost << "Press \"save\" to save calibration to the camera and exit." << std::endl;
    }

    mpGLWindow->DrawCaption(ost.str());
    mpGLWindow->DrawMenus();
    mpGLWindow->HandlePendingEvents();
    mpGLWindow->swap_buffers();

    // GUI interface
    while (!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
  }
}

// Delete all acquired data, reset camera model parameters to defaults
void CameraCalibrator::Reset()
{
  if (mpCamera)
    delete mpCamera;

  mpCamera = new TaylorCamera(mirSize, mirSize, mirSize);

  static gvar3<int> gvnUseExistingCalib("UseExistingCalib", 0, HIDDEN | SILENT);
  if (*gvnUseExistingCalib)
  {
    TooN::Vector<9> v9Params = mVideoSource.GetParams();
    std::cout << "Setting params: " << v9Params << std::endl;
    mpCamera->SetParams(v9Params);
  }

  mbGrabNextFrame = false;
  mbOptimizing = false;
  mbInit = false;
  mbFindCenter = true;
  mnImageToShow = 0;

  for (unsigned i = 0; i < mvpCalibImgs.size(); ++i)
    delete mvpCalibImgs[i];

  mvpCalibImgs.clear();
}

// // This can be used with GUI.RegisterCommand to capture user input
void CameraCalibrator::GUICommandCallBack(void* ptr, std::string command, std::string params)
{
  Command c;
  c.command = command;
  c.params = params;
  static_cast<CameraCalibrator*>(ptr)->mqCommands.push(c);
}

// Deals with user interface commands
void CameraCalibrator::GUICommandHandler(std::string command, std::string params)
{
  if (command == "CameraCalibrator.Optimize")
  {
    if (mvpCalibImgs.size() > 0)
      mbOptimizing = true;

    return;
  }

  if (command == "CameraCalibrator.GrabMore")
  {
    mbOptimizing = false;
    mbInit = false;
    mbFindCenter = true;
    return;
  }

  if (command == "CameraCalibrator.Reset")
  {
    Reset();
    return;
  }

  if (command == "CameraCalibrator.Remove")
  {
    if (mvpCalibImgs.size() < 2)
      Reset();
    else
    {
      delete mvpCalibImgs[mnImageToShow];
      mvpCalibImgs.erase(mvpCalibImgs.begin() + mnImageToShow);

      if (mnImageToShow == (int)mvpCalibImgs.size())
        mnImageToShow = 0;
    }

    return;
  }

  if (command == "CameraCalibrator.GrabNextFrame")
  {
    mbGrabNextFrame = true;
    return;
  }

  if (command == "KeyPress")
  {
    if (params == "Space")
    {
      mbGrabNextFrame = true;
      return;
    }
  }

  if (command == "CameraCalibrator.ShowNext")
  {
    ++mnImageToShow;

    if (mnImageToShow >= (int)mvpCalibImgs.size())
      mnImageToShow = 0;

    return;
  }

  if (command == "CameraCalibrator.SaveCalib")
  {
    Vector<9> v9Params = mpCamera->GetParams();

    bool bSuccess = mVideoSource.SaveParams(v9Params);

    if (!bSuccess)
    {
      ROS_ERROR("Couldn't save params to camera!!");
    }

    return;
  }
  if (command == "exit" || command == "quit")
  {
    mbDone = true;
    return;
  }

  if (command == "MouseDown")
  {
    std::stringstream ss;
    ss << params;

    int button;
    int state;
    CVD::ImageRef where;
    ss >> button >> state >> where.x >> where.y;

    Vector<3> v3UnProj = mpCamera->UnProject(vec(where));
    ROS_INFO_STREAM("Clicked: " << where << "  UnProj: " << v3UnProj);

    return;
  }

  ROS_WARN_STREAM("CameraCalibrator: Unhandled command in GUICommandHandler: " << command);
}

// Finds the best image poses, camera polynomials and center of projection using linear methods
void CameraCalibrator::InitOptimization(bool bFindCenter)
{
  if (bFindCenter)
  {
    // Find best center pose with exhaustive search, this many iterations
    int nCenterSearchIterations = 20;

    // The center of the image
    Vector<2> v2DefaultCenter;
    v2DefaultCenter[0] = mpCamera->GetImageSize().x / 2.0;
    v2DefaultCenter[1] = mpCamera->GetImageSize().y / 2.0;

    // Start at image center for center of projection
    Vector<2> v2StartCenter = v2DefaultCenter;

    // A reasonable spread to start
    Vector<2> v2Spread = v2StartCenter / 4;

    // Check 25 points at each level
    CVD::ImageRef irNumPoints(5, 5);
    Vector<2> v2BestPos;
    Vector<4> v4BestParams;
    double dMinError;

    for (int i = 0; i < nCenterSearchIterations; i++)
    {
      FindBestCenter(v2StartCenter, v2Spread, irNumPoints, v2BestPos, v4BestParams, dMinError);

      // Update where to start for next iteration
      v2StartCenter = v2BestPos;
      v2Spread *= 0.5;  // Tighten the search area
    }

    ROS_INFO_STREAM("CameraCalibrator: Found best center of projection: " << v2BestPos << " with error: " << dMinError);

    // Update camera parameters
    Vector<9> v9CurrParams = mpCamera->GetParams();
    v9CurrParams.slice<0, 4>() = v4BestParams;
    v9CurrParams.slice<4, 2>() = v2BestPos;
    mpCamera->SetParams(v9CurrParams);
  }

  ComputeParamsUpdatePoses(mvpCalibImgs, mpCamera->GetCenter());

  // Initialize these in preparation for optimization
  mdLambda = 0.0001;
  mdLambdaFactor = 2.0;
}

// Takes one step of Levenberg-Marquardt optimization for the whole system (camera poses and parameters)
bool CameraCalibrator::OptimizeOneStepLM()
{
  // This is a textbook implementation of LM optimization according to Hartley & Zisserman MVG book.
  // No attempts have been made to save on any matrix multiplications.

  int nViews = mvpCalibImgs.size();
  int nPoints = 0;
  for (int i = 0; i < nViews; ++i)
  {
    nPoints += mvpCalibImgs[i]->mvGridCorners.size();
  }

  // The big jacobian matrices and the error
  Matrix<> A = Zeros(2 * nPoints, 6 * nViews);
  Matrix<> B(2 * nPoints, 9);
  Vector<> epsilon(2 * nPoints);

  double dSumSquaredError = 0.0;
  int nTotalMeas = 0;
  int nMeasBase = 0;

  for (int n = 0; n < nViews; n++)
  {
    // Get the error and jacobian for each calibration image
    int nMotionBase = n * 6;
    std::vector<CalibImageTaylor::ErrorAndJacobians> vEAJ = mvpCalibImgs[n]->Project(*mpCamera);

    // If this isn't true then we're starting off with bad initial conditions so we won't get anywhere
    if (vEAJ.size() != mvpCalibImgs[n]->mvGridCorners.size())
      return false;

    // For each corner....
    for (unsigned int i = 0; i < vEAJ.size(); i++)
    {
      CalibImageTaylor::ErrorAndJacobians& EAJ = vEAJ[i];

      // Insert jacobians and error into big matrices
      A.slice(nMeasBase, nMotionBase, 2, 6) = EAJ.m26PoseJac;
      B.slice(nMeasBase, 0, 2, 9) = EAJ.m2NCameraJac;
      epsilon.slice(nMeasBase, 2) = EAJ.v2Error;

      nMeasBase += 2;

      dSumSquaredError += EAJ.v2Error * EAJ.v2Error;
      ++nTotalMeas;
    }
  }

  mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);

  // The following is textbook implementation (not very efficient though)
  Matrix<> U = A.T() * A;
  Matrix<> V = B.T() * B;
  Matrix<> W = A.T() * B;
  Vector<> epsilon_A = A.T() * epsilon;
  Vector<> epsilon_B = B.T() * epsilon;

  // U is now Ustar
  for (int i = 0; i < U.num_rows(); ++i)
    U[i][i] *= (1.0 + mdLambda);

  // V is now Vstar
  for (int i = 0; i < V.num_rows(); ++i)
    V[i][i] *= (1.0 + mdLambda);

  V = Cholesky<>(V).get_inverse();  // V is now Vstar_inverse

  Matrix<> Y = W * V;

  Vector<> vCameraPoseUpdate = Cholesky<>(U - Y * W.T()).backsub(epsilon_A - Y * epsilon_B);
  Vector<> vCameraParamsUpdate = V * (epsilon_B - W.T() * vCameraPoseUpdate);

  // Save old camera params
  Vector<9> v9OldParams = mpCamera->GetParams();

  // Apply updates
  for (int n = 0; n < nViews; n++)
  {
    mvpCalibImgs[n]->mse3CamFromWorldNew =
      SE3<>::exp(vCameraPoseUpdate.slice(n * 6, 6)) * mvpCalibImgs[n]->mse3CamFromWorld;
  }

  mpCamera->UpdateParams(vCameraParamsUpdate);

  // Get the new errors
  double dSumSquaredErrorNew = 0.0;
  for (int n = 0; n < nViews; n++)
  {
    std::vector<Vector<2>> vErrors = mvpCalibImgs[n]->ProjectGetOnlyErrors(*mpCamera, true);
    for (unsigned int i = 0; i < vErrors.size(); i++)
    {
      dSumSquaredErrorNew += vErrors[i] * vErrors[i];
    }
  }

  ROS_DEBUG_STREAM("CameraCalibrator: Sum squared error BEFORE LM update: " << dSumSquaredError);
  ROS_DEBUG_STREAM("CameraCalibrator: Sum squared error AFTER LM update: " << dSumSquaredErrorNew);

  if (dSumSquaredErrorNew < dSumSquaredError)  // step was good, so keep params
  {
    ROS_DEBUG("CameraCalibrator: Good step, keeping params");
    ModifyLambda_GoodStep();
    for (int n = 0; n < nViews; n++)
    {
      mvpCalibImgs[n]->mse3CamFromWorld = mvpCalibImgs[n]->mse3CamFromWorldNew;
    }
  }
  else  // step was bad, increase mdLambda and revert parameters
  {
    ROS_DEBUG("CameraCalibrator: Bad step, reverting params");
    ModifyLambda_BadStep();
    mpCamera->SetParams(v9OldParams);
  }

  ROS_DEBUG_STREAM("CameraCalibrator: Cam Params at end of LM step: " << mpCamera->GetParams());

  return true;
}

void CameraCalibrator::FindBestCenter(Vector<2> v2StartCenter, Vector<2> v2Spread, CVD::ImageRef irNumPoints,
                                      Vector<2>& v2BestPos, Vector<4>& v4BestParams, double& dMinError)
{
  // This will find the best center around v2StartCenter in the box of size v2Spread
  // Will test points spaced at numSteps apart, which should be odd

  // Make sure they're odd
  if (irNumPoints[0] % 2 != 1)
    irNumPoints[0]++;

  if (irNumPoints[1] % 2 != 1)
    irNumPoints[1]++;

  Vector<2> v2StartPos = v2StartCenter - 0.5 * v2Spread;
  Vector<2> v2StepSize;
  v2StepSize[0] = v2Spread[0] / (irNumPoints[0] - 1);
  v2StepSize[1] = v2Spread[1] / (irNumPoints[1] - 1);

  int nViews = mvpCalibImgs.size();

  dMinError = 1e100;
  v2BestPos = v2StartCenter;

  for (int y = 0; y < irNumPoints[1]; y++)
  {
    for (int x = 0; x < irNumPoints[0]; x++)
    {
      Vector<2> v2CurrentPos = v2StartPos;
      v2CurrentPos[0] += x * v2StepSize[0];
      v2CurrentPos[1] += y * v2StepSize[1];

      // Update the calibration image poses and get new camera polynomial coeffs
      Vector<4> v4NewParams = ComputeParamsUpdatePoses(mvpCalibImgs, v2CurrentPos);

      // Update the polynomial coeffs and projection center
      Vector<9> v9CurrParams = mpCamera->GetParams();
      v9CurrParams.slice<0, 4>() = v4NewParams;
      v9CurrParams.slice<4, 2>() = v2CurrentPos;
      mpCamera->SetParams(v9CurrParams);

      // Calculate the error
      double dSumSquaredError = 0.0;
      for (int n = 0; n < nViews; n++)
      {
        std::vector<Vector<2>> vErrors = mvpCalibImgs[n]->ProjectGetOnlyErrors(*mpCamera);
        for (unsigned int i = 0; i < vErrors.size(); i++)
        {
          dSumSquaredError += vErrors[i] * vErrors[i];
        }
      }

      // Is this error the best one yet?
      if (dSumSquaredError < dMinError)
      {
        dMinError = dSumSquaredError;
        v2BestPos = v2CurrentPos;
        v4BestParams = v4NewParams;
      }
    }
  }
}

// Compute the z component of the translation vector of the CalibImateTaylor's poses, and the polynomial
// coefficients of the camera model.
TooN::Vector<4> CameraCalibrator::ComputeParamsUpdatePoses(std::vector<CalibImageTaylor*> vpCalibImgs,
    TooN::Vector<2> v2Center)
{
  int nViews = vpCalibImgs.size();
  int nPoints = 0;

  for (int i = 0; i < nViews; ++i)
  {
    nPoints += vpCalibImgs[i]->mvGridCorners.size();
    vpCalibImgs[i]->GuessInitialPose(
      v2Center);  // Update internal pose, make sure this is called before building big matrix!
  }

  // This follows Scaramuzza thesis section 3.2.2
  TooN::Matrix<> M(2 * nPoints, 4 + nViews);
  TooN::Vector<> b(2 * nPoints);

  int nIdx = 0;

  for (int i = 0; i < nViews; ++i)
  {
    std::pair<TooN::Matrix<>, TooN::Vector<>> MandB =
        vpCalibImgs[i]->BuildIntrinsicMatrixEntries(v2Center, 4, i, nViews);
    int nNumEntries = MandB.second.size();

    // Insert into big matrix
    M.slice(nIdx, 0, nNumEntries, 4 + nViews) = MandB.first;
    b.slice(nIdx, nNumEntries) = MandB.second;

    nIdx += nNumEntries;
  }

  TooN::SVD<> svd(M);

  // Some camera parameters are very small (ie in the affine matrix), to handle this correctly we need to override SVD's
  // condition number
  double dConditionNumber = 1 / (M.num_rows() * std::numeric_limits<double>::epsilon());
  TooN::Vector<> vParams = svd.backsub(b, dConditionNumber);

  // Update translation vectors with t3
  for (int i = 0; i < nViews; ++i)
  {
    vpCalibImgs[i]->mse3CamFromWorld.get_translation()[2] = vParams[4 + i];
    if (vParams[4 + i] < 0)
    {
      ROS_DEBUG_STREAM("Z component of calib image " << i << " is " << vParams[4 + i]);
    }
  }

  return vParams.slice(0, 4);  // this is the new value of camera parameters
}
