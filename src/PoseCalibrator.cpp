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
//=========================================================================================

#include <mcptam/PoseCalibrator.h>
#include <mcptam/VideoSourceMulti.h>
#include <mcptam/KeyFrameViewer.h>
#include <mcptam/BundleAdjusterSingle.h>
#include <mcptam/MapMakerCalib.h>
#include <mcptam/TrackerCalib.h>
#include <mcptam/Utility.h>
#include <mcptam/OpenGL.h>
#include <mcptam/Map.h>
#include <gvars3/instances.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>

using namespace GVars3;
using namespace TooN;

PoseCalibrator::PoseCalibrator() : SystemBase("PoseCalibrator", true, true)
{
  if (mpVideoSourceMulti->GetNumGroups() != 1)
  {
    ROS_FATAL_STREAM("PoseCalibrator: All cameras need to be triggered together in one group.");
    ROS_FATAL_STREAM("PoseCalibrator: Currently you have " << mpVideoSourceMulti->GetNumGroups() << " camera groups.");
    ros::shutdown();
    return;
  }

  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowNextKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("ShowPrevKeyFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("GrabMore", GUICommandCallBack, this);
  GUI.RegisterCommand("SaveCalib", GUICommandCallBack, this);
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("Optimize", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("InitTracker", GUICommandCallBack, this);

  // Add menu groupings and buttons to the GL window
  /* Menu
  Root
    |
    +-> Reset
    +-> Init
    +-> Images
    | |
    | +-> Back
    | +-> Level0 Points
    | +-> Glare Masks
    | +-> Draw Masks
    +-> Keyframes
    | |
    | +-> Back
    | +-> Candidates (on/off)
    | +-> Show Next
    | +-> Show Prev
    | +-> Level
    +-> Optimize
    | |
    | +-> Grab More
    | +-> Save Calib
    +-> Level
  */

  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");

  GUI.ParseLine("DrawMasks=0");
  GUI.ParseLine("DrawCandidates=0");
  GUI.ParseLine("DrawLevel=0");
  GUI.ParseLine("GlareMasking=0");
  GUI.ParseLine("LevelZeroPoints=1");

  bool bLevelZeroPoints;
  mNodeHandlePriv.param<bool>("level_zero_points", bLevelZeroPoints, true);

  static gvar3<int> gvnLevelZeroPoints("LevelZeroPoints", 0, HIDDEN | SILENT);
  *gvnLevelZeroPoints = bLevelZeroPoints;

  // Main Menu
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Init InitTracker Root");
  GUI.ParseLine("Menu.AddMenuButton Root \"Images...\" \"\" Images");
  GUI.ParseLine("Menu.AddMenuButton Root \"Keyframes...\" \"\" View");
  GUI.ParseLine("Menu.AddMenuButton Root Optimize Optimize Opti");
  GUI.ParseLine("Menu.AddMenuSlider Root \"Level\" DrawLevel 0 3 Root");

  // Images Menu
  GUI.ParseLine("Menu.AddMenuButton Images \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Level0 Pts\" LevelZeroPoints Images");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Draw Masks\" DrawMasks Images");
  GUI.ParseLine("Menu.AddMenuToggle Images \"Glare Mask\" GlareMasking Images");

  // View Keyframes
  GUI.ParseLine("Menu.AddMenuButton View \"< Back\" \"\" Root");
  GUI.ParseLine("Menu.AddMenuToggle View \"Candidates\" DrawCandidates View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Next\" ShowNextKeyFrame View");
  GUI.ParseLine("Menu.AddMenuButton View \"Show Prev\" ShowPrevKeyFrame View");
  GUI.ParseLine("Menu.AddMenuSlider View \"Level\" DrawLevel 0 3 View");

  // Optimize Menu
  GUI.ParseLine("Menu.AddMenuButton Opti \"Grab More\" GrabMore Root");
  GUI.ParseLine("Menu.AddMenuButton Opti \"Save Calib\" SaveCalib Opti");

  mNodeHandlePriv.param<int>("pattern_width", mirPatternSize[0], 0);
  mNodeHandlePriv.param<int>("pattern_height", mirPatternSize[1], 0);
  mNodeHandlePriv.param<double>("square_size", mdSquareSize, 0);

  // Advertise reset before creating mapmaker
  mResetSystemServer = mNodeHandlePriv.advertiseService("reset", &PoseCalibrator::ResetSystemCallback, this);

  mpBundleAdjuster = new BundleAdjusterSingle(*mpMap, mmCameraModels);
  mpMapMaker = new MapMakerCalib(*mpMap, mmCameraModels, *mpBundleAdjuster);
  mpKeyFrameViewer = new KeyFrameViewer(*mpMap, *mpGLWindow, mmDrawOffsets, mpVideoSourceMulti->GetSizes());

  ImageBWMap masksMap = LoadMasks();

  // Create the CalibratorTrackers, one for each camera
  for (TaylorCameraMap::iterator it = mmCameraModels.begin(); it != mmCameraModels.end(); it++)
  {
    std::string camName = it->first;
    mmTrackers[camName] = new TrackerCalib(*mpMap, *mpMapMaker, mmCameraModels, camName, mmDrawOffsets[camName],
                                           mirPatternSize, mdSquareSize, mpGLWindow);
    mmTrackers[camName]->SetMasks(masksMap);
  }

  mnLastNumInit = 0;
  mbOptimizing = false;
  mbDone = false;
}

PoseCalibrator::~PoseCalibrator()
{
  delete mpBundleAdjuster;
  delete mpMapMaker;
  delete mpKeyFrameViewer;

  for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
  {
    delete it->second;
  }
}

// Blocking function that loops indefinitiely
void PoseCalibrator::Run()
{
  static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN | SILENT);

  while (!mbDone && ros::ok())
  {
    mpGLWindow->SetupViewport();
    mpGLWindow->SetupVideoOrtho();
    mpGLWindow->SetupVideoRasterPosAndZoom();

    mbOptimizing = *gvsCurrentSubMenu == "Opti";

    std::string caption;

    if (!mbOptimizing)
    {
      caption = Track();
    }
    else
    {
      caption = Optimize();
    }

    mpGLWindow->DrawCaption(caption);
    mpGLWindow->DrawMenus();
    mpGLWindow->swap_buffers();
    mpGLWindow->HandlePendingEvents();

    // GUI interface
    while (!mqCommands.empty())
    {
      GUICommandHandler(mqCommands.front().command, mqCommands.front().params);
      mqCommands.pop();
    }
  }
}

// Perform all tracking operations
std::string PoseCalibrator::Track()
{
  // This data will be displayed on GUI
  static std::queue<ros::Time> qLoopTimes;
  static std::deque<ros::Duration> qTotalDurations;
  static unsigned int nMaxQueueSize = 10;

  std::stringstream captionStream;  // gather messages here

  // Grab new video frame...
  ros::Time timestamp;
  bool bGotNewFrame = mpVideoSourceMulti->GetAndFillFrameBW(ros::WallDuration(0.1), mmFramesBW, timestamp);

  static gvar3<std::string> gvsCurrentSubMenu("Menu.CurrentSubMenu", "", HIDDEN | SILENT);
  bool bDrawKeyFrames = *gvsCurrentSubMenu == "View";

  if (bGotNewFrame)
  {
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    bool bTriedInit = false;
    bool bNeedToDrop = false;

    mvInitialized.clear();

    ros::Time start = ros::Time::now();
    int nCamNum = 1;

    // Go through all the images that we got
    for (ImageBWMap::iterator it = mmFramesBW.begin(); it != mmFramesBW.end(); it++, nCamNum++)
    {
      TrackerCalib* pTracker = mmTrackers[it->first];

      // If this tracker has not yet found a checkerboard, and none of the trackers in this loop has tried to initialize
      // yet,
      // means it this tracker's turn to find the checkerboard!
      if (pTracker->meCheckerboardStage != TrackerCalib::CHECKERBOARD_RUNNING && !bTriedInit)
      {
        pTracker->TrackFrame(it->second, timestamp, !bDrawKeyFrames, true);
        bTriedInit = true;
      }
      else  // otherwise just try tracking as usual, if tracker has not been initialized yet it'll just do nothing
      {
        pTracker->TrackFrame(it->second, timestamp, !bDrawKeyFrames, false);
      }

      // Print sequential number as overlay
      glColor3f(0, 1.0, 0);
      CVD::ImageRef irOffset = pTracker->mmDrawOffsets[pTracker->mCamName];
      std::stringstream ss;
      ss << "#" << nCamNum;
      mpGLWindow->PrintString(irOffset + CVD::ImageRef(10, 40), ss.str(), 10);

      // If any tracker needs to add a KeyFrame to the Map, we'll generate a new MultiKeyFrame and do just that
      bNeedToDrop |= pTracker->mbNeedToDrop;

      // Collect the trackers that are initialized
      if (pTracker->meCheckerboardStage == TrackerCalib::CHECKERBOARD_SECOND_STAGE ||
          pTracker->meCheckerboardStage == TrackerCalib::CHECKERBOARD_RUNNING)
        mvInitialized.push_back(pTracker);
    }

    bool bNewInit = mvInitialized.size() > mnLastNumInit;
    mnLastNumInit = mvInitialized.size();

    if (bNeedToDrop)  // Did any of the trackers want to add to map?
    {
      ROS_DEBUG("PoseCalibrator: Need to drop New MKF");

      if (!mpMap->mbGood)  // map has not been initialized yet with calibration pattern points
      {
        // this means that the first tracker signaled needToDrop
        ROS_DEBUG("PoseCalibrator: Need to initialize map");
        TrackerCalib* pFirstTracker = mmTrackers.begin()->second;
        std::string firstCamName = mmTrackers.begin()->first;

        // Call map maker's init from calib image
        bool bSuccess = mpMapMaker->InitFromCalibImage(*(pFirstTracker->mpCalibImage), mdSquareSize, firstCamName,
                                                       pFirstTracker->mpCurrentMKF->mse3BaseFromWorld);
        pFirstTracker->MarkKeyFrameAdded();

        if (!bSuccess)
        {
          // std::cout<<"Dumping cameras to cameras.dat"<<std::endl;
          // DumpCamerasToFile("cameras.dat");
          pFirstTracker->Reset(true);
        }
        else
        {
          mtLastMKF = ros::Time::now();
        }
      }
      else  // There is a good map
      {
        // Enough time elapsed since we last added an MKF?
        if (bNewInit || (ros::Time::now() - mtLastMKF > ros::Duration(1.0)))
        {
          ROS_DEBUG("PoseCalibrator: Gathering calibrated trackers");
          // Need to gather up all trackers that have been calibrated and assemble into a MKF
          MultiKeyFrame* pMKF = new MultiKeyFrame;
          pMKF->mdTotalDepthMean = 0;

          for (unsigned i = 0; i < mvInitialized.size(); ++i)
          {
            TrackerCalib* pTracker = mvInitialized[i];
            std::string camName = pTracker->mCamName;

            if (pTracker->GetTrackingQuality() == Tracker::GOOD)
            {
              // Take KeyFrame from TrackerCalib, put into MKF
              TransferKeyFrame(pMKF, pTracker);
              pMKF->mdTotalDepthMean += pMKF->mmpKeyFrames[camName]->mdSceneDepthMean;
            }
            else
            {
              // Tell tracker we added it just to clear need to drop flag, but don't actually add it
              pTracker->MarkKeyFrameAdded();
            }
          }

          pMKF->mdTotalDepthMean /= pMKF->mmpKeyFrames.size();
          mpMapMaker->AddMultiKeyFrame(pMKF);  // pMKF taken by MapMaker and is now NULL

          mtLastMKF = ros::Time::now();
        }
        else  // Otherwise we are too eager, don't add a new MKF yet
        {
          ROS_DEBUG("PoseCalibrator: Too eager, won't add new MKF yet");
          // All the locks were cleared when TrackFrame finished, but if we don't add the keyframes to the map,
          // the measurements that were recorded in them (and their backlinks) would stick around.
          // However, at the beginning of TrackerCalib's TrackFrame all measurements and backlinks are cleared, so we
          // don't have to do anything here

          // Still need to clear the need to drop flag otherwise TrackerCalib will think we didn't attend to its needs
          // and
          // throw an assertion

          for (unsigned i = 0; i < mvInitialized.size(); ++i)
            mvInitialized[i]->MarkKeyFrameAdded();
        }
      }

    }  // end if need to drop

    qTotalDurations.push_back(ros::Time::now() - start);
    if (qTotalDurations.size() > nMaxQueueSize)
      qTotalDurations.pop_front();

    qLoopTimes.push(ros::Time::now());
    if (qLoopTimes.size() > nMaxQueueSize)
      qLoopTimes.pop();

    PublishPoses();
  }  // end if got new frame

  if (bDrawKeyFrames)
  {
    // Clear screen with black
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    // Show the View menu
    mpKeyFrameViewer->Draw();
    captionStream << mpKeyFrameViewer->GetMessageForUser();
  }
  else
  {
    // Get a message from each tracker
    for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
    {
      captionStream << it->second->GetMessageForUser();
      captionStream << std::endl;
    }
  }

  if (qTotalDurations.size() == nMaxQueueSize)
  {
    double dAvg = AverageDuration(qTotalDurations);
    captionStream << std::endl
                  << "Average Tracking Duration " << dAvg;
  }

  if (qLoopTimes.size() == nMaxQueueSize)
  {
    ros::Duration dur(qLoopTimes.back() - qLoopTimes.front());
    double dFPS = (nMaxQueueSize - 1) / dur.toSec();
    captionStream << std::endl
                  << "FPS: " << dFPS;
  }

  return captionStream.str();
}

// Perform optimization of the relative poses of KeyFrames
std::string PoseCalibrator::Optimize()
{
  static tf::TransformBroadcaster br;
  std::stringstream captionStream;

  // Redraw the images each time because we need to wipe the screen
  // so that the GUI display can update properly
  for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
  {
    TrackerCalib& ct = *(it->second);
    std::string camName = it->first;
    glRasterPos(ct.mmDrawOffsets[camName]);
    glDrawPixels(ct.mpCurrentMKF->mmpKeyFrames[camName]->maLevels[0].image);
  }

  ROS_DEBUG("Chain bundle stepping...");

  double dSigmaSquared;
  double dMeanChiSquared;
  bool bSuccess = mpMapMaker->CalibOneStep();
  mmFinalPoses = mpMapMaker->mmFinalPoses;
  dSigmaSquared = mpMapMaker->mdSigmaSquared;
  dMeanChiSquared = mpMapMaker->mdMeanChiSquared;

  if (bSuccess)
  {
    for (SE3Map::iterator it = mmFinalPoses.begin(); it != mmFinalPoses.end(); it++)
    {
      captionStream << it->first << " Relative";
      TooN::Vector<3> v3Trans = it->second.get_translation();
      captionStream << " Translation: " << v3Trans;

      // Use Bullet rotation matrix to get roll, pitch, yaw angles instead of recoding my own
      Matrix<3> m3Rot = it->second.get_rotation().get_matrix();
      tf::Matrix3x3 rot(m3Rot(0, 0), m3Rot(0, 1), m3Rot(0, 2), m3Rot(1, 0), m3Rot(1, 1), m3Rot(1, 2), m3Rot(2, 0),
                        m3Rot(2, 1), m3Rot(2, 2));
      double r, p, y;
      rot.getRPY(r, p, y);

      captionStream << "  Rotation: " << r << "R " << p << "P " << y << "Y" << std::endl;

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(v3Trans[0], v3Trans[1], v3Trans[2]));
      transform.setBasis(rot);
      br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "base", it->first));
    }

    captionStream << "Mean weighted error (pixels): " << sqrt(dMeanChiSquared) << std::endl;
    captionStream << "Robust kernel sigma (pixels): " << sqrt(dSigmaSquared) << std::endl;
  }
  else
  {
    captionStream << "Bundle adjustment failed!" << std::endl;
  }

  return captionStream.str();
}

// Take the KeyFrame from a TrackerCalib and put it into a MultiKeyFrame
void PoseCalibrator::TransferKeyFrame(MultiKeyFrame* pMKF, TrackerCalib* pCT)
{
  std::string camName = pCT->mCamName;

  // They KeyFrame we're grabbing
  KeyFrame* pTransferKF = pCT->mpCurrentMKF->mmpKeyFrames[camName];

  pMKF->mmpKeyFrames[camName] = pTransferKF;     // transferred pointer
  pMKF->mmpKeyFrames[camName]->mpParent = pMKF;  // transferred KeyFrame's parent

  // Destroy tracker's mpCurrentMKF so it (and the appropriate KeyFrame) will be regenerated
  pCT->mpCurrentMKF->mmpKeyFrames.erase(
      camName);  // first remove our KF from list of keyframes so destructor of MKF won't try to delete KF
  delete pCT->mpCurrentMKF;
  pCT->mpCurrentMKF = NULL;

  if (pMKF->mmpKeyFrames.size() == 1)  // if first camera to be transfered
  {
    pMKF->mse3BaseFromWorld = pTransferKF->mse3CamFromWorld;  // set mkf pose to be pose from tracker
    pTransferKF->mse3CamFromBase = SE3<>();                   // identity transform
  }
  else
  {
    pTransferKF->mse3CamFromBase = pTransferKF->mse3CamFromWorld * pMKF->mse3BaseFromWorld.inverse();
  }

  pCT->MarkKeyFrameAdded(*pTransferKF);  // pass the kf back to allow tracker to regenerate
}

// Deals with user interface commands
void PoseCalibrator::GUICommandHandler(std::string command, std::string params)
{
  if (command == "exit" || command == "quit")
  {
    mbDone = true;
    return;
  }

  if (command == "ShowNextKeyFrame")
  {
    mpKeyFrameViewer->Next();
    return;
  }

  if (command == "ShowPrevKeyFrame")
  {
    mpKeyFrameViewer->Prev();
    return;
  }

  if (command == "Optimize")
  {
    mpMapMaker->PauseRun();
    if (!mpMapMaker->CalibInit())
    {
      mpMapMaker->ResumeRun();
      GUI.ParseLine("Menu.ShowMenu Root");  // kick it back to root menu
    }

    return;
  }

  if (command == "GrabMore")
  {
    mpMapMaker->ResumeRun();
    return;
  }

  if (command == "Reset")
  {
    mpMapMaker->RequestReset();

    for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
      it->second->Reset(false);

    return;
  }

  // KeyPress commands are issued by GLWindow
  if (command == "KeyPress")
  {
    if (params == "Space")
    {
      for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
        it->second->RequestInit(false);
    }
    else if (params == "r")
    {
      mpMapMaker->RequestReset();

      for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
        it->second->Reset(false);
    }
    else if (params == "q" || params == "Escape")
    {
      mbDone = true;
    }
    else
    {
      int nResetNum = atoi(params.c_str());

      // Only cameras #2 and greater can be reset individually. If you tried to
      // reset camera #1, it would destroy the fixed points extracted
      // from the checkerboard, and calibration would be impossible.

      if (nResetNum > 1)  // reset the corresponding tracker
      {
        TrackerCalibPtrMap::iterator it = std::next(mmTrackers.begin(), nResetNum - 1);
        TrackerCalib* pTracker = it->second;
        std::string camName = it->first;
        ROS_INFO_STREAM("Resetting " << camName << " tracker");
        pTracker->Reset(false);
        mpMapMaker->PauseRun();
        mpMapMaker->RemoveMultiKeyFrames(camName, false);
        mpMapMaker->ResumeRun();
      }
    }
    return;
  }

  if (command == "InitTracker")
  {
    for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
      it->second->RequestInit(false);

    return;
  }

  if (command == "SaveCalib")
  {
    mbDone = mpVideoSourceMulti->SavePoses(mmFinalPoses);

    // If we want to output poses for processing in another program do it now
    std::string poseFileName;
    if (mNodeHandlePriv.getParam("pose_out_file", poseFileName))
    {
      // Write kf poses to file
      std::ofstream file(poseFileName.c_str());
      if (!file.is_open())
      {
        ROS_ERROR_STREAM("Could not open " << poseFileName << " for writing poses");
      }
      else
      {
        ROS_INFO_STREAM("Writing poses to " << poseFileName);

        for (SE3Map::iterator it = mmFinalPoses.begin(); it != mmFinalPoses.end(); it++)
        {
          std::string camName = it->first;
          TooN::SE3<> se3Pose = (it->second).inverse();
          file << camName << std::endl;
          file << se3Pose << std::endl;
        }
        file.close();
      }
    }

    return;
  }

  ROS_FATAL_STREAM("System: Unhandled command in GUICommandHandler: " << command);
  ros::shutdown();
}

bool PoseCalibrator::ResetSystemCallback(mcptam::Reset::Request& request, mcptam::Reset::Response& response)
{
  ROS_INFO("PoseCalibrator: In reset callback, calling reset on individual trackers");
  for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
    it->second->Reset(false);

  return true;
}

void PoseCalibrator::PublishPoses()
{
  static tf::TransformBroadcaster br;

  for (TrackerCalibPtrMap::iterator it = mmTrackers.begin(); it != mmTrackers.end(); it++)
  {
    std::string camName = it->first;
    TrackerCalib* pTracker = it->second;
    geometry_msgs::Pose poseMsg;
    poseMsg = util::SE3ToPoseMsg(pTracker->GetCurrentPose().inverse());

    tf::Transform transform;
    tf::poseMsgToTF(poseMsg, transform);
    br.sendTransform(tf::StampedTransform(transform, pTracker->GetCurrentTimestamp(), "vision_world", camName));
  }
}
