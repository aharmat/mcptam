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

#include <mcptam/CameraGroupSubscriber.h>

// Static members
bool CameraGroupSubscriber::sbDynamicSync = false;
std::string CameraGroupSubscriber::sImageTopic = "image_raw";
std::string CameraGroupSubscriber::sInfoTopic = "camera_info";
std::string CameraGroupSubscriber::sPoseTopic = "pose";
std::string CameraGroupSubscriber::sCameraPrefix = "";

CameraGroupSubscriber::CameraGroupSubscriber(std::vector<std::string> vCameraNames, bool bGetPoseSeparately)
: mNodeHandlePriv("~")
, mpSync(new message_filters::Synchronizer<ApproxTimePolicy>(ApproxTimePolicy(5)))  // queue size of 5
, mLastTimestamp(0,0)
{
  // Make sure we're using our own callback queue
  mNodeHandle.setCallbackQueue(&mCallbackQueue);
  mpImageTransport = new image_transport::ImageTransport(mNodeHandle);
  
  mvCameraNames = vCameraNames;
  mNumCams = mvCameraNames.size();
  
  ROS_INFO_STREAM("CameraGroupSubscriber: Loading "<<mNumCams<<" cameras");
  
  // Maximum of 8 cameras in a group. Unlikely to have more since we'll have
  // bandwidth issues with all those images coming in together. Also, need
  // to specifiy the synchronizer policy at compile time so we can't use
  // the runtime variable mNumCams
  
  if(mNumCams < 1)
  {
    ROS_FATAL("CameraGroupSubscriber: No topics given, what are you doing?");
    ros::shutdown();
    return;
  }
  else if(mNumCams > 8)
  {
    ROS_FATAL_STREAM("CameraGroupSubscriber: Maximum 8 cameras in a group, you specified "<<mNumCams);
    ros::shutdown();
    return;
  }
  
  // The images from the cameras in this group should arrive at the same time, but often
  // there are slight delays leading to differing timestamps. Therefore, use the approximate
  // time policy to synchronize them.
  // [image subscribers] --> [synchronizer] --> [callback function]
  
  // Setting the policy lower bound helps the approximate time synchronizer perform better (see ros doc)
  // However, it's a pain to get a good estimated framerate in different circumstances (ie live capture, 
  // replaying from bag file, etc.) so don't do this for now
  
  /*
  ApproxTimePolicy* policy = dynamic_cast<ApproxTimePolicy*>( mpSync->getPolicy() );
  for(unsigned i=0; i < 8; ++i)
    policy->setInterMessageLowerBound(i, ros::Duration((1.0/8.0)));
  */
  
  // Create space for our image subscribers
  mvpImageSubs.resize(mNumCams);
  for(unsigned i=0; i < mNumCams; ++i)
  {
    // Subscribe to the appropriate image topic
    mvpImageSubs[i] = new image_transport::SubscriberFilter();
    mvpImageSubs[i]->subscribe(*mpImageTransport, std::string(sCameraPrefix + "/" + mvCameraNames[i] + "/" + sImageTopic), 2); 
  }
  
  // Connect the synchronizer to the subscribers
  // If there are fewer than 8 cameras, the unfilled slots are connected
  // back to the first subscriber (no performance hit)
  switch(mNumCams)
  {
    case 1:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],
                         *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 2:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[0],*mvpImageSubs[0],
                         *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 3:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[0],
                         *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 4:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
                         *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 5:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
                         *mvpImageSubs[4],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 6:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
                         *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[0],*mvpImageSubs[0]);
      break;
    }
    case 7:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
                         *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[6],*mvpImageSubs[0]);
      break;
    }
    case 8:
    {
      mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
                         *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[6],*mvpImageSubs[7]);
      break;
    }
    default:
    {
      ROS_ERROR("CameraGroupSubscriber: Couldn't resolve mpSync connectInput switch block, should never happen");
      return;
    }
  }
  
  // Connect the callback to the synchronizer
  mpSync->registerCallback(&CameraGroupSubscriber::ImageCallback, this);
  
  // Subscribe to the camera info and pose topics
  mvInfoSubs.resize(mNumCams);
  
  if(bGetPoseSeparately)
    mvPoseSubs.resize(mNumCams);
  
  for(unsigned i=0; i < mNumCams; ++i)
  {
    mvInfoSubs[i] = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(std::string(sCameraPrefix + "/" + mvCameraNames[i] + "/" + sInfoTopic), 1, boost::bind(&CameraGroupSubscriber::InfoCallback, this, _1, mvCameraNames[i])); 
    
    if(bGetPoseSeparately)
      mvPoseSubs[i] = mNodeHandle.subscribe<geometry_msgs::Pose>(std::string(sCameraPrefix + "/" + mvCameraNames[i] + "/" + sPoseTopic), 1, boost::bind(&CameraGroupSubscriber::PoseCallback, this, _1, mvCameraNames[i])); 
  }
  
  ROS_INFO_STREAM("CameraGroupSubscriber: Waiting for camera info" << (bGetPoseSeparately ? " and separate pose" : "") << " from all cameras");
  
  while(mmSavedInfos.size() < mNumCams && ros::ok())  // loop while we haven't collected all the info yet
  {
    mCallbackQueue.callAvailable();
    ros::WallDuration(0.2).sleep();
  }
  
  for(unsigned i=0; i < mNumCams; ++i)
    mvInfoSubs[i].shutdown();  // stop getting messages
  
  while(bGetPoseSeparately && mmSavedPoses.size() < mNumCams && ros::ok())
  {
    mCallbackQueue.callAvailable();
    ros::WallDuration(0.2).sleep();
  }
  
  if(bGetPoseSeparately)
  {
    for(unsigned i=0; i < mNumCams; ++i)
      mvPoseSubs[i].shutdown();  // stop getting messages
  }
  
  ROS_INFO("CameraGroupSubscriber: Got all info");
  
  // At this point we have mmSavedInfos and mmSavedPoses filled, 
  // and all of mvInfoSubs and mvPoseSubs have been shutdown so 
  // we won't get any more messages from them
}

CameraGroupSubscriber::~CameraGroupSubscriber()
{
  // Need to delete this first because it will try to disconnect from the
  // mvpImageSubs in the destructor
  delete mpSync;
  
  for(unsigned i=0; i < mNumCams; ++i)
  {
    mvpImageSubs[i]->unsubscribe();
    delete mvpImageSubs[i];
  }
    
  mvpImageSubs.clear();
  mvInfoSubs.clear();
  mvPoseSubs.clear();
  
  delete mpImageTransport;
}

// This will be called by the ROS subscriber whenever a CameraInfo message arrives from any of the cameras
void CameraGroupSubscriber::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr& infoMsg, std::string cameraName)
{
  ROS_DEBUG("CameraGroupSubscriber: in InfoCallback");
  // Save to the appropriate slot
  mmSavedInfos[cameraName] = *infoMsg;
}

// This will be called by the ROS subscriber whenever a Pose message arrives from any of the cameras
void CameraGroupSubscriber::PoseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg, std::string cameraName)
{
  ROS_DEBUG("CameraGroupSubscriber: in PoseCallback");
  // Save to the appropriate slot
  mmSavedPoses[cameraName] = *poseMsg;
}

// This will be called by the ROS subscriber whenever images from all the cameras are received
void CameraGroupSubscriber::ImageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::ImageConstPtr& msg3, const sensor_msgs::ImageConstPtr& msg4,
                                          const sensor_msgs::ImageConstPtr& msg5, const sensor_msgs::ImageConstPtr& msg6, const sensor_msgs::ImageConstPtr& msg7, const sensor_msgs::ImageConstPtr& msg8)
{
  // Remeber that if the number of cameras is less than 8, the messages in the unused slots will point back to the first image message
  try
  {
    if(mNumCams > 0)
      mmLastImages[mvCameraNames[0]] = cv_bridge::toCvShare(msg1);
   
    if(mNumCams > 1)
      mmLastImages[mvCameraNames[1]] = cv_bridge::toCvShare(msg2);
      
    if(mNumCams > 2)
      mmLastImages[mvCameraNames[2]] = cv_bridge::toCvShare(msg3);
      
    if(mNumCams > 3)
      mmLastImages[mvCameraNames[3]] = cv_bridge::toCvShare(msg4);
      
    if(mNumCams > 4)
      mmLastImages[mvCameraNames[4]] = cv_bridge::toCvShare(msg5);
      
    if(mNumCams > 5)
      mmLastImages[mvCameraNames[5]] = cv_bridge::toCvShare(msg6);
      
    if(mNumCams > 6)
      mmLastImages[mvCameraNames[6]] = cv_bridge::toCvShare(msg7);
      
    if(mNumCams > 7)
      mmLastImages[mvCameraNames[7]] = cv_bridge::toCvShare(msg8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CameraGroupSubscriber: cv_bridge exception: %s", e.what());
    return;
  }
  
  ros::Time currTimestamp;
  
  if(mmLastImages.size() == 1)  // want exact stamp, converting with toSec and then back again loses some precision
  {
    currTimestamp = mmLastImages.begin()->second->header.stamp;
  }
  else
  {
    double dStamp = 0;
    
    for(ImagePtrMap::iterator it = mmLastImages.begin(); it != mmLastImages.end(); it++)
    {
      dStamp += it->second->header.stamp.toSec();
    }
    
    currTimestamp = ros::Time(dStamp / mmLastImages.size());
  }
  
  if(sbDynamicSync && !mLastTimestamp.isZero() && (currTimestamp - mLastTimestamp).toSec() > 0)
  {
    ApproxTimePolicy* policy = dynamic_cast<ApproxTimePolicy*>( mpSync->getPolicy() );
    for(unsigned i=0; i < 8; ++i)
      policy->setInterMessageLowerBound(i, (currTimestamp - mLastTimestamp) * 0.8);
  }
  
  mLastTimestamp = currTimestamp;
}

// Grab a new set of images from all the cameras in the group
ImagePtrMap CameraGroupSubscriber::GetNewImage(bool* bActive)
{
  mmLastImages.clear();
  
  ros::WallRate rate(200);
  
  // Wait until mmLastImages has been filled and we don't want to bail
  while(mmLastImages.size() < mNumCams && ros::ok() && *bActive)
  {
    mCallbackQueue.callAvailable();   // This will trigger any waiting callbacks
    rate.sleep();
  }
  
  if(mmLastImages.size() < mNumCams)
    return ImagePtrMap(); // aborted early, so return empty map
  else // otherwise return good set of images
    return mmLastImages;  
}
