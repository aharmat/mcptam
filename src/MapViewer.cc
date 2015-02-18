#include <mcptam/MapViewer.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/OpenGL.h>
#include <iomanip>
#include <cvd/gl_helpers.h>

MapViewer::MapViewer(Map &map, GLWindow2 &glw)
: mMap(map)
, mGLWindow(glw)
, mfAttenuation{0.0f, 0.0f, 0.005f}
{
  mse3WorldToRotCenter = TooN::SE3<>();
  mse3RotCenterToViewer = TooN::SE3<>::exp(TooN::makeVector(0,0,-5,0,0,0));
  
  std::cout<<"Initial rot center translation: "<<mse3RotCenterToViewer.get_translation()<<std::endl;
  
  mse3ViewerFromWorld = (mse3WorldToRotCenter * mse3RotCenterToViewer).inverse();
  
  mdPanSensitivity = 0.005;
  mdZoomSensitivity = 0.1;
  mdRotSensitivity = 0.01;
  
  mdZNear = 0.03;
  
  mfDefaultPointSize = 1;
  mfMinPointSize = 2;
  glGetFloatv( GL_POINT_SIZE_MAX, &mfMaxPointSize);
}

void MapViewer::DrawMapDots(int nPointVis)
{
  SetupFrustum();
  SetupModelView();
  
  //glColor3f(0,1,1);
  //glEnable(GL_MULTISAMPLE);
  
  glPointSize(mfDefaultPointSize);
  glPointParameterf(GL_POINT_SIZE_MIN, mfMinPointSize);
  glPointParameterf(GL_POINT_SIZE_MAX, mfMaxPointSize);
  //glPointParameterf(GL_POINT_FADE_THRESHOLD_SIZE, 10.0);
  
  glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, mfAttenuation);
  
  //glTexEnvf( GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE );
  //glEnable( GL_POINT_SPRITE );
  
  glBegin(GL_POINTS);
  
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    if(point.mbDeleted)
      continue;
      
    if(!(point.mnUsing & nPointVis))  // bitfield AND to determine point visibility
      continue;
    
    TooN::Vector<3> v3Pos = point.mv3WorldPos;
    TooN::Vector<4> v4Color;
    /*
    v4Color.slice<0,3>() = gavLevelColors[point.mnSourceLevel];
    
    if(point_it == mMap.mlpPoints.begin())
      v4Color[3] = 1;
    else
      v4Color[3] = 0.4;
    */
    
    if(point.mbSelected)
    {
      v4Color = TooN::makeVector(1,1,0,1);
    }
    else
    {
      v4Color = TooN::makeVector(1,0,1,1);
    }
    
    
    CVD::glColor(v4Color);
    CVD::glVertex(v3Pos);
  }
  glEnd();
  //glDisable( GL_POINT_SPRITE);
  //glDisable(GL_MULTISAMPLE);
}

void MapViewer::DrawGrid()
{
  SetupFrustum();
  SetupModelView();
  glLineWidth(1);
  
  glBegin(GL_LINES);
  
  // Draw a larger grid around the outside..
  double dGridInterval = 0.1;
  
  double dMin = -100.0 * dGridInterval;
  double dMax =  100.0 * dGridInterval;
  
  for(int x=-10;x<=10;x+=1)
  {
    if(x==0)
      glColor3f(1,1,1);
    else
      glColor3f(0.3,0.3,0.3);
      
    glVertex3d((double)x * 10 * dGridInterval, dMin, 0.0);
    glVertex3d((double)x * 10 * dGridInterval, dMax, 0.0);
  }
  
  for(int y=-10;y<=10;y+=1)
  {
    if(y==0)
      glColor3f(1,1,1);
    else
      glColor3f(0.3,0.3,0.3);
      
    glVertex3d(dMin, (double)y * 10 *  dGridInterval, 0.0);
    glVertex3d(dMax, (double)y * 10 * dGridInterval, 0.0);
  }
  
  glEnd();

  glBegin(GL_LINES);
  dMin = -10.0 * dGridInterval;
  dMax =  10.0 * dGridInterval;
  
  for(int x=-10;x<=10;x++)
  {
    if(x==0)
      glColor3f(1,1,1);
    else
      glColor3f(0.5,0.5,0.5);
    
    glVertex3d((double)x * dGridInterval, dMin, 0.0);
    glVertex3d((double)x * dGridInterval, dMax, 0.0);
  }
  
  for(int y=-10;y<=10;y++)
  {
    if(y==0)
      glColor3f(1,1,1);
    else
      glColor3f(0.5,0.5,0.5);
      
    glVertex3d(dMin, (double)y * dGridInterval, 0.0);
    glVertex3d(dMax, (double)y * dGridInterval, 0.0);
  }
  
  glColor3f(1,0,0);
  glVertex3d(0,0,0);
  glVertex3d(1,0,0);
  glColor3f(0,1,0);
  glVertex3d(0,0,0);
  glVertex3d(0,1,0);
  glColor3f(1,1,1);
  glVertex3d(0,0,0);
  glVertex3d(0,0,1);
  glEnd();
  
//   glColor3f(0.8,0.8,0.8);
//   glRasterPos3f(1.1,0,0);
//   mGLWindow.PrintString("x");
//   glRasterPos3f(0,1.1,0);
//   mGLWindow.PrintString("y");
//   glRasterPos3f(0,0,1.1);
//   mGLWindow.PrintString("z");
}

void MapViewer::DrawMap(int nPointVis)
{
  mMessageForUser.str(""); // Wipe the user message clean
  
  // Update viewer position according to mouse input:
  MouseUpdate update = mGLWindow.GetMouseUpdate();
  
  // Pan: shift + middle mouse
  // Rotate: middle mouse
  // Zoom: mouse wheel
  
  // Pan moves the rotation center parallel to viewer image
  // First, transform right click vector into world frame
  TooN::Vector<3> v3PanInput = TooN::Zeros;
  v3PanInput.slice<0,2>() = update.mv2ShiftMiddleClick;
  TooN::Vector<3> v3PanInputInWorld = mse3ViewerFromWorld.inverse().get_rotation() * v3PanInput;
  // Then apply update
  double dDistFromRotCenter = TooN::norm(mse3RotCenterToViewer.get_translation());
  TooN::Vector<6> v6RotCenterUpdate = TooN::Zeros;
  v6RotCenterUpdate.slice<0,3>() = v3PanInputInWorld * dDistFromRotCenter * mdPanSensitivity * -1;
  mse3WorldToRotCenter = TooN::SE3<>::exp(v6RotCenterUpdate) * mse3WorldToRotCenter;
  
  // Zoom moves camera relative to rotation center
  // Want to move more when distance is already far, and vice versa
  double dDistChangeFactor = (1 + update.mdWheel*mdZoomSensitivity);
  mse3RotCenterToViewer.get_translation() *= dDistChangeFactor;
  
  // Rotate moves camera relative to rotation center
  // First, compute axis of rotation as the cross between the image normal and the middle click vector
  TooN::Vector<3> v3RotInput = TooN::Zeros;
  v3RotInput.slice<0,2>() = update.mv2MiddleClick;
  TooN::Vector<3> v3RotAxis = TooN::makeVector(0,0,1) ^ v3RotInput;
  // Now transform rotation vector into world frame
  TooN::Vector<3> v3RotAxisInWorld = mse3ViewerFromWorld.inverse().get_rotation() * v3RotAxis;
  // Then apply update
  v6RotCenterUpdate = TooN::Zeros;
  v6RotCenterUpdate.slice<3,3>() = v3RotAxisInWorld * mdRotSensitivity;
  mse3WorldToRotCenter = TooN::SE3<>::exp(v6RotCenterUpdate) * mse3WorldToRotCenter;
  
  // Finally update the the pose we really want
  mse3ViewerFromWorld = (mse3WorldToRotCenter * mse3RotCenterToViewer).inverse();

  mGLWindow.SetupViewport();
  glClearColor(0,0,0,0);
  glClearDepth(1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);

  glEnable(GL_DEPTH_TEST);
  DrawGrid();
  DrawMapDots(nPointVis);
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    DrawCamera(mkf.mse3BaseFromWorld, true);
  }
    
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  mMessageForUser << "Map: " << mMap.mlpPoints.size() << "P, " << mMap.mlpMultiKeyFrames.size() << "MKF" << std::endl;
  //mMessageForUser << "mse3WorldToRotCenter trans: "<<mse3WorldToRotCenter.get_translation() << std::endl;
  //mMessageForUser << "mse3RotCenterToViewer trans: "<<mse3RotCenterToViewer.get_translation() << std::endl;
  //mMessageForUser << "mse3ViewerFromWorld trans: "<<mse3ViewerFromWorld.get_translation() ;
}

std::string MapViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}

void MapViewer::SetupFrustum()
{
  glMatrixMode(GL_PROJECTION);  
  glLoadIdentity();
  
  double dRatio = (double)mGLWindow.size().y / mGLWindow.size().x;
  
  double dLeft = -mdZNear;
  double dRight = mdZNear;
  double dBottom = dRatio*mdZNear;
  double dTop = -dRatio*mdZNear;
  double dNear = mdZNear;
  double dFar = 50;
  
  glFrustum(dLeft, dRight, dBottom, dTop, dNear, dFar);
  glScalef(1,1,-1);
  
  mm4Projection = TooN::Zeros;
  mm4Projection(0,0) = 2*dNear/(dRight-dLeft);
  mm4Projection(0,2) = (dRight+dLeft)/(dRight-dLeft);
  mm4Projection(1,1) = 2*dNear/(dTop-dBottom);
  mm4Projection(1,2) = (dTop+dBottom)/(dTop-dBottom);
  mm4Projection(2,2) = (dNear+dFar)/(dNear-dFar); 
  mm4Projection(2,3) = 2*dNear*dFar/(dNear-dFar);
  mm4Projection(3,2) = -1.0;
  
  TooN::Matrix<4> m4Scale = TooN::Identity;
  m4Scale(2,2) = -1;
  
  mm4Projection = mm4Projection * m4Scale;
  
  mm24WindowConvert = TooN::Zeros;
  mm24WindowConvert(0,0) = mGLWindow.size().x * 0.5;
  mm24WindowConvert(1,1) = mGLWindow.size().y * 0.5;
  mm24WindowConvert(0,3) = mGLWindow.size().x * 0.5;
  mm24WindowConvert(1,3) = mGLWindow.size().y * 0.5;

  return;
};

void MapViewer::SetupModelView(TooN::SE3<> se3WorldFromCurrent)
{
  glMatrixMode(GL_MODELVIEW);  
  glLoadIdentity();
  CVD::glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
  return;
};

void MapViewer::DrawCamera(TooN::SE3<> se3CfromW, bool bSmall)
{
  SetupModelView(se3CfromW.inverse());
  SetupFrustum();
  
  if(bSmall)
    glLineWidth(1);
  else
    glLineWidth(3);
  
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.1f, 0.0f, 0.0f);
  glColor3f(0,1,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.1f, 0.0f);
  glColor3f(1,1,1);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.1f);
  glEnd();

  
  if(!bSmall)
  {
	  glLineWidth(1);
	  glColor3f(0.5,0.5,0.5);
	  SetupModelView();
	  TooN::Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
	  glBegin(GL_LINES);
	  glColor3f(1,1,1);
	  glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
	  glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
  	  glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
	  glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
	  glEnd();
  }
  
}

bool MapViewer::ProjectPoint(TooN::Vector<3> v3WorldPos, TooN::Vector<2>& v2Projected, double& dDistance, double& dPointRadius)
{
  TooN::Vector<3> v3CamPos = mse3ViewerFromWorld * v3WorldPos;
  TooN::Vector<4> v4Clip = mm4Projection * TooN::unproject(v3CamPos);
  
  if(v4Clip[0] < -v4Clip[3] || v4Clip[0] > v4Clip[3])
    return false;
    
  if(v4Clip[1] < -v4Clip[3] || v4Clip[1] > v4Clip[3])
    return false;
    
  if(v4Clip[2] < -v4Clip[3] || v4Clip[2] > v4Clip[3])
    return false;
    
  TooN::Vector<3> v3NDC = TooN::project(v4Clip);
  v3NDC[1] *= -1;  // flip y coordinate from OpenGL to computer vision convention
  
  v2Projected = mm24WindowConvert * TooN::unproject(v3NDC);
  
  float a = mfAttenuation[0];
  float b = mfAttenuation[1];
  float c = mfAttenuation[2];
  float d = TooN::norm(v3CamPos);
  
  float fPointWidth = mfDefaultPointSize * sqrt(1/(a + b*d + c*d*d));
  fPointWidth = std::max(mfMinPointSize, std::min(fPointWidth, mfMaxPointSize));
  
  dPointRadius = fPointWidth/2;
  dDistance = d;
  
  return true;
}

