#include <mcptam/MapViewer.h>
#include <mcptam/MapPoint.h>
#include <mcptam/KeyFrame.h>
#include <mcptam/LevelHelpers.h>
#include <mcptam/OpenGL.h>
#include <iomanip>
#include <cvd/gl_helpers.h>

MapViewer::MapViewer(Map &map, GLWindow2 &glw):
  mMap(map), mGLWindow(glw)
{
  mse3ViewerFromWorld = TooN::SE3<>();
}

void MapViewer::DrawMapDots()
{
  SetupFrustum();
  SetupModelView();
  
  //glColor3f(0,1,1);
  glEnable(GL_MULTISAMPLE);
  
  float maxSize = 0.0f;
  glGetFloatv( GL_POINT_SIZE_MAX, &maxSize );
  
  glPointParameterf(GL_POINT_SIZE_MIN, 2.0f);
  glPointParameterf(GL_POINT_SIZE_MAX, maxSize);
  //glPointParameterf(GL_POINT_FADE_THRESHOLD_SIZE, 10.0);
  
  float vals[3] = {0.0f, 0.0f, 0.005f};
  glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, vals);
  
  //glTexEnvf( GL_POINT_SPRITE, GL_COORD_REPLACE, GL_TRUE );
  //glEnable( GL_POINT_SPRITE );
  
  glBegin(GL_POINTS);
  
  for(MapPointPtrList::iterator point_it = mMap.mlpPoints.begin(); point_it != mMap.mlpPoints.end(); ++point_it)
  {
    MapPoint& point = *(*point_it);
    
    TooN::Vector<3> v3Pos = point.mv3WorldPos;
    CVD::glColor(gavLevelColors[point.mnSourceLevel]);
    CVD::glVertex(v3Pos);
  }
  glEnd();
  //glDisable( GL_POINT_SPRITE);
  glDisable(GL_MULTISAMPLE);
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

void MapViewer::DrawMap()
{
  mMessageForUser.str(""); // Wipe the user message clean
  
  // Update viewer position according to mouse input:
  std::pair<TooN::Vector<6>, TooN::Vector<6> > pv6 = mGLWindow.GetMousePoseUpdate();
  TooN::SE3<> se3Temp = mse3ViewerFromWorld.inverse() * TooN::SE3<>::exp(TooN::makeVector(pv6.first[4], -1*pv6.first[3], pv6.first[2],   //translation
                                                               -1*pv6.second[3], -1*pv6.second[4], pv6.first[5]));  //rotation
  mse3ViewerFromWorld = se3Temp.inverse();

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
  DrawMapDots();
  
  for(MultiKeyFramePtrList::iterator mkf_it = mMap.mlpMultiKeyFrames.begin(); mkf_it != mMap.mlpMultiKeyFrames.end(); ++mkf_it)
  {
    MultiKeyFrame& mkf = *(*mkf_it);
    DrawCamera(mkf.mse3BaseFromWorld, true);
  }
    
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  mMessageForUser << " Map: " << mMap.mlpPoints.size() << "P, " << mMap.mlpMultiKeyFrames.size() << "MKF";
}

std::string MapViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}

void MapViewer::SetupFrustum()
{
  glMatrixMode(GL_PROJECTION);  
  glLoadIdentity();
  double zNear = 0.03;
  glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,50);
  glScalef(1,1,-1);
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


