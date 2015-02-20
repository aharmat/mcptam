// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// MapViewer.h
//
// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the 
// current map and the camera/keyframe poses within it.
//
#ifndef __MAP_VIEWER_H
#define __MAP_VIEWER_H

#include <mcptam/GLWindow2.h>
#include <mcptam/Map.h>
#include <mcptam/EditAction.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <sstream>

class Map;

class MapViewer
{
public:
  MapViewer(Map &map, GLWindow2 &glw);
  void Init();
  void Draw();
  bool GUICommandHandler(std::string command, std::string params, std::shared_ptr<EditAction>& pAction); 
  std::string GetMessageForUser();
  
protected:
  
  void DrawGrid();
  void DrawMapDots(int nPointVis);
  void DrawCamera(TooN::SE3<> se3, bool bSmall=false);
  void SetupFrustum();
  void SetupModelView(TooN::SE3<> se3WorldFromCurrent = TooN::SE3<>());
  bool ProjectPoint(TooN::Vector<3> v3WorldPos, TooN::Vector<2>& v2Projected, double& dDistance, double& dPointRadius);
  
  void ToggleSelection(CVD::ImageRef irPixel);
  void SetSelectionInArea(CVD::ImageRef irBegin, CVD::ImageRef irEnd,  bool bSelected);
  
  std::vector<MapPoint*> GatherSelected();
  
  void ToggleAllPoints(int nPointVis);
  
  void DrawCrosshairs(CVD::ImageRef irPos, TooN::Vector<4> v4Color, float fLineWidth);
  void DrawRectangle(CVD::ImageRef irBegin, CVD::ImageRef irEnd, TooN::Vector<4> v4Color, float fLineWidth);
  
  void InitOrthoDrawing();
  
  void PutPointsOnLayer(int nLayer, bool bOnlySelected);
  
  Map &mMap;
  GLWindow2 &mGLWindow;
  
  TooN::SE3<> mse3ViewerFromWorld;
  TooN::SE3<> mse3RotCenterToViewer;
  TooN::SE3<> mse3WorldToRotCenter;
  
  double mdPanSensitivity;
  double mdZoomSensitivity;
  double mdRotSensitivity;
  
  double mdZNear;
  
  TooN::Matrix<4> mm4Projection;
  TooN::Matrix<2,4> mm24WindowConvert;
  
  float mfAttenuation[3];
  float mfOldAttenuation[3];
  float mfMinPointSize;
  float mfMaxPointSize;
  float mfDefaultPointSize;

  std::ostringstream mMessageForUser;
  
  enum SelectionMode{SINGLE, BOX_SELECT, BOX_UNSELECT} mSelectionMode;
  enum SelectionStatus{READY, SELECTING} mSelectionStatus;
  
  double mdSelectionThresh;
  CVD::ImageRef mirSelectionBegin;
  CVD::ImageRef mirSelectionCursor;
  
  bool mbMKey;
  
  double mdLastMaxViewerZ;
};

#endif
