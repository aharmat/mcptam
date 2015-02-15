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
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <sstream>

class Map;

class MapViewer
{
public:
  MapViewer(Map &map, GLWindow2 &glw);
  void DrawMap();
  std::string GetMessageForUser();
  
protected:
  Map &mMap;
  GLWindow2 &mGLWindow;
  
  void DrawGrid();
  void DrawMapDots();
  void DrawCamera(TooN::SE3<> se3, bool bSmall=false);
  void SetupFrustum();
  void SetupModelView(TooN::SE3<> se3WorldFromCurrent = TooN::SE3<>());
  
  TooN::SE3<> mse3ViewerFromWorld;

  std::ostringstream mMessageForUser;
};

#endif
