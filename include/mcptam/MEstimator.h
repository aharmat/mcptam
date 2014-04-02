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
 * \file MEstimator.h
 * \brief Declaration of MEstimator structures
 *
 * This code is from the original PTAM, which is
 * Copyright 2008 Isis Innovation Limited
 *
 * Defines various MEstimators which can be used by the Tracker and
 * the Bundle adjuster. Not that some of the inputs are square
 * quantities!
 *
 ****************************************************************************************/

#ifndef __M_ESTIMATOR_H
#define __M_ESTIMATOR_H

#include <vector>
#include <algorithm>
#include <cassert>

struct Tukey
{
  inline static double FindSigmaSquared(std::vector<double> &vErrorSquared, bool bPrint=false);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct Cauchy
{
  inline static double FindSigmaSquared(std::vector<double> &vErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct Huber
{
  inline static double FindSigmaSquared(std::vector<double> &vErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};

struct LeastSquares
{
  inline static double FindSigmaSquared(std::vector<double> &vErrorSquared);
  inline static double SquareRootWeight(double dErrorSquared, double dSigmaSquared);
  inline static double Weight(double dErrorSquared, double dSigmaSquared);
  inline static double ObjectiveScore(double dErrorSquared, double dSigmaSquared);
};


inline double Tukey::Weight(double dErrorSquared, double dSigmaSquared)
{
  double dSqrt = SquareRootWeight(dErrorSquared, dSigmaSquared);
  return dSqrt * dSqrt;
}

inline double Tukey::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  if(dErrorSquared > dSigmaSquared)
    return 0.0;
  else
    return 1.0 - (dErrorSquared / dSigmaSquared);
}

inline double Tukey::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  // NB All returned are scaled because
  // I'm not multiplying by sigmasquared/6.0
  if(dErrorSquared > dSigmaSquared)
    return 1.0;
  double d = 1.0 - dErrorSquared / dSigmaSquared;
  return (1.0 - d*d*d);
}


inline double Tukey::FindSigmaSquared(std::vector<double> &vErrorSquared, bool bPrint)
{ 
  double dSigmaSquared; 
  assert(vErrorSquared.size() > 0);
  std::sort(vErrorSquared.begin(), vErrorSquared.end());
  double dMedianSquared = vErrorSquared[vErrorSquared.size() / 2];
  if(bPrint)
  {
    ROS_INFO_STREAM("Median Squared: "<<dMedianSquared<<" sqrt: "<<sqrt(dMedianSquared));
    ROS_INFO_STREAM("Errors:");
    for(unsigned i=0; i < vErrorSquared.size(); ++i)
      ROS_INFO_STREAM(vErrorSquared[i]);
  }
  double dSigma = 1.4826 * (1 + 5.0 / (vErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  4.6851 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}


///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double Cauchy::Weight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0 / (1.0 + dErrorSquared / dSigmaSquared);
}

inline double Cauchy::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

inline double Cauchy::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  return log(1.0 + dErrorSquared / dSigmaSquared);
}


inline double Cauchy::FindSigmaSquared(std::vector<double> &vErrorSquared)
{ 
  double dSigmaSquared; 
  assert(vErrorSquared.size() > 0);
  std::sort(vErrorSquared.begin(), vErrorSquared.end());
  double dMedianSquared = vErrorSquared[vErrorSquared.size() / 2];
  double dSigma = 1.4826 * (1 + 5.0 / (vErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  4.6851 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}


///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double Huber::Weight(double dErrorSquared, double dSigmaSquared)
{
  if(dErrorSquared < dSigmaSquared)
    return 1;
  else
    return sqrt(dSigmaSquared / dErrorSquared);
}

inline double Huber::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return sqrt(Weight(dErrorSquared, dSigmaSquared));
}

inline double Huber::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  if(dErrorSquared< dSigmaSquared)
    return 0.5 * dErrorSquared;
  else
    {
      double dSigma = sqrt(dSigmaSquared);
      double dError = sqrt(dErrorSquared);
      return dSigma * ( dError - 0.5 * dSigma);
    }
}


inline double Huber::FindSigmaSquared(std::vector<double> &vErrorSquared)
{ 
  double dSigmaSquared; 
  assert(vErrorSquared.size() > 0);
  std::sort(vErrorSquared.begin(), vErrorSquared.end());
  double dMedianSquared = vErrorSquared[vErrorSquared.size() / 2];
  double dSigma = 1.4826 * (1 + 5.0 / (vErrorSquared.size() * 2 - 6)) * sqrt(dMedianSquared);
  dSigma =  1.345 * dSigma;
  dSigmaSquared = dSigma * dSigma;
  return dSigmaSquared;
}

///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////

inline double LeastSquares::Weight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0;
}

inline double LeastSquares::SquareRootWeight(double dErrorSquared, double dSigmaSquared)
{
  return 1.0;
}

inline double LeastSquares::ObjectiveScore(double dErrorSquared, const double dSigmaSquared)
{
  return dErrorSquared;
}


inline double LeastSquares::FindSigmaSquared(std::vector<double> &vErrorSquared)
{ 
  if(vErrorSquared.size() == 0)
    return 0.0;
  double dSum = 0.0;
  for(unsigned int i=0; i<vErrorSquared.size(); i++)
    dSum+=vErrorSquared[i];
  return dSum / vErrorSquared.size();
}

#endif

