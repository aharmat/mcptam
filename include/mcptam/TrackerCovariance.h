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
 * \file TrackerCovariance.h
 * \brief Functions for computing Tracker covariance
 *
 ****************************************************************************************/

#ifndef __TRACKER_COVARIANCE_H
#define __TRACKER_COVARIANCE_H

#include <mcptam/Types.h>
#include <Eigen/Core>

namespace g2o{
  template <typename MatrixType>
  class LinearSolverCholmodCustom;
}

class TrackerCovariance
{
public:
  TrackerCovariance( std::string fileName = "", std::vector<int> vAnalysisMeasNum = std::vector<int>(), int nNumPredPoints = 2);
  ~TrackerCovariance();
  
  TooN::Matrix<6> CalcCovariance(TrackerDataPtrVector& vpAllMeas, bool bDoAnalysis);

protected:

  TooN::Vector<> PolyFit(TooN::Vector<> vX, TooN::Vector<> vY, int nDegree);

  g2o::LinearSolverCholmodCustom<Eigen::Matrix2d>* mpLinearSolver;
  std::string mFileName;
  std::vector<int> mvAnalysisMeasNum;
  int mnNumPredPoints;
};

#endif

