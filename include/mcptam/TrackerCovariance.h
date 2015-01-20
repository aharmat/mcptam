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
#include <unordered_set>

namespace g2o{
  template <typename MatrixType>
  class LinearSolverCholmodCustom;
  
  template <typename MatrixType>
  class SparseBlockMatrix;
}

class TrackerCovariance
{
public:
  TrackerCovariance();
  ~TrackerCovariance();
  
  TooN::Matrix<6> CalcCovarianceFull(TrackerDataPtrVector& vpAllMeas);
  TooN::Matrix<6> CalcCovarianceEstimate(TrackerDataPtrVector& vpTruncMeas, int nQueryPoint, std::string fileName=std::string());
  TooN::Matrix<6> CalcCovarianceEstimate(TrackerDataPtrVector& vpDenseMeas, TrackerDataPtrVector& vpSparseMeas, bool bFillNonexistant=false, std::string fileName=std::string());
  
  //debug
  void SavePointCovMatrix(std::string fileName, std::unordered_set<MapPoint*> spParticipatingPoints);
  
  static int snNumFitPoints;
  static int snNumEvalPerMeasNum;
  static int snNumRandomFits;
  static int snMinMeasNum;

protected:

  void CreateMatrices(TrackerDataPtrVector& vpMeas, g2o::SparseBlockMatrix<Eigen::Matrix2d>*& J1_Sigma_J1t, Eigen::MatrixXd*& J2);
  void CreateMatrices(TrackerDataPtrVector& vpDenseMeas, TrackerDataPtrVector& vpSparseMeas, std::vector<int> vDenseContent, 
                                        std::vector<g2o::SparseBlockMatrix<Eigen::Matrix2d>*>& vJ1_Sigma_J1t, Eigen::MatrixXd*& J2, bool bFillNonexistant);
                                        
  TooN::Vector<> PolyFit(TooN::Vector<> vX, TooN::Vector<> vY, int nDegree);

  g2o::LinearSolverCholmodCustom<Eigen::Matrix2d>* mpLinearSolver;
  
};

#endif

