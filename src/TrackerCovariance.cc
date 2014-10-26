#include <mcptam/TrackerCovariance.h>
#include <mcptam/TrackerData.h>
#include <mcptam/linear_solver_cholmod_custom.h>
#include <unordered_set>

//#include <Eigen/Core>
//using Eigen::Map;

#include <g2o/core/sparse_block_matrix.h>

TrackerCovariance::TrackerCovariance()
{
  mpLinearSolver = new g2o::LinearSolverCholmodCustom<Eigen::Matrix2d>(6);
}

TrackerCovariance::~TrackerCovariance()
{
  delete mpLinearSolver;
}

TooN::Matrix<6> TrackerCovariance::CalcCovariance(TrackerDataPtrVector& vpAllMeas)
{
  int nMeasNum = vpAllMeas.size();
  int* blockIndices = new int[nMeasNum];
  
  ros::Time buildStart = ros::Time::now();
  std::cerr<<"About to build cross cov system with "<<nMeasNum<<" measurements"<<std::endl;
  for(int i=0; i < nMeasNum; ++i)
  {
    vpAllMeas[i]->mnBlockIdx = i;
    blockIndices[i] = (i+1)*2;
  }
  
  g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t = new g2o::SparseBlockMatrix<Eigen::Matrix2d>(blockIndices, blockIndices, nMeasNum, nMeasNum);
  delete[] blockIndices;
  
  Eigen::MatrixXd J2(nMeasNum*2, 6);
  Eigen::MatrixXd Zinv_J2(nMeasNum*2, 6);  // Z = J1 * Sigma * J1^T
  
  TooN::Matrix<3> m3TempCov;
  for(unsigned i=0; i < vpAllMeas.size(); ++i)
  {
    TrackerData &td = *vpAllMeas[i];
    
    // Diagonal element
    Eigen::Matrix2d m2PixelCov = Eigen::Matrix2d::Identity() * (1.0/td.mdSqrtInvNoise);
    Eigen::Matrix2d* pm2Diag = J1_Sigma_J1t->block(td.mnBlockIdx, td.mnBlockIdx, true);
    
    // NOTE: Eigen's default ordering is column major. TooN's is row major. I don't know if any of 
    // g2o's internals assume column major ordering when working with Eigen matrices, so I don't want
    // to set that. 
    //TooN::Matrix<2,2,double> m2DiagWrapped = TooN::wrapMatrix<2,2,double>(m2Diag->data());
    
    
    //TooN::Matrix<2, 2, double, TooN::ColMajor> m2DiagWrapped(m2Diag->data());
    //m2DiagWrapped = td.mm23Jacobian * td.mPoint.mm3WorldCov * td.mm23Jacobian.T() + m2PixelCov;
    
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> m23Jac = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(td.mm23Jacobian.get_data_ptr());
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m3WorldCov = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(td.mPoint.mm3WorldCov.get_data_ptr());
    
    *pm2Diag = m23Jac * m3WorldCov * m23Jac.transpose() + m2PixelCov;
    
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> m26Jac = Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >(td.mm26Jacobian.get_data_ptr());
    J2.block(i*2, 0, 2, 6) = m26Jac;
    
    // Off diagonal elements
    for(unsigned j=i+1; j < vpAllMeas.size(); ++j)
    {
      TrackerData& td_other = *vpAllMeas[j];
    
      // They refer to the same point
      if(&td.mPoint == &td_other.mPoint)
      {
        m3TempCov = td.mPoint.mm3WorldCov;
      }
      else  // They refer to different points
      {
        // Cross cov with other 
        if(!td.mPoint.CrossCov(&td_other.mPoint, m3TempCov))
          continue;
      }
      
      Eigen::Matrix2d* pm2OffDiag = J1_Sigma_J1t->block(td.mnBlockIdx, td_other.mnBlockIdx, true);
      //TooN::Matrix<2, 2, double, TooN::ColMajor> m2OffDiagWrapped(m2OffDiag->data());
      //m2OffDiagWrapped = td.mm23Jacobian * m3CrossCov * td_other.mm23Jacobian.T();
      
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m3CrossCov = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(m3TempCov.get_data_ptr());
      Eigen::Matrix<double, 2, 3, Eigen::RowMajor> m23OtherJac = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(td_other.mm23Jacobian.get_data_ptr());
    
      *pm2OffDiag = m23Jac * m3CrossCov * m23OtherJac.transpose();
    }
  }
  
  std::cerr<<"Built cross cov system in "<<ros::Time::now() - buildStart<<" seconds"<<std::endl;
  
  ros::Time solveStart = ros::Time::now();
  std::cerr<<"About to solve cross cov system"<<std::endl;
  mpLinearSolver->init();
  mpLinearSolver->solve(*J1_Sigma_J1t, Zinv_J2.data(), J2.data());
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result = J2.transpose() * Zinv_J2; 
  std::cerr<<"Solved cross cov system in "<<ros::Time::now() - solveStart<<" seconds"<<std::endl;
  
  TooN::Matrix<6,6,double> m6Cov = TooN::wrapMatrix<6,6,double>(result.data());
  
  
  //TooN::Matrix<6> m6Cov = TooN::Zeros;
  delete J1_Sigma_J1t; 
  return m6Cov;
}
