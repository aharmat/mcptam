#include <mcptam/TrackerCovariance.h>
#include <mcptam/TrackerData.h>
#include <mcptam/linear_solver_cholmod_custom.h>
#include <mcptam/Sample.h>
#include <TooN/Cholesky.h>
#include <TooN/SVD.h>
#include <unordered_set>

//#include <Eigen/Core>
//using Eigen::Map;

#include <g2o/core/sparse_block_matrix.h>

TrackerCovariance::TrackerCovariance(std::string fileName, std::vector<int> vAnalysisMeasNum, int nNumPredPoints)
{
  mpLinearSolver = new g2o::LinearSolverCholmodCustom<Eigen::Matrix2d>(6);
  mFileName = fileName;
  mvAnalysisMeasNum = vAnalysisMeasNum;
  mnNumPredPoints = nNumPredPoints;
  
  std::sort(mvAnalysisMeasNum.begin(), mvAnalysisMeasNum.end());
}

TrackerCovariance::~TrackerCovariance()
{
  delete mpLinearSolver;
}

TooN::Matrix<6> TrackerCovariance::CalcCovariance(TrackerDataPtrVector& vpAllMeas, bool bDoAnalysis)
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
  
  TooN::Matrix<6,6,double> m6Info = TooN::wrapMatrix<6,6,double>(result.data());
  TooN::Cholesky<6> cholInfo(m6Info);
  TooN::Matrix<6,6,double> m6CovRet = cholInfo.get_inverse();
  
  if(bDoAnalysis)
  {
    ROS_ASSERT(!mFileName.empty());
    ROS_ASSERT(mvAnalysisMeasNum.size() > 0);
    ROS_ASSERT(mnNumPredPoints > 1);
    
    std::vector<std::pair<int, double> > vData;
    std::vector<TooN::Matrix<6> > vCov;
    
    // The TrackerDataPtrVector we were given for the measurements is already randomly shuffled, 
    // so don't bother doing anything other than taking slices of J1_Sigma_J1t and J2 from 0 to
    // analysis meas num
    for(unsigned i=0; i < mvAnalysisMeasNum.size(); ++i)
    {
      int nAnalysisMeasNum = mvAnalysisMeasNum[i];
      if(nAnalysisMeasNum > (int)vpAllMeas.size())
        continue;
      
      int nStartMeasIdx = Sample::uniform(0, vpAllMeas.size()-nAnalysisMeasNum);
      int nStartBlockIdx = vpAllMeas[nStartMeasIdx]->mnBlockIdx;
      int nEndBlockIdx = vpAllMeas[nStartMeasIdx+nAnalysisMeasNum-1]->mnBlockIdx;
      
      g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t_analysis = J1_Sigma_J1t->slice(nStartBlockIdx, nEndBlockIdx, nStartBlockIdx, nEndBlockIdx, false);
      Eigen::MatrixXd J2_analysis = J2.block(nStartMeasIdx*2,0,nAnalysisMeasNum*2,6);
      Eigen::MatrixXd Zinv_J2_analysis(nAnalysisMeasNum*2, 6);
      
      mpLinearSolver->init();
      mpLinearSolver->solve(*J1_Sigma_J1t_analysis, Zinv_J2_analysis.data(), J2_analysis.data());
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_analysis = J2_analysis.transpose() * Zinv_J2_analysis; 
      TooN::Matrix<6,6,double> m6Info_analysis = TooN::wrapMatrix<6,6,double>(result_analysis.data());
      
      cholInfo.compute(m6Info_analysis);
      
      TooN::Matrix<6,6,double> m6Cov_analysis = cholInfo.get_inverse();
      
      vCov.push_back(m6Cov_analysis);
      vData.push_back(std::make_pair(nAnalysisMeasNum, TooN::norm_fro(m6Cov_analysis)));
    }
    
    /*
    int nNumPredPoints = mnNumPredPoints;
    
    if(nNumPredPoints > (int)vData.size())
      nNumPredPoints = vData.size();
      
    TooN::Vector<> vX(nNumPredPoints);
    TooN::Vector<> vY(nNumPredPoints);
    
    for(int i=0; i < nNumPredPoints; ++i)
    {
      vX[i] = log(vData[i].first);
      vY[i] = log(vData[i].second);
    }
    
    TooN::Vector<> vCoeff = PolyFit(vX, vY, 1);
    double b = vCoeff[1];
    double c = exp(vCoeff[0]);
    */
    std::ofstream ofs(mFileName, std::ofstream::app);  // append to file rather than overwrite
    ROS_ASSERT(ofs.is_open());
    
    ofs << "--------------------------" << std::endl;
    //ofs << "Used "<<nNumPredPoints<<" points for prediction"<<std::endl;
    //ofs << "MeasNum, CovNorm, PredCovNorm, DiffNorm" << std::endl;
    ofs << "MeasNum, CovNorm" << std::endl;
    
    for(unsigned i=0; i < vData.size(); ++i)
    {
      /*
      double dPredicted = c * pow(vData[i].first, b);
      double dRatio = dPredicted/vData[nNumPredPoints-1].second;
      TooN::Matrix<6> m6CovPredicted = vCov[nNumPredPoints-1] * dRatio;
      TooN::Matrix<6> m6Diff = m6CovPredicted - vCov[i];
      
      ofs << vData[i].first << ", " << vData[i].second << ", " << dPredicted << ", " << TooN::norm_fro(m6Diff) << std::endl;
      */
      
      ofs << vData[i].first << ", " << vData[i].second << std::endl;
    }
    
    /*
    ofs << "vX, vY" << std::endl;
    for(int i=0; i < nNumPredPoints; ++i)
    {
      ofs << vX[i] <<", "<< vY[i] << std::endl;
    }
    */
    
    ofs.close();
  }
  
  delete J1_Sigma_J1t; 
  return m6CovRet;
}

TooN::Vector<> TrackerCovariance::PolyFit(TooN::Vector<> vX, TooN::Vector<> vY, int nDegree)
{
  TooN::Vector<> vEmpty(0);
  
  if(vX.size() != vY.size())
    return vEmpty;
    
  if(nDegree < 1)
    return vEmpty;
    
  int nDimensions = vX.size();
  
  if(nDimensions <= nDegree)
    return vEmpty;
    
  // Uses the Vandermonde matrix method of fitting a least squares polynomial to a set of data
  TooN::Matrix<> mxVandermondeTrans(nDegree+1,nDimensions);
  
  mxVandermondeTrans[0] = TooN::Ones(nDimensions);
  mxVandermondeTrans[1] = vX;
  
  for(int i=2; i <= nDegree; ++i)
  {
    mxVandermondeTrans[i] = mxVandermondeTrans[i-1] * vX.as_diagonal();
  }
  
  // create the SVD decomposition
  TooN::SVD<> svd(mxVandermondeTrans.T());
  
  TooN::Vector<> a = svd.backsub(vY);
  
  return a;
}
