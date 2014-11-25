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

int TrackerCovariance::snNumFitPoints = 3;
int TrackerCovariance::snNumEvalPerMeasNum = 10;
int TrackerCovariance::snNumRandomFits = 50;
int TrackerCovariance::snMinMeasNum = 10;

TrackerCovariance::TrackerCovariance()
{
  mpLinearSolver = new g2o::LinearSolverCholmodCustom<Eigen::Matrix2d>(6);
}

TrackerCovariance::~TrackerCovariance()
{
  delete mpLinearSolver;
}

void TrackerCovariance::CreateMatrices(TrackerDataPtrVector& vpMeas, g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t, Eigen::MatrixXd* J2)
{
  int nMeasNum = vpMeas.size();
  int* blockIndices = new int[nMeasNum];
  
  for(int i=0; i < nMeasNum; ++i)
  {
    vpMeas[i]->mnBlockIdx = i;
    blockIndices[i] = (i+1)*2;
  }
  
  J1_Sigma_J1t = new g2o::SparseBlockMatrix<Eigen::Matrix2d>(blockIndices, blockIndices, nMeasNum, nMeasNum);
  delete[] blockIndices;
  
  J2 = new Eigen::MatrixXd(nMeasNum*2, 6);
  
  TooN::Matrix<3> m3TempCov;
  for(unsigned i=0; i < vpMeas.size(); ++i)
  {
    TrackerData &td = *vpMeas[i];
    
    // Diagonal element
    Eigen::Matrix2d m2PixelCov = Eigen::Matrix2d::Identity() * (1.0/td.mdSqrtInvNoise);
    Eigen::Matrix2d* pm2Diag = J1_Sigma_J1t->block(td.mnBlockIdx, td.mnBlockIdx, true);
    
    // NOTE: Eigen's default ordering is column major. TooN's is row major. I don't know if any of 
    // g2o's internals assume column major ordering when working with Eigen matrices, so I don't want
    // to set that.     
    Eigen::Matrix<double, 2, 3, Eigen::RowMajor> m23Jac = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(td.mm23Jacobian.get_data_ptr());
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m3WorldCov = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(td.mPoint.mm3WorldCov.get_data_ptr());
    
    *pm2Diag = m23Jac * m3WorldCov * m23Jac.transpose() + m2PixelCov;
    
    Eigen::Matrix<double, 2, 6, Eigen::RowMajor> m26Jac = Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor> >(td.mm26Jacobian.get_data_ptr());
    J2->block(i*2, 0, 2, 6) = m26Jac;
    
    // Off diagonal elements
    for(unsigned j=i+1; j < vpMeas.size(); ++j)
    {
      TrackerData& td_other = *vpMeas[j];
    
      // They refer to the same point
      if(&td.mPoint == &td_other.mPoint)
      {
        m3TempCov = td.mPoint.mm3WorldCov;
      }
      else  // They refer to different points
      {
        // Cross cov with other 
        bool bExists = td.mPoint.CrossCov(&td_other.mPoint, m3TempCov);
        ROS_ASSERT(bExists);
      }
      
      Eigen::Matrix2d* pm2OffDiag = J1_Sigma_J1t->block(td.mnBlockIdx, td_other.mnBlockIdx, true);
      
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m3CrossCov = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> >(m3TempCov.get_data_ptr());
      Eigen::Matrix<double, 2, 3, Eigen::RowMajor> m23OtherJac = Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> >(td_other.mm23Jacobian.get_data_ptr());
    
      *pm2OffDiag = m23Jac * m3CrossCov * m23OtherJac.transpose();
    }
  }
}

TooN::Matrix<6> TrackerCovariance::CalcCovarianceFull(TrackerDataPtrVector& vpAllMeas)
{
  g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t;
  Eigen::MatrixXd* J2;
  
  ros::Time buildStart = ros::Time::now();
  std::cerr<<"About to build cross cov system with "<<vpAllMeas.size()<<" measurements"<<std::endl;
  
  CreateMatrices(vpAllMeas, J1_Sigma_J1t, J2);
  
  std::cerr<<"Built cross cov system in "<<ros::Time::now() - buildStart<<" seconds"<<std::endl;
  
  Eigen::MatrixXd Zinv_J2(J2->rows(), J2->cols());  // Z = J1 * Sigma * J1^T
  
  ros::Time solveStart = ros::Time::now();
  std::cerr<<"About to solve cross cov system"<<std::endl;
  mpLinearSolver->init();
  mpLinearSolver->solve(*J1_Sigma_J1t, Zinv_J2.data(), J2->data());
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result = J2->transpose() * Zinv_J2; 
  
  TooN::Matrix<6,6,double> m6Info = TooN::wrapMatrix<6,6,double>(result.data());
  TooN::Cholesky<6> cholInfo(m6Info);
  TooN::Matrix<6,6,double> m6CovRet = cholInfo.get_inverse();
  
  std::cerr<<"Solved cross cov system in "<<ros::Time::now() - solveStart<<" seconds"<<std::endl;
  
  delete J1_Sigma_J1t; 
  delete J2;
  
  return m6CovRet;
}

TooN::Matrix<6> TrackerCovariance::CalcCovarianceEstimate(TrackerDataPtrVector& vpTruncMeas, int nQueryPoint, std::string fileName)
{
  g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t;
  Eigen::MatrixXd* J2;
  
  ros::Time buildStart = ros::Time::now();
  std::cerr<<"About to build cross cov system with "<<vpTruncMeas.size()<<" measurements"<<std::endl;
  
  CreateMatrices(vpTruncMeas, J1_Sigma_J1t, J2);
  
  std::cerr<<"Built cross cov system in "<<ros::Time::now() - buildStart<<" seconds"<<std::endl;
  
  int nTotalMeasNum = vpTruncMeas.size();
  
  int nFitMaxMeas = nTotalMeasNum - TrackerCovariance::snNumRandomFits;
  int nFitMinMeas = TrackerCovariance::snMinMeasNum;
  double dFitDeltaMeas = (nFitMaxMeas - nFitMinMeas) / (TrackerCovariance::snNumFitPoints - 1.0);
  int nNumEvals = TrackerCovariance::snNumEvalPerMeasNum;
  
  // Vector of measurement numbers that we'll generate cov norms for
  std::vector<int> vFitMeasNum;
  vFitMeasNum.push_back(nFitMinMeas);
  vFitMeasNum.push_back(nFitMaxMeas);
  
  for(int i=0; i < TrackerCovariance::snNumFitPoints-2; ++i)
  {
    int nFitMeasNum = std::round(nFitMinMeas + (i+1)*dFitDeltaMeas);
    vFitMeasNum.push_back(nFitMeasNum);
  }
  
  // Make sure we put largest meas num at end since we'll index into it later
  std::sort(vFitMeasNum.begin(), vFitMeasNum.end());
  
  // This will hold, for each tried measurement number, the vector of resulting cov norms and matrices
  std::vector<std::vector<std::pair<double, TooN::Matrix<6> > > > vEvals(vFitMeasNum.size());
  TooN::Cholesky<6> cholInfo;  // used to invert the information matrix
  
  for(unsigned i=0; i < vFitMeasNum.size(); ++i)
  {
    int nFitMeasNum = vFitMeasNum[i];
    
    // Step value for going through system matrices and extracting sub-matrices.
    // Distributes submatrix slices evenly through available range
    int nMatrixDelta = nFitMeasNum - std::ceil((nFitMeasNum*nNumEvals - nTotalMeasNum)/(nNumEvals - 1.0));
    ROS_ASSERT((nNumEvals-1)*nMatrixDelta + nFitMeasNum <= nTotalMeasNum);
    
    vEvals[i].resize(nNumEvals);
    
    for(int j=0; j < nNumEvals; ++j)
    {
      int nStartMeasIdx = j*nMatrixDelta;
      int nStartBlockIdx = vpTruncMeas[nStartMeasIdx]->mnBlockIdx;
      int nEndBlockIdx = vpTruncMeas[nStartMeasIdx+nFitMeasNum-1]->mnBlockIdx;
      
      g2o::SparseBlockMatrix<Eigen::Matrix2d>* J1_Sigma_J1t_eval = J1_Sigma_J1t->slice(nStartBlockIdx, nEndBlockIdx, nStartBlockIdx, nEndBlockIdx, false);
      Eigen::MatrixXd J2_eval = J2->block(nStartMeasIdx*2,0,nFitMeasNum*2,6);
      Eigen::MatrixXd Zinv_J2_eval(nFitMeasNum*2, 6);
      
      mpLinearSolver->init();
      mpLinearSolver->solve(*J1_Sigma_J1t_eval, Zinv_J2_eval.data(), J2_eval.data());
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result_eval = J2_eval.transpose() * Zinv_J2_eval; 
      TooN::Matrix<6,6,double> m6Info_eval = TooN::wrapMatrix<6,6,double>(result_eval.data());
      
      cholInfo.compute(m6Info_eval);
      
      TooN::Matrix<6,6,double> m6Cov_eval = cholInfo.get_inverse();
      
      // Store at appropriate location for lookup later
      vEvals[i][j] = std::make_pair(TooN::norm_fro(m6Cov_eval), m6Cov_eval);
    }
  }
  
  // Now it's time to fit some curves
  std::vector<std::pair<double,double> > vCurveCoeffs(TrackerCovariance::snNumRandomFits);
  
  for(int i=0; i < TrackerCovariance::snNumRandomFits; ++i)
  {
    TooN::Vector<> vX(vFitMeasNum.size());
    TooN::Vector<> vY(vFitMeasNum.size());

    for(unsigned j=0; j < vFitMeasNum.size(); ++j)
    {
      // Choose a random result for each measurement number
      int nRandIdx = Sample::uniform(0, nNumEvals-1);
      
      // Logs because we're fitting y = c * x^b using standard linear regression
      vX[j] = log(vFitMeasNum[j]);
      vY[j] = log(vEvals[j][nRandIdx]);
    }
    
    TooN::Vector<> vCoeff = PolyFit(vX, vY, 1);
    double b = vCoeff[1];
    double c = exp(vCoeff[0]);
    
    vCurveCoeffs[i].first = c;
    vCurveCoeffs[i].second = b;
  }
  
  // Evaluate these curves, the one we want is the one that produces the median value 
  // at nQueryPoint
  
  std::vector<std::pair<double, int> > vPredictedValues(vCurveCoeffs.size());
  
  for(unsigned i=0; i < vCurveCoeffs.size(); ++i)
  {
    double b = vCurveCoeffs[i].second;
    double c = vCurveCoeffs[i].first;
    
    double dPredicted = c * pow(nQueryPoint, b);
    vPredictedValues[i].first = dPredicted;
    vPredictedValues[i].second = i;
  }
  
  // Specialized sort, only puts the median point in the right spot for extraction
  std::nth_element(vPredictedValues.begin(), vPredictedValues.begin() + vPredictedValues.size()/2, vPredictedValues.end());
    
  double dMedianValue = vPredictedValues[vPredictedValues.size()/2].first;
  int nMedianCurveIdx = vPredictedValues[vPredictedValues.size()/2].second;
    
  double b_median = vCurveCoeffs[nMedianCurveIdx].second;
  double c_median = vCurveCoeffs[nMedianCurveIdx].first;
  
  // Find the point amongst the ones with the largest meas num that best fits
  // this curve. We'll then scale that point's covariance matrix by the appropriate amount
  
  std::vector<std::pair<double, TooN::Matrix<6> > >& vLargestMeasNumEvals = vEvals[vEvals.size()-1];
  double dCurveAtLargest = c_median * pow(vFitMeasNum[vFitMeasNum.size()-1], b_median);
  
  double dSmallestAbsError = std::numeric_limits<double>::max();
  int nSmallestErrorIdx = -1;
  
  for(unsigned i=0; i < vLargestMeasNumEvals.size(); ++i)
  {
    double dAbsError = fabs(vLargestMeasNumEvals[i].first - dCurveAtLargest);
    
    if(dAbsError < dSmallestAbsError)
    {
      dSmallestAbsError = dAbsError;
      nSmallestErrorIdx = i;
    }
  }
  
  ROS_ASSERT(nSmallestErrorIdx != -1);
  
  double dSmallestErrorNorm = vLargestMeasNumEvals[nSmallestErrorIdx].first;
  TooN::Matrix<6> m6SmallestErrorCov = vLargestMeasNumEvals[nSmallestErrorIdx].second;
  
  double dRatio = dMedianValue/dSmallestErrorNorm;
  TooN::Matrix<6> m6CovPredicted = m6SmallestErrorCov * dRatio;
  
  if(!fileName.empty())
  {
    std::ofstream ofs(fileName, std::ofstream::app);  // append to file rather than overwrite
    ROS_ASSERT(ofs.is_open());
    
    ofs << "--------------------------" << std::endl;
    ofs << "------- Points (meas num, cov norm) -----------" << std::endl;
    
    for(unsigned i=0; i < vFitMeasNum.size(); ++i)
    {
      int nFitMeasNum = vFitMeasNum[i];
      for(int j=0; j < nNumEvals; ++j)
      {
        double dEvalNorm = vEvals[i][j].first;
        ofs << nFitMeasNum <<", " << dEvalNorm << std::endl;
      }
    }
    
    ofs << "------- Curves (c,b as in y = c * x^b)-----------" << std::endl;
    
    for(unsigned i=0; i < vCurveCoeffs.size(); ++i)
    {
      double b = vCurveCoeffs[i].second;
      double c = vCurveCoeffs[i].first;
      
      ofs << c << ", " << b << std::endl;
    }
    
    ofs.close();
  }

  return m6CovPredicted;
}

/*
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
    

    std::ofstream ofs(mFileName, std::ofstream::app);  // append to file rather than overwrite
    ROS_ASSERT(ofs.is_open());
    
    ofs << "--------------------------" << std::endl;
    //ofs << "Used "<<nNumPredPoints<<" points for prediction"<<std::endl;
    //ofs << "MeasNum, CovNorm, PredCovNorm, DiffNorm" << std::endl;
    ofs << "MeasNum, CovNorm" << std::endl;
    
    for(unsigned i=0; i < vData.size(); ++i)
    {
      ofs << vData[i].first << ", " << vData[i].second << std::endl;
    }
    
    ofs.close();
  }
  
  delete J1_Sigma_J1t; 
  return m6CovRet;
}
*/

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
