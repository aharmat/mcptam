
#include <mcptam/EntropyComputation.h>


//this function evaluates a point's entropy reduction
double EvaluatePoint(Tracker* tracker, MapPoint& point, KeyFrame& tracker_kf, double priorPointCovariance, int pointLevel, double& prevEntropy)
{
		
	//step 1: find the point's anchor keyframe
	KeyFrame& anchorKF = *point.mpPatchSourceKF;
	
	//check to see if the anchorKF and trackerKF are from the SAME MKF parent.  If this is the case, ignore
	//if (anchorKF.mpParent == tracker_kf.mpParent)
		//return 0; //todo (adas): do we need this still?
	
	//step 2: extract the location of the anchor keyframe
	TooN::SE3<> anchorCamFromWorld = anchorKF.mse3CamFromWorld;
	
	//step 3: compute the relative transform from the anchor keyframe to the current tracker location
	TooN::SE3<> trackerCamFromWorld = tracker_kf.mse3CamFromWorld;
	TooN::SE3<> relativeTransform = trackerCamFromWorld*anchorCamFromWorld.inverse(); 

	//step 4: compute the motion derivatives of the point wrt the anchor keyframe
	
	 TooN::Vector<3> v3PointInCam = anchorKF.mse3CamFromWorld*point.mv3WorldPos;
     double dLength = TooN::norm(v3PointInCam);
     double dRho = 1.0/dLength;
     TooN::Vector<3> v3PointInCamDir = v3PointInCam * dRho;
     TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
     double angle = asin(TooN::norm(v3Axis));
     TooN::normalize(v3Axis);
     v3Axis *= angle;
     TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
        
     // Now we are going to figure out how the image point changes when the point is perturbed
     TooN::Matrix<3> m3Jac;
     // d_beta, rotation about x axis
      m3Jac.T()[0] = Rp.inverse() * (TooN::SO3<>::generator_field(0,Rp*v3PointInCam));
     // d_alpha, rotation about y axis
      m3Jac.T()[1] = Rp.inverse() * (TooN::SO3<>::generator_field(1,Rp*v3PointInCam));
      // d_rho, change in inverse depth
     m3Jac.T()[2] = -1*v3PointInCam / dRho;
	
	//step 5: transform those motion derivatives into the observing frame
	
	 m3Jac = relativeTransform.get_rotation().get_matrix() * m3Jac;
	
	//step 6: use the Taylor derivatives to map the point motion derivatives into image jacobians
	
	TooN::Vector<3> v3_dTheta, v3_dPhi;
	TooN::Vector<3> v3PointInTrakerCam = tracker_kf.mse3CamFromWorld*point.mv3WorldPos;
	TaylorCamera& camera =  tracker->mmCameraModels[tracker_kf.mCamName];
	camera.GetCamSphereDeriv(v3PointInTrakerCam, v3_dTheta, v3_dPhi);  // derivatives of spherical coords wrt camera frame coords*/
	
	//step 7: using the jacobians, compute the predicted depth covariance of the point 
	
	const TooN::Vector<3> v3Motion = m3Jac.T()[2];  
    // Convert to motion on the sphere
    TooN::Vector<2> v2CamSphereMotion;
    v2CamSphereMotion[0] = v3_dTheta * v3Motion;  // theta component
    v2CamSphereMotion[1] = v3_dPhi * v3Motion;  // phi component
          
    // Convert to motion on the image
    TooN::Matrix<2> m2CamDerivs = camera.GetProjectionDerivs();      
    TooN::Vector<2> v2ImageJacobian = m2CamDerivs * v2CamSphereMotion;
          
	//step 8: using the prior point covariance and the new point covariance, compute the reduction in entropy
	double levelScaleSigmaSquared = LevelScale(pointLevel)*LevelScale(pointLevel)*10;
	TooN::Matrix<2> R = TooN::Identity*((levelScaleSigmaSquared)); //todo (adas): is this the best noise model?
	TooN::Matrix<2> S = (v2ImageJacobian.as_col()*priorPointCovariance*v2ImageJacobian.as_row()) +R;
	TooN::Matrix<1,2> K = priorPointCovariance*v2ImageJacobian.as_row()*TooN::inv(S);
	double updatedCov = (1.0 - (K[0])*v2ImageJacobian)*priorPointCovariance;
			
	//step 10: return the entropy reduction.
	
	double previousEntropy = compute_point_entropy_scalar(priorPointCovariance);
	double updatedEntropy =  compute_point_entropy_scalar(updatedCov);
	
	double depthEntropyReduction = previousEntropy - updatedEntropy;
	prevEntropy = previousEntropy; //save this
	
	//todo (adas): add max/min for the entropy reduction? check for inf, nan, other strange outputs

	//shouldn't be able to reduce it by more than factor 100
	//if( updatedEntropy< previousEntropy/100.0)
	//	depthEntropyReduction = 0; //todo (adas) do we need this?

	return depthEntropyReduction;
}

TooN::Vector<3> EvaluateTracker(Tracker* tracker) //TODO (adas): include rotation states
{
	TooN::Vector<3> trackerEntropy;
	
	double cov_xx = tracker->mm6PoseCovariance(0,0);
	double cov_yy = tracker->mm6PoseCovariance(1,1);
	double cov_zz = tracker->mm6PoseCovariance(2,2);
	
	double hx = compute_point_entropy_scalar(cov_xx);
	double hy = compute_point_entropy_scalar(cov_yy);
	double hz = compute_point_entropy_scalar(cov_zz);
	
	trackerEntropy[0] = hx; trackerEntropy[1] = hy; trackerEntropy[2] = hz;
	
	return trackerEntropy;
	
}
