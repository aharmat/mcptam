/****************************************************************************************
 *
 * \file EntropyComputation.cpp
 * 
 *
 * Copyright 2015   Arun Das, University of Waterloo (adas@uwaterloo.ca)
 *
 *
 ****************************************************************************************/



#include <mcptam/EntropyComputation.h>

void ComputePointMotionInAnchorKF(TooN::Vector<3> pointInCamera, KeyFrame& anchorKF, double deltaLength, TooN::SO3<>& Rp)
{
    double dRho = 1.0/deltaLength;
    TooN::Vector<3> pointInCamDir = pointInCamera * dRho;
    TooN::Vector<3> axis = pointInCamDir ^ TooN::makeVector(0,0,1);
    double angle = asin(TooN::norm(axis));
    TooN::normalize(axis);
    axis *= angle;
    Rp = TooN::SO3<>::exp(axis); 

}



void PerturbPoint(TooN::Vector<3> pointInCamera, TooN::SO3<> rp, double deltaLength,  TooN::Matrix<3>& jacobian)
{
    // d_beta, rotation about x axis
    jacobian.T()[0] = rp.inverse() * (TooN::SO3<>::generator_field(0,rp*pointInCamera));
    // d_alpha, rotation about y axis
    jacobian.T()[1] = rp.inverse() * (TooN::SO3<>::generator_field(1,rp*pointInCamera));
    // d_rho, change in inverse depth
    jacobian.T()[2] = -1 * pointInCamera * deltaLength;

}


double ComputeUpdatedCovariance(int pointLevel, TooN::Vector<2> imageJacobian, double priorPointCovariance)
{
    double levelScaleSigmaSquared = LevelScale(pointLevel)*LevelScale(pointLevel)*10;
    TooN::Matrix<2> R = TooN::Identity*((levelScaleSigmaSquared)); 
    TooN::Matrix<2> S = (imageJacobian.as_col()*priorPointCovariance*imageJacobian.as_row()) +R;
    TooN::Matrix<1,2> K = priorPointCovariance*imageJacobian.as_row()*TooN::inv(S);
    double updatedCovariance = (1.0 - (K[0])*imageJacobian)*priorPointCovariance;

    return updatedCovariance;

}



//this function evaluates a point's entropy reduction
double EvaluatePointEntropyReduction(Tracker* tracker, MapPoint& point, KeyFrame& trackerKF, double priorPointCovariance, int pointLevel, double& prevEntropy)
{
    //step 1: find the point's anchor keyframe
    KeyFrame& anchorKF = *point.mpPatchSourceKF;

    //step 2: extract the location of the anchor keyframe
    TooN::SE3<> anchorCamFromWorld = anchorKF.mse3CamFromWorld;

    //step 3: compute the relative transform from the anchor keyframe to the current tracker location
    TooN::SE3<> trackerCamFromWorld = trackerKF.mse3CamFromWorld;
    TooN::SE3<> relativeTransform = trackerCamFromWorld*anchorCamFromWorld.inverse();

    //step 4: compute the motion derivatives of the point wrt the anchor keyframe
    TooN::Vector<3> pointInCamera = anchorKF.mse3CamFromWorld*point.mv3WorldPos;
    double deltaLength = TooN::norm(pointInCamera);
    TooN::SO3<> Rp;
    ComputePointMotionInAnchorKF(pointInCamera, anchorKF, deltaLength, Rp);

    /*
    double dRho = 1.0/dLength;
    TooN::Vector<3> v3PointInCamDir = v3PointInCam * dRho;
    TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
    double angle = asin(TooN::norm(v3Axis));
    TooN::normalize(v3Axis);
    v3Axis *= angle;
    TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
    */

    // Now we are going to figure out how the image point changes when the point is perturbed

    TooN::Matrix<3> m3Jac;
    PerturbPoint(pointInCamera, Rp, deltaLength, m3Jac);
   
    /*
    TooN::Matrix<3> m3Jac;
    // d_beta, rotation about x axis
    m3Jac.T()[0] = Rp.inverse() * (TooN::SO3<>::generator_field(0,Rp*v3PointInCam));
    // d_alpha, rotation about y axis
    m3Jac.T()[1] = Rp.inverse() * (TooN::SO3<>::generator_field(1,Rp*v3PointInCam));
    // d_rho, change in inverse depth
    m3Jac.T()[2] = -1*v3PointInCam / dRho;
    */

    //step 5: transform those motion derivatives into the observing frame
    m3Jac = relativeTransform.get_rotation().get_matrix() * m3Jac;

    //step 6: use the Taylor derivatives to map the point motion derivatives into image jacobians
    TooN::Vector<3> v3dTheta, v3dPhi;
    TooN::Vector<3> v3PointInTrakerCam = trackerKF.mse3CamFromWorld*point.mv3WorldPos;
    TaylorCamera& camera =  tracker->mmCameraModels[trackerKF.mCamName];
    camera.GetCamSphereDeriv(v3PointInTrakerCam, v3dTheta, v3dPhi);  // derivatives of spherical coords wrt camera frame coords*/

    //step 7: using the jacobians, compute the predicted depth covariance of the point
    const TooN::Vector<3> v3Motion = m3Jac.T()[2];
    // Convert to motion on the sphere
    TooN::Vector<2> v2CamSphereMotion;
    v2CamSphereMotion[0] = v3dTheta * v3Motion;  // theta component
    v2CamSphereMotion[1] = v3dPhi * v3Motion;  // phi component

    // Convert to motion on the image
    TooN::Matrix<2> m2CamDerivs = camera.GetProjectionDerivs();
    TooN::Vector<2> v2ImageJacobian = m2CamDerivs * v2CamSphereMotion;

    //step 8: using the prior point covariance and the new point covariance
    /*
    double levelScaleSigmaSquared = LevelScale(pointLevel)*LevelScale(pointLevel)*10;
    TooN::Matrix<2> R = TooN::Identity*((levelScaleSigmaSquared)); 
    TooN::Matrix<2> S = (v2ImageJacobian.as_col()*priorPointCovariance*v2ImageJacobian.as_row()) +R;
    TooN::Matrix<1,2> K = priorPointCovariance*v2ImageJacobian.as_row()*TooN::inv(S);
    double updatedCov = (1.0 - (K[0])*v2ImageJacobian)*priorPointCovariance;
    */
    double updatedCovariance = ComputeUpdatedCovariance(pointLevel, v2ImageJacobian, priorPointCovariance);


    //step 9: return the entropy reduction.
    double previousEntropy = compute_point_entropy_scalar(priorPointCovariance);
    double updatedEntropy =  compute_point_entropy_scalar(updatedCovariance);

    double depthEntropyReduction = previousEntropy - updatedEntropy;
    prevEntropy = previousEntropy; //save this

    ///// SABA 
    ROS_INFO("ENTROPY EVALUATE FUNCTIONALIZED: %f", depthEntropyReduction);    
    return depthEntropyReduction;
}

TooN::Vector<3> EvaluateTrackerEntropy(Tracker* tracker) 
{
    TooN::Vector<3> trackerEntropy;

    double covXX = tracker->mm6PoseCovariance(0,0);
    double covYY = tracker->mm6PoseCovariance(1,1);
    double covZZ = tracker->mm6PoseCovariance(2,2);

    double hx = compute_point_entropy_scalar(covXX);
    double hy = compute_point_entropy_scalar(covYY);
    double hz = compute_point_entropy_scalar(covZZ);

    trackerEntropy[0] = hx; trackerEntropy[1] = hy; trackerEntropy[2] = hz;

    return trackerEntropy;

}
