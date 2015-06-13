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


//=========================================================================================
//
// Copyright 2012 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// There are a bunch of classes declared and defined in this file, instead of declaring
// them in a header file. The reason is that the g2o namespace has conflicts between the Eigen::Map class
// and mcptam's own Map class. It would be nice to look into this and find a solution.
//
//=========================================================================================

#include <mcptam/ChainBundle.h>
#include <mcptam/MEstimator.h>
#include <iomanip>
#include <ros/ros.h>

// These next two lines are needed otherwise Eigen::Map and our own Map class
// get confused by the compiler and barely comprehensible messages get spewed
#include <Eigen/Core>
using Eigen::Map;
  
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/hyper_graph_action.h>


//debugging
static int poseID = 0;

/// A TooN SE3 represented as a vertex in the g2o pose graph
class VertexPoseSE3 : public g2o::BaseVertex<6, TooN::SE3<> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPoseSE3()
    {
      setToOrigin();
      _nID = poseID++;
    }

    virtual void setToOriginImpl() 
    {
      _estimate = TooN::SE3<>();
    }

    virtual void oplusImpl(const double* update)
    {
      TooN::Vector<6> v6Update = TooN::makeVector(update[0],update[1],update[2],update[3],update[4],update[5]);
      _estimate = TooN::SE3<>::exp(v6Update) * _estimate;
    }
    
    /*
    void printEstimate() const
    {
      ROS_INFO_STREAM("Pose "<<_nID<<" estimate: "<<std::endl<<_estimate);
    }
    */
    
    virtual bool read(std::istream& is)
    {
      TooN::SE3<> se3;
      is >> se3;
      setEstimate(se3);
      return true;
    }

    virtual bool write(std::ostream& os) const
    {
      os << estimate();
      return os.good();
    }
    
    int _nID;

};

/// Holds the inter-link transforms needed to compute the Jacobians of measurements relative to
/// their pose chains. Since multiple measurements share one pose chain (ie multiple measurements
/// from one KeyFrame), it woudl be wasteful to calculate these inter-link transforms over
/// and over again inside each measurement class.
class PoseChainHelper
{
  public:
    void UpdateTransforms()
    {
      _vTransforms.resize(_vpVertices.size());
      TooN::SE3<> se3BfW;  // The pose of the current chain link, identity to start
      
      // Traverse the chain, starting at the root going forward
      for(unsigned i=0; i < _vpVertices.size(); ++i) 
      {
        const VertexPoseSE3* pPoseVertex = _vpVertices[i];
        se3BfW = pPoseVertex->estimate() * se3BfW;  // CHECK!! GOOD
        
        _vTransforms[i].first = se3BfW;
      }
      
      // At this point, se3BfW contains the pose of the final chain link in the world frame
      // Before continuing, want to get the transforms of the final pose from each of the
      // vertices, and store them in _vTransforms as well. Actually, we only want the 
      // rotations, so we'll zero the translation components.
      
      TooN::SE3<> se3CfB;  // The pose of the final chain link in the frame of the current one, identity to start
      
      for(int i=_vpVertices.size()-1; i >= 0; --i) 
      {
        const VertexPoseSE3* pPoseVertex = _vpVertices[i];
        
        // Note order of these lines!
        _vTransforms[i].second = se3CfB;
        _vTransforms[i].second.get_translation() = TooN::Zeros;
        se3CfB = se3CfB * pPoseVertex->estimate();  // note order of multiplication!
      }
    }
    
    // Test if two poise chains move together from a derivative calculation
    // point of view. For example, if they share one movable first vertex, and 
    // then all the other vertices are fixed, the end poses would experience 
    // identical transformations when the first vertex is perturbed.
    // The chains are tested up to nDepth depth for "this" chain
    bool MoveTogether(const PoseChainHelper& other, int nDepth) const
    {
      ROS_ASSERT(nDepth >= 0);
      ROS_ASSERT((int)_vpVertices.size() > nDepth);  // nDepth is zero-based
        
      // find furthest shared vertex
      int nFurthestSharedIdx = -1;
      
      while(true)
      {
        int nTestIdx = nFurthestSharedIdx + 1;
        
        // If one or both have run out of vertices
        if((int)_vpVertices.size() <= nTestIdx || (int)other._vpVertices.size() <= nTestIdx)
          break;
          
        // If the vertices at nTestIdx don't match
        if(_vpVertices[nTestIdx] != other._vpVertices[nTestIdx])
          break;
          
        nFurthestSharedIdx = nTestIdx;
        
        // If we've hit our desired depth, we're good
        if(nFurthestSharedIdx == nDepth)
          return true;
      }
      
      // if they don't share the first vertex, can't be true since they
      // would move completely independently
      if(nFurthestSharedIdx == -1)
        return false;
        
      // From the furthest shared idx forward, all vertices up to nDepth
      // need to be fixed (on our side, not necessarily other), otherwise return false
       
      for(int i=nFurthestSharedIdx; i <= nDepth; ++i)
      {
        if(!_vpVertices[i]->fixed())
          return false;
      }
      
      return true;
    }
  
    std::vector<VertexPoseSE3*> _vpVertices;
    
    /// Holds pairs of transforms, the first is the pose of the link in the world frame, and the second is the pose of the
    /// final link in the link's frame (actually, only the rotation component is needed in the latter)
    std::vector<std::pair<TooN::SE3<>, TooN::SE3<> > > _vTransforms; 
    
    /*
     *  Example of vTransforms for a two pose chain (vTransforms[1].second is identity)
     * 
     *       --------------------- vTransforms[1].first -------------    -- vTransforms[1].second -- 
     *      |                                                       \|/ /                           |
     *    world ----------------------> MKF ------------------------> KF <--------------------------
     *      |                         /|\ |                          /|\
     *       -- vTransforms[0].first --    -- vTransforms[0].second --
     */
};

// debugging
static int pointID = 0;

/// A point, relative to its parent frame, parameterized by a 3-vector as a vertex in the g2o pose graph
class VertexRelPoint : public g2o::BaseVertex<3, TooN::Vector<3> >
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexRelPoint()
    {
      setToOrigin();
      _nID = pointID++;
      _pPoseChainHelper = NULL;
    }
  
    virtual void setToOriginImpl() {
      _estimate = TooN::makeVector(0,0,0);
    }

    virtual void oplusImpl(const double* update)
    {
      // Use this for XYZ update parameterization
      // Make sure the same thing is done in EdgeChainMeas::linearizeOplus
      // The logic here is that the update in XYZ parameterization can be simply added
      // to the point estimate rather than converting from bearing+inverse depth
      //_estimate += TooN::makeVector(update[0], update[1], update[2]);
      //return;
      
      // The update is specified in a coordinate frame where the _estimate
      // vector is first rotated to align with the z axis. This is done because
      // then we don't have to deal with singularities arising from
      // parameterizing  the surface of a sphere with two coordinates (assuming
      // that the size of the update vector is small enough to stay away from
      // these areas, which should be the case)
      
      double dDistBefore = TooN::norm(_estimate);
      double dRhoBefore = 1.0/dDistBefore;
      TooN::Vector<3> v3PointInCamDir = _estimate * dRhoBefore;
      
      // Get the axis of rotation between the the point direction and the z axis
      TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
      double angle = asin(TooN::norm(v3Axis));
      
      TooN::normalize(v3Axis);
      v3Axis *= angle;
      
      // Convert from axis-angle to rotation matrix
      TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
      
      double d_beta = update[0];  // rotation about x axis
      double d_alpha = update[1]; // rotation about y axis
      double d_rho = update[2];   // displacement along z axis
      
      // To update the estimate, first rotate it to align with z axis, apply rotation update, rotate back, then apply scaling update
      _estimate = (1/(dRhoBefore + d_rho)) * (Rp.inverse() * TooN::SO3<>::exp(TooN::makeVector(d_beta,d_alpha,0)) * Rp * v3PointInCamDir);
      
      // If the point is poorly constrained, the estimate can run away easily, making it hard the optimize properly
      // when the next round of optimization is done (potentially with more/better constraints)
      double dDistAfter = TooN::norm(_estimate);
      if(dDistAfter > 1e5)
        _estimate *= 1e5/dDistAfter;
      if(dDistAfter < 1e-5)
        _estimate *= 1e-5/dDistAfter;
      
      /*
      TooN::Vector<3> v3PointInCamDir = _estimate;
      TooN::normalize(v3PointInCamDir);
      
      TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
      double angle = asin(TooN::norm(v3Axis));
      
      if(angle != 0)
      {
        TooN::normalize(v3Axis);
        v3Axis *= angle;
      }
      
      TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
      
      double d_beta = update[0];  // rotation about x axis
      double d_alpha = update[1]; // rotation about y axis
      double d_rho = update[2];
      
      _estimate = (1/(1+d_rho)) * (Rp.inverse() * TooN::SO3<>::exp(TooN::makeVector(d_beta,d_alpha,0)) * Rp * _estimate);
      double dDist = TooN::norm(_estimate);
      if(dDist > 1e5)
        _estimate *= 1e5/dDist;
      if(dDist < 1e-5)
        _estimate *= 1e-5/dDist;
      */
    }
    
    // The current point estimate transformed into global cartesian coordinates
    // Used to project the point into cameras
    TooN::Vector<3> estimateInGlobalCartesian() const
    {
      ROS_ASSERT(_pPoseChainHelper);
      
      // This NEEDS to be uncommented if you're using numerical jacobians!
      // With analytical it doesn't matter, just wasting your time if you leave it uncommented
      //_pPoseChainHelper->UpdateTransforms(); 
      
      TooN::SE3<> se3SfW = _pPoseChainHelper->_vTransforms.back().first;  // Pose of the source KeyFrame that the point is defined relative to      
      
      // This is essentially the same as kfSrc.mse3CamFromWorld.inverse() * v3Cam where v3Cam = kfSrc.mse3CamFromWorld * point.mv3WorldPos
      return se3SfW.inverse() * _estimate;   
    }
    
    /*
    void printEstimate() const
    {
      ROS_INFO_STREAM(<<"Point "<<_nID<<" estimate: "<<_estimate);
    }
    */
    
    virtual bool read(std::istream& is)
    {
      TooN::Vector<3> v3;
      is >> v3;
      setEstimate(v3);
      return true;
    }

    virtual bool write(std::ostream& os) const
    {
      os << estimate();
      return os.good();
    }
    
    int _nID;
    PoseChainHelper* _pPoseChainHelper;  // stores the chain of poses that defines the source pose of the point
};

//debugging
static int measID = 0;

/** @brief A measurement comprising a map point (with pose chain) seen by the final link of another pose chain
 * 
 *  Since any number of poses are allowed in a pose chain, this class inherits from BaseMultiEdge, which
 *  allows connections to an unlimited number of vertices */
class EdgeChainMeas : public g2o::BaseMultiEdge<2, TooN::Vector<2> > 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    /** @param cam The camera model that made the actual measurement
     *  @param cameraName Needed so that if this measurement ends up as an outlier, we can return it encoded as in ChainBundle::GetOutlierMeasurements
     *  @param bVerbose Print a bunch of stuff? */
    EdgeChainMeas(TaylorCamera& cam, std::string cameraName)
    : BaseMultiEdge<2, TooN::Vector<2> > ()
    , _camName(cameraName)
    , _camera(cam)
    {
      _nID = measID++;
      _pPoseChainHelper = NULL;
    }
    
    /** @brief Defines the error function for this measurement */
    virtual void computeError()
    {
      ROS_ASSERT(_pPoseChainHelper);
      
      // This NEEDS to be uncommented if you're using numerical jacobians!
      // With analytical it doesn't matter, just wasting your time if you leave it uncommented
      //_pPoseChainHelper->UpdateTransforms();
      
      TooN::SE3<> se3CfW = _pPoseChainHelper->_vTransforms.back().first;  // Pose of the KeyFrame that made the measurement
      VertexRelPoint* pPointVertex = dynamic_cast<VertexRelPoint*>(_vertices.back());
      ROS_ASSERT(pPointVertex);
        
      TooN::Vector<3> v3Global = pPointVertex->estimateInGlobalCartesian();
      TooN::Vector<3> v3Cam = se3CfW * v3Global;
      TooN::Vector<2> v2Image = _camera.Project(v3Cam);
    
      _m2CamDerivs = _camera.GetProjectionDerivs();
              
      TooN::Vector<2> v2Error = _measurement - v2Image;
      _error(0) = v2Error[0];
      _error(1) = v2Error[1];
    }
    
    /// Get the "score" for this measurement. In a simple implementation this would just be the error squared, but
    /// here we do something a bit more complicated due to robustification 
    virtual double chi2() const 
    {
      double val = _error.dot(information()*_error);
      
      VertexRelPoint* pPointVertex = dynamic_cast<VertexRelPoint*>(_vertices.back());
      ROS_ASSERT(pPointVertex);
      
      // If the point is fixed, then we HAVE to make sure its weight is 1.0 (because it's a calibration point).
      // To do this, return a negative number, which will be interpreted by the robustify function to be "below"
      // the threshold sigma, and thus the weight will be 1.0.
      
      // Make sure we have a robust kernel, otherwise return the normal value
      if(pPointVertex->fixed() && robustKernel())
        val *= -1;
        
      return val;
    }
    
    /*
    void printJacobians()
    {
      for (size_t i = 0; i < _vertices.size(); ++i) { 
        
        g2o::OptimizableGraph::Vertex* vi = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[i]);

        if (vi->fixed())
          continue;
          
        ROS_INFO_STREAM("Vertex "<<i<<" estimate: ");
        
        if(i == _vertices.size()-1)
        {
          const VertexRelPoint* pPointVertex = dynamic_cast<const VertexRelPoint*>(_vertices[i]);
          pPointVertex->printEstimate();
        }
        else
        {
          const VertexPoseSE3* pPoseVertex = dynamic_cast<const VertexPoseSE3*>(_vertices[i]);
          pPoseVertex->printEstimate();
        }
        
        ROS_INFO_STREAM("Jacobian of measurement "<<_nID<<" wrt vertex "<<i);
        ROS_INFO_STREAM(_jacobianOplus[i]);
      }
    }
    */
    
    /// Compute the Jacobians of the measurement
    virtual void linearizeOplus()
    {
      #ifdef G2O_OPENMP
        for (size_t i = 0; i < _vertices.size(); ++i) {
          g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[i]);
          v->lockQuadraticForm();
        }
      #endif
      
      //============ Analytical Jacobian =================
      
      // If you want to compare jacobians computed by analytical and numerical means, choose option 1
      // and print whatever you want out after the numerical method later on. If you don't want to 
      // compare, use option 2, which defines _jacobianOplusTemp as a reference to _jacobianOplus
      
      /*
      // Option 1:
      std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> > _jacobianOplusTemp(_vertices.size());
      */
      
      // Option 2:
      std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> >& _jacobianOplusTemp = _jacobianOplus;
      
      const VertexRelPoint* pPointVertex = dynamic_cast<const VertexRelPoint*>(_vertices.back());
      ROS_ASSERT(pPointVertex);
      ROS_ASSERT(_pPoseChainHelper);
    
      TooN::Vector<3> v3Global = pPointVertex->estimateInGlobalCartesian();
      TooN::Vector<3> v3Cam = _pPoseChainHelper->_vTransforms.back().first * v3Global;
      TooN::Vector<3> v3_dTheta, v3_dPhi;
      
      TaylorCamera::GetCamSphereDeriv(v3Cam, v3_dTheta, v3_dPhi);  // derivatives of spherical coords wrt camera frame coords
    
      unsigned poseN = _pPoseChainHelper->_vpVertices.size();  // number of poses in the pose chain that defines the observing camera
      unsigned pointN = pPointVertex->_pPoseChainHelper->_vpVertices.size();  // number of poses in the pose chain that defines the point
     
      for (unsigned i = 0; i < poseN; ++i)   // loop over the poses corresponding to the observing camera
      {
        const VertexPoseSE3* pPoseVertex = dynamic_cast<const VertexPoseSE3*>(_vertices[i]);

        if (pPoseVertex->fixed())  // Jacobian is zero, skip
          continue;
          
        // Make sure the jacobian is the right size
        if (_jacobianOplusTemp[i].rows() != 2 || _jacobianOplusTemp[i].cols() != 6)
          _jacobianOplusTemp[i].resize(2,6);
          
        // If the observing camera's and the point's pose chains move together, it means that moving the camera
        // doesn't affect the point (i.e. the point is defined relative to the observing camera, or some other
        // camera that is fixed to the one we are looking at). Therefore, the jacobian will be zeros.
        if(_pPoseChainHelper->MoveTogether(*(pPointVertex->_pPoseChainHelper), i))
        {
          _jacobianOplusTemp[i] = JacobianType::Zero(2,6);
          continue;
        }
          
        const TooN::Vector<3> v3Base = _pPoseChainHelper->_vTransforms[i].first * v3Global;  // The point in the current pose's frame
        const TooN::Vector<4> v4Base = unproject(v3Base);
      
        // For each of six degrees of freedom...
        for(int m=0;m<6;m++)
        {
          // Get the motion of the point in the current pose's frame when the pose changes by one of the degrees of freedom
          const TooN::Vector<4> v4Motion_Base = TooN::SE3<>::generator_field(m, v4Base); 
          // Then the motion of the point in the originally measuring camera only depends on the relative rotation between it and the current pose, not the translation
          // Keep in mind we zeroed out the translation components in the pose chain helper's update function
          const TooN::Vector<4> v4Motion_Cam = _pPoseChainHelper->_vTransforms[i].second * v4Motion_Base;  // CHECK!! GOOD
          
          // Convert to motion on the sphere
          TooN::Vector<2> v2CamSphereMotion;
          v2CamSphereMotion[0] = v3_dTheta * v4Motion_Cam.slice<0,3>();  // theta component
          v2CamSphereMotion[1] = v3_dPhi * v4Motion_Cam.slice<0,3>();  // phi component
          
          // Convert to motion on the image
          TooN::Vector<2> v2Column = _m2CamDerivs * v2CamSphereMotion;
          
          // v2Column is the change in the location of the image point
          // HOWEVER: we want the change in the ERROR! This is the negative of the
          // change in the image point
          
          _jacobianOplusTemp[i].col(m)(0) = -1 * v2Column[0];
          _jacobianOplusTemp[i].col(m)(1) = -1 * v2Column[1];
        }
      }
    
      // Now we do the poses corresponding to the point
      for (unsigned i = 0; i < pointN; ++i) 
      {
        // We already processed the first poseN vertices so add that as an offset
        unsigned iFull = poseN+i;  
        const VertexPoseSE3* pPoseVertex = dynamic_cast<const VertexPoseSE3*>(_vertices[iFull]);

        if (pPoseVertex->fixed())  // Jacobian is zero, skip
          continue;
          
        // Make sure the jacobian is the right size
        if (_jacobianOplusTemp[iFull].rows() != 2 || _jacobianOplusTemp[iFull].cols() != 6)
          _jacobianOplusTemp[iFull].resize(2,6);
          
        // Same as before, but checking wrt to the other PoseChainHelper
        if(pPointVertex->_pPoseChainHelper->MoveTogether(*_pPoseChainHelper, i))
        {
          _jacobianOplusTemp[iFull] = JacobianType::Zero(2,6);
          continue;
        }
          
        const TooN::Vector<3> v3Base = pPointVertex->_pPoseChainHelper->_vTransforms[i].first * v3Global;  // The point in the current pose's frame
        const TooN::Vector<4> v4Base = unproject(v3Base);
      
        // For each of six degrees of freedom...
        for(int m=0;m<6;m++)
        {
          // Get the motion of the point in the current pose's frame when the pose changes by one of the degrees of freedom
          // Negative because the point is attached to the frame! Therefore the motion is in the opposite
          // direction of what it would be if the coordinate frame rotated out from under it
          const TooN::Vector<4> v4Motion_Base = -1 * TooN::SE3<>::generator_field(m, v4Base); 
          // Then the motion of the point in the originally measuring camera only depends on the relative rotation between it and the current pose, not the translation
          // Unfortunately, can't compute this rotation in advance since it's calculated from two separate pose chains
          TooN::SE3<> se3CamFromBase = _pPoseChainHelper->_vTransforms.back().first * (pPointVertex->_pPoseChainHelper->_vTransforms[i].first).inverse();
          se3CamFromBase.get_translation() = TooN::Zeros;
          const TooN::Vector<4> v4Motion_Cam = se3CamFromBase * v4Motion_Base;  // CHECK!! GOOD
          
          // Convert to motion on the sphere
          TooN::Vector<2> v2CamSphereMotion;
          v2CamSphereMotion[0] = v3_dTheta * v4Motion_Cam.slice<0,3>();  // theta component
          v2CamSphereMotion[1] = v3_dPhi * v4Motion_Cam.slice<0,3>();  // phi component
          
          // Convert to motion on the image
          TooN::Vector<2> v2Column = _m2CamDerivs * v2CamSphereMotion;
          
          // v2Column is the change in the location of the image point
          // HOWEVER: we want the change in the ERROR! This is the negative of the
          // change in the image point
          
          _jacobianOplusTemp[iFull].col(m)(0) = -1 * v2Column[0];
          _jacobianOplusTemp[iFull].col(m)(1) = -1 * v2Column[1];
        }
      }
      
      // Still need to get jacobian for the point (if not fixed)
      if(!pPointVertex->fixed())
      {
        if (_jacobianOplusTemp.back().rows() != 2 || _jacobianOplusTemp.back().cols() != 3)
          _jacobianOplusTemp.back().resize(2,3);
          
        // The following computations are the same as in VertexRelPoint::oplusImpl
        TooN::Vector<3> v3PointInCam = pPointVertex->estimate();
        double dLength = TooN::norm(v3PointInCam);
        double dRho = 1.0/dLength;
        TooN::Vector<3> v3PointInCamDir = v3PointInCam * dRho;
        
        TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
        double angle = asin(TooN::norm(v3Axis));
        
        TooN::normalize(v3Axis);
        v3Axis *= angle;
        
        TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
        
        // Now we are going to figure out how the image point changes when the point is perturbed
        // as would be done with a call to VertexRelPoint::oplusImpl
        
        TooN::Matrix<3> m3Jac;
        
        // d_beta, rotation about x axis
        m3Jac.T()[0] = Rp.inverse() * (TooN::SO3<>::generator_field(0,Rp*v3PointInCam));
        
        // d_alpha, rotation about y axis
        m3Jac.T()[1] = Rp.inverse() * (TooN::SO3<>::generator_field(1,Rp*v3PointInCam));
        
        // d_rho, change in inverse depth
        m3Jac.T()[2] = -1*v3PointInCam / dRho;
        
        /*
        TooN::Vector<3> v3PointInCam = pPointVertex->estimate();
        TooN::Vector<3> v3PointInCamDir = v3PointInCam;
        TooN::normalize(v3PointInCamDir);
        
        TooN::Vector<3> v3Axis = v3PointInCamDir ^ TooN::makeVector(0,0,1);
        double angle = asin(TooN::norm(v3Axis));
        
        if(angle != 0)
        {
          TooN::normalize(v3Axis);
          v3Axis *= angle;
        }
        
        TooN::SO3<> Rp = TooN::SO3<>::exp(v3Axis);
        TooN::Matrix<3> m3Jac;
        
        // d_beta, rotation about x axis
        m3Jac.T()[0] = Rp.inverse() * (TooN::SO3<>::generator_field(0,Rp*v3PointInCam));
        
        // d_alpha, rotation about y axis
        m3Jac.T()[1] = Rp.inverse() * (TooN::SO3<>::generator_field(1,Rp*v3PointInCam));
        
        // d_rho, change in inverse depth
        m3Jac.T()[2] = -1*v3PointInCam;
        */
        // If you want to use XYZ parameterization for the updates, use this instead
        // Make sure that the same thing is done in VertexRelPoint::oplusImpl
        // The logic behind this is that the perturbations in the XYZ frame are simply the unit vectors
        // rather than the bearing+inverse depth perturbations that we computed in m3Jac above
        // m3Jac = TooN::Identity;
        
        // At this point, we have 3 motion vectors for each of the 3 perturbation dimensions. However,
        // they are defined in the point's source frame. We are interested in the 3 motion vectors in the
        // observing frame, so we need to transform the changes into the observing frame.
        // This depends only on the relative rotation between the frames, not the translation.
        // Unfortunately, can't compute this rotation in advance since it's calculated from two separate pose chains
        TooN::SE3<> se3CamFromSource = _pPoseChainHelper->_vTransforms.back().first * (pPointVertex->_pPoseChainHelper->_vTransforms.back().first).inverse();
        
        // Transform the motion vectors with the found rotation 
        m3Jac = se3CamFromSource.get_rotation().get_matrix() * m3Jac;
        
        // For each of three degrees of freedom...
        for(int m=0;m<3;m++)
        {
          // Get the motion of the point in the camera frame when the position of the point changes by one of the degrees of freedom
          const TooN::Vector<3> v3Motion = m3Jac.T()[m];  // CHECK!! GOOD
          
          // Convert to motion on the sphere
          TooN::Vector<2> v2CamSphereMotion;
          v2CamSphereMotion[0] = v3_dTheta * v3Motion;  // theta component
          v2CamSphereMotion[1] = v3_dPhi * v3Motion;  // phi component
          
          // Convert to motion on the image
          TooN::Vector<2> v2Column = _m2CamDerivs * v2CamSphereMotion;
          
          // v2Column is the change in the location of the image point
          // HOWEVER: we want the change in the ERROR! This is the negative of the
          // change in the image point
          _jacobianOplusTemp.back().col(m)(0) = -1 * v2Column[0];
          _jacobianOplusTemp.back().col(m)(1) = -1 * v2Column[1];
        }
      }
      
      // ============= End Analytical Jacobian ========================
    
      //============ Numerical Jacobian =================
      // If you activate this you NEED to uncomment the appropriate line
      // in VertexRelPoint::estimateInGlobalCartesian() as well as
      // in EdgeChainMeas::computeError() !!
      /*
      const double delta = 1e-9;
      const double scalar = 1.0 / (2*delta);
      ErrorVector errorBak;
      ErrorVector errorBeforeNumeric = _error;

      for (size_t i = 0; i < _vertices.size(); ++i) { 
        //Xi - estimate the jacobian numerically
        g2o::OptimizableGraph::Vertex* vi = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[i]);

        if (vi->fixed())
          continue;

        int vi_dim = vi->dimension();
        double add_vi[vi_dim];
        std::fill(add_vi, add_vi + vi_dim, 0.0);
        assert(_dimension >= 0);
        assert(_jacobianOplus[i].rows() == _dimension && _jacobianOplus[i].cols() == vi_dim && "jacobian cache dimension does not match");
          _jacobianOplus[i].resize(_dimension, vi_dim);
        // add small step along the unit vector in each dimension
        for (int d = 0; d < vi_dim; ++d) {
          vi->push();
          add_vi[d] = delta;
          vi->oplus(add_vi);
          computeError();
          errorBak = _error;
          vi->pop();
          vi->push();
          add_vi[d] = -delta;
          vi->oplus(add_vi);
          computeError();
          errorBak -= _error;
          vi->pop();
          add_vi[d] = 0.0;

          _jacobianOplus[i].col(d) = scalar * errorBak;
        } // end dimension
        
        //std::cerr<<"Jacobian of measurement "<<_nID<<" wrt vertex "<<i<<std::endl;
        //std::cerr<<"Numerical: "<<std::endl<<_jacobianOplus[i]<<std::endl;
        //std::cerr<<"Analytic: "<<std::endl<<_jacobianOplusTemp[i]<<std::endl;
        //std::cerr<<"Diff: "<<std::endl<<_jacobianOplus[i] - _jacobianOplusTemp[i]<<std::endl;
        
        //std::cout<<"Jacobian of measurement "<<_nID<<" wrt vertex "<<i<<std::endl;
        //std::cout<<_jacobianOplus[i]<<std::endl;
      }
      _error = errorBeforeNumeric;
      */
      // ============= End Numerical Jacobian ========================
      
      #ifdef G2O_OPENMP
        for (int i = (int)(_vertices.size()) - 1; i >= 0; --i) {
          g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[i]);
          v->unlockQuadraticForm();
        }
      #endif
      
    }
    
    virtual bool read(std::istream& is)
    {
      TooN::Vector<2> v2Meas;
      is >> v2Meas;
      setMeasurement(v2Meas);
      
      for (int i=0; i<6; i++)
        for (int j=i; j<6; j++) {
          is >> information()(i,j);
          if (i!=j)
            information()(j,i) = information()(i,j);
        }
      return true;
    }
    
    virtual bool write(std::ostream& os) const
    {
      os << measurement();
        
      for (int i=0; i<6; i++)
        for (int j=i; j<6; j++){
          os << " " <<  information()(i,j);
        }
      return os.good();
    }
  
    std::string _camName;
    int _nID;
    PoseChainHelper* _pPoseChainHelper;
  
  protected:
    TaylorCamera& _camera;
    TooN::Matrix<2> _m2CamDerivs;
    
};

/** @brief Holds the data needed for robustification. 
 * 
 *  This is a separate class from RobustKernelAdaptive because all measurements will have a pointer to their own
 *  instance of RobustKernelAdaptive, but all of them need access to the same robust statistics. */
class RobustKernelData
{
  public:
    /** @param pOptimizer The optimizer we're working with, needed to access the measurements (edges) from which robust statistics are computed
     *  @param dMinSigmaSquared The minimum value of the robust sigma squared (needed to prevent the case where all measurements are pretty good but the worst ones are still eliminated) */
    RobustKernelData(g2o::SparseOptimizer* pOptimizer, double dMinSigmaSquared)
    {
      _pOptimizer = pOptimizer;
      _dMinSigmaSquared = dMinSigmaSquared;
      _bRecomputeSigmaSquared = false;
    }
    
    /// Sets a flag to indicate recomputation of statistics is necessary
    void RecomputeNextTime(){ _bRecomputeSigmaSquared = true; }
    
    /// Return recompute flag value
    bool NeedRecompute(){ return _bRecomputeSigmaSquared; }
    
    /// Perform recomputation of robust statistics
    void RecomputeNow()
    {
      ROS_ASSERT(_bRecomputeSigmaSquared);  // make sure someone has asked us to do this in the right way
      _bRecomputeSigmaSquared = false;
      
      std::vector<double> vErrorSquared;        
      for (g2o::OptimizableGraph::EdgeContainer::const_iterator edge_it = _pOptimizer->activeEdges().begin(); edge_it != _pOptimizer->activeEdges().end(); edge_it++) 
      {
        EdgeChainMeas* pMeas = dynamic_cast<EdgeChainMeas*>(*edge_it);
        vErrorSquared.push_back(fabs(pMeas->chi2()));  // take absolute value because chi2 will be set negative for fixed points
      }
      
      double dSigmaSquared = Huber::FindSigmaSquared(vErrorSquared);
      
      _dSigmaSquared = dSigmaSquared;
      _dSigmaSquaredLimited = _dSigmaSquared;
          
      if(_dSigmaSquaredLimited < _dMinSigmaSquared)
        _dSigmaSquaredLimited = _dMinSigmaSquared;
        
      _dSigmaLimited = sqrt(_dSigmaSquaredLimited);
      
      ROS_DEBUG_STREAM("Sigma squared (clamped): "<<_dSigmaSquaredLimited<<" Min sigma squared: "<<_dMinSigmaSquared);
    }
    
    /// Return the raw sigma squared value
    double GetSigmaSquared(){ return _dSigmaSquared; }
    
    /// Return the limited sigma squared value
    double GetSigmaSquaredLimited(){ return _dSigmaSquaredLimited; }
    
    /// Return the limited sigma value
    double GetSigmaLimited(){ return _dSigmaLimited; }
    
  protected:
    
    g2o::SparseOptimizer* _pOptimizer;   ///< Pointer to the optimizer
    double _dMinSigmaSquared;           ///< Minimum allowed value of sigma squared limited
    double _dSigmaSquared;              ///< The last computed value of sigma squared
    double _dSigmaSquaredLimited;       ///< The last computed value of sigma squared, or the minimum allowed, whichever is larger
    double _dSigmaLimited;              ///< Square root of the limited sigma squared
    bool _bRecomputeSigmaSquared;       ///< Do we have to recompute robust statistics?
};

/** @brief A robust kernel that uses a changing value of sigma, which is computed from the current
 *  errors in the graph.
 * 
 *  Uses the robust statistics computed by RobustKernelData instead of recomputing its own
 *  data, since each measurement will have a pointer to a unique RobustKernelAdaptive object. */
class RobustKernelAdaptive : public g2o::RobustKernel
{
  public:
    /** @param pData Pointer to the RobustKernelData object used to store robust statistics */
    RobustKernelAdaptive(RobustKernelData* pData)
    {
      _pData = pData;
    }
    
    /** @brief Called by the activeRobustChi2() function of the optimizer to compute a robustified value of e2
     *  @param e2 The chi2 value being robustified
     *  @param [out] rho Vector of robustified e2, its first derivative, and its second derivative */
    virtual void robustify(double e2, Eigen::Vector3d& rho) const
    {
      if(_pData->NeedRecompute())
      {
        _pData->RecomputeNow();
      }
      
      double dSigmaSquared = _pData->GetSigmaSquaredLimited();
      double dSigma = _pData->GetSigmaLimited();
      
      // This next section is taken from RobustKernelHuber
      // Keep in mind, e2 will be negative for any points that are fixed, which 
      // means they get a weight of 1.0.
      if (e2 <= dSigmaSquared) 
      { // inlier
        rho[0] = fabs(e2);
        rho[1] = 1.;
        rho[2] = 0.;
      } 
      else 
      { // outlier
        double e = sqrt(e2); // absolut value of the error
        rho[0] = 2 * dSigma * e - dSigmaSquared; // rho(e)   = 2 * sigma * e - sigma^2
        rho[1] = dSigma / e;        // rho'(e)  = sigma / e
        rho[2] = - 0.5 * rho[1] / e2;    // rho''(e) = -1 / (2*e2 ^ (3/2)) = -1/2 * (sigma/e2) / e2
      }
    }
  
  protected: 
    RobustKernelData* _pData;  ///< Pointer to the data object
};

/// Sets the update flag of the RobustKernelData object, should be added as a pre-iteration action to the optimizer
class UpdateSigmaSquaredAction : public g2o::HyperGraphAction
{
  public:
    /** @param pData Pointer to the RobustKernelData object */
    UpdateSigmaSquaredAction(RobustKernelData* pData)
    : _pData(pData)
    { }
    
    /// This function is called by the optimizer at the appropriate time
    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0)
    {
      _pData->RecomputeNextTime();
      return this;
    }
    
  protected:
    RobustKernelData* _pData;   ///< Pointer to the data object
  
};

/// Updates the pose chain helpers, which calculate various inter-link transforms. Should be added as a compute errors action to the optimizer
class UpdateHelpersAction : public g2o::HyperGraphAction
{
  public:
    /** @param helpers A map of pose chain vectors => helper object pointers, all helper objects are kept in there */
    UpdateHelpersAction(HelperMap& helpers)
    : _mHelpers(helpers)
    { }
  
    /// This function is called by the optimizer at the appropriate time
    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0)
    {
      // Just loop over all the helper objects and call their update function
      for(HelperMap::iterator it = _mHelpers.begin(); it != _mHelpers.end(); ++it)
      {
        it->second->UpdateTransforms();
      }
      
      return this;
    }
  
  protected:
    HelperMap& _mHelpers;  ///< Reference to the helper map
};

class UpdateTotalIterationsAction : public g2o::HyperGraphAction
{
  public:
  
    UpdateTotalIterationsAction(g2o::OptimizationAlgorithmLevenberg* pAlgorithm, int* pTotalIterations)
    : _pAlgorithm(pAlgorithm)
    , _pTotalIterations(pTotalIterations)
    { }
    
    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0)
    {
      *_pTotalIterations += _pAlgorithm->levenbergIteration();
      
      return this;
    }
     
  protected:
  
    g2o::OptimizationAlgorithmLevenberg* _pAlgorithm;
    int* _pTotalIterations;
};


/** @brief Checks to see if optimization converged by comparing the RMS value of the last update vector to a given threshold.
 * 
 *  If converged, sets the abort flag to true which is also accessed by the optimizer, thus ending the iterations. Should be
 *  added as a post-iteration action. */
class CheckConvergedUpdateMagAction : public g2o::HyperGraphAction
{
  public:
    /** @param pAlgorithm Pointer to the optimization algorithm. Unfortunately, we can't access the solver's update vector through 
     *                     the graph passed to operator(), so this is needed.
        @param dLimit The update vector RMS threshold */
    CheckConvergedUpdateMagAction(g2o::OptimizationAlgorithmWithHessian* pAlgorithm, double dLimit) 
    : _bConverged(false)
    , _dLimit(dLimit)
    , _pAbortFlag(NULL)
    , _pAlgorithm(pAlgorithm)
    {}
    
    /// Give this action access to the global abort flag
    void SetAbortFlag(bool* pAbortFlag)
    {
      _bConverged = false;
      _pAbortFlag = pAbortFlag;
    }
    
    /// Has the optimizer converged, as per the criteria given above?
    bool IsConverged()
    {
      return _bConverged;
    }
    
    /// Set the internal state to not converged
    void SetNotConverged()
    {
      _bConverged = false;
    }
    
    /// This function is called by the optimizer at the appropriate time
    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0)
    {
      ROS_ASSERT(_pAbortFlag);  // Make sure we've been given the abort flag pointer
      
      const double* pUpdate = _pAlgorithm->solver()->x();  // The last update vector
      int nUpdateSize = _pAlgorithm->solver()->vectorSize();
      
      double dSumSquaredUpdate = 0;
      for(int i=0; i < nUpdateSize; ++i)
        dSumSquaredUpdate += pUpdate[i] * pUpdate[i];
        
      double dRMS = sqrt(dSumSquaredUpdate/nUpdateSize);
      
      ROS_DEBUG_STREAM("===== Update size: "<<nUpdateSize<<" magnitude: "<<dSumSquaredUpdate<<" RMS: "<<dRMS<<" Limit: "<<_dLimit<<" =========");
      
      if(dRMS < _dLimit)
      {
        ROS_INFO_STREAM("Update RMS "<<dRMS<<" smaller than limit "<<_dLimit<<", setting abort flag");
        _bConverged = true;
        *_pAbortFlag = true;
      }
      
      // mostly for debug
      if(!std::isfinite(dRMS))
      {
        ROS_ERROR("Update RMS not finite! Vector: ");
        for(int i=0; i < nUpdateSize; ++i)
          ROS_ERROR_STREAM(pUpdate[i]);
        /*
        for (g2o::OptimizableGraph::EdgeContainer::const_iterator edge_it = pOptimizer->activeEdges().begin(); edge_it != pOptimizer->activeEdges().end(); edge_it++) 
        {
          EdgeChainMeas* pMeas = dynamic_cast<EdgeChainMeas*>(*edge_it);
          pMeas->printJacobians();
        }
        */
      }
      
      return this;
    }
      
  protected:
    bool _bConverged;     ///< Have we detected convergence?
    double _dLimit;       ///< The limit on update vector RMS below which we're converged
    bool* _pAbortFlag;    ///< Pointer to the flag that triggers abortion of BA
    g2o::OptimizationAlgorithmWithHessian* _pAlgorithm;   ///< Pointer to the optimization algorithm so that we can get solver's udpate vector
};

/** @brief Checks to see if optimization converged by comparing the change in residual error to a given threshold.
 * 
 *  If converged, sets the abort flag to true which is also accessed by the optimizer, thus ending the iterations. Should
 *  be added as a post-iteration action. */
class CheckConvergedResidualAction : public g2o::HyperGraphAction
{
  public:
    /** @param dPercentLimit The threshold for the change in residual error below which optimization is considered converged */
    CheckConvergedResidualAction(double dPercentLimit) 
    : _bConverged(false)
    , _dPercentLimit(dPercentLimit)
    , _pAbortFlag(NULL)
    , _dLastChi2(std::numeric_limits<double>::max())
    {}
    
    /// Give action access to the global abort flag
    void SetAbortFlag(bool* pAbortFlag)
    {
      _bConverged = false;
      _pAbortFlag = pAbortFlag;
    }
    
    /// Have we converged?
    bool IsConverged()
    {
      return _bConverged;
    }
    
    /// Set internal flag to not converged
    void SetNotConverged()
    {
      _bConverged = false;
    }
    
    /// This function is called by the optimizer at the appropriate time
    virtual HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters = 0)
    {
      ROS_ASSERT(_pAbortFlag);  // make sure we've been given the abort flag
      const g2o::SparseOptimizer* pOptimizer = dynamic_cast<const g2o::SparseOptimizer*>(graph);
      
      double dCurrChi2 = pOptimizer->activeRobustChi2();
      double dPctChange = (_dLastChi2 - dCurrChi2)/_dLastChi2;
      
      ROS_DEBUG_STREAM("last chi2: "<<_dLastChi2<<" new chi2: "<<dCurrChi2<<" % change: "<<dPctChange<<" % limit: "<<_dPercentLimit);
      
      if(dPctChange >= 0 && dPctChange <= _dPercentLimit)
      {
        ROS_INFO_STREAM("Percent change "<<dPctChange<<" below limit "<<_dPercentLimit<<", stopping.");
  
        _bConverged = true;
        *_pAbortFlag = true;
      }
      else if(dCurrChi2 == 0)
      {
        ROS_INFO_STREAM("Chi2 at zero, stopping.");
  
        _bConverged = true;
        *_pAbortFlag = true;
      }
      
      _dLastChi2 = dCurrChi2;
      return this;
    }
      
  protected:
    bool _bConverged;         ///< Have we converged?
    double _dPercentLimit;   ///< The convergence threshold 
    bool* _pAbortFlag;        ///< Pointer to the global BA abort flag
    double _dLastChi2;       ///< Last value of chi2
    
};

using namespace TooN;
using namespace g2o;

// Static members
int ChainBundle::snMaxIterations = 100; // 100
int ChainBundle::snMaxTrialsAfterFailure = 100;
double ChainBundle::sdUpdatePercentConvergenceLimit = 1e-10;
double ChainBundle::sdUpdateRMSConvergenceLimit = 1e-10; // 1e-10
double ChainBundle::sdMinMEstimatorSigma = 0.5;//0.4;

// Constructor takes camera models
ChainBundle::ChainBundle(TaylorCameraMap& cameraModels, bool bUseRobust, bool bUseTukey, bool bVerbose)
: mmCameraModels(cameraModels)
, mbUseRobust(bUseRobust)
, mbUseTukey(bUseTukey)
, mbVerbose(bVerbose)
{
  mnCurrId = 1;
  
  mpOptimizer = new SparseOptimizer;
  mpRobustKernelData = new RobustKernelData(mpOptimizer, ChainBundle::sdMinMEstimatorSigma * ChainBundle::sdMinMEstimatorSigma);
  
  // Tried to use BlockSolver_6_3 with marginalized points, but it has all kinds of problems in certain cases
  // (all points fixed, all poses fixed, getting the point covariance)
  // So use BlockSolverX and non-marginalized points for now
  
  //BlockSolver_6_3::LinearSolverType* pLinearSolver = new LinearSolverCholmod<BlockSolver_6_3::PoseMatrixType>();
  //BlockSolver_6_3* pSolver = new BlockSolver_6_3(pLinearSolver);
  BlockSolverX::LinearSolverType* pLinearSolver = new LinearSolverCholmod<BlockSolverX::PoseMatrixType>();
  BlockSolverX* pSolver = new BlockSolverX(pLinearSolver);
  OptimizationAlgorithmLevenberg* pAlgorithm = new OptimizationAlgorithmLevenberg(pSolver);
  
  pAlgorithm->setMaxTrialsAfterFailure(ChainBundle::snMaxTrialsAfterFailure);
  mpOptimizer->setAlgorithm(pAlgorithm);
  
  // Create and add the various actions
  mpUpdateSigmaSquaredAction = new UpdateSigmaSquaredAction(mpRobustKernelData);
  mpOptimizer->addPreIterationAction(mpUpdateSigmaSquaredAction);
  
  mpUpdateHelpersAction = new UpdateHelpersAction(mmHelpers);
  mpOptimizer->addComputeErrorAction(mpUpdateHelpersAction);
  
  mpConvergedUpdateMagAction = new CheckConvergedUpdateMagAction(pAlgorithm, ChainBundle::sdUpdateRMSConvergenceLimit);
  mpOptimizer->addPostIterationAction(mpConvergedUpdateMagAction);
  
  mpConvergedResidualAction = new CheckConvergedResidualAction(ChainBundle::sdUpdatePercentConvergenceLimit);
  mpOptimizer->addPostIterationAction(mpConvergedResidualAction);
  
  mpUpdateTotalIterationsAction = new UpdateTotalIterationsAction(pAlgorithm, &mnTotalIterations);
  mpOptimizer->addPostIterationAction(mpUpdateTotalIterationsAction);
  
  mdLastMaxCov = std::numeric_limits<double>::max();
  mbConverged = false;
}

ChainBundle::~ChainBundle()
{
  delete mpOptimizer;
  delete mpUpdateSigmaSquaredAction;
  delete mpUpdateHelpersAction;
  
  delete mpConvergedUpdateMagAction;
  delete mpConvergedResidualAction;
  
  for(HelperMap::iterator it = mmHelpers.begin(); it != mmHelpers.end(); ++it)
    delete it->second;
  
}

// Add a pose to the system, return value is the bundle adjuster's ID for the pose
int ChainBundle::AddPose(SE3<> se3PoseFromRef, bool bFixed)
{
  VertexPoseSE3* pPoseVertex =  new VertexPoseSE3;
  pPoseVertex->setId(mnCurrId++);
  pPoseVertex->setEstimate(se3PoseFromRef);
  pPoseVertex->setFixed(bFixed);
  
  mpOptimizer->addVertex(pPoseVertex);
  
 return pPoseVertex->id();
}

// Add a map point to the system, return value is the bundle adjuster's ID for the point
int ChainBundle::AddPoint(Vector<3> v3PointInCam, std::vector<int> vCams, bool bFixed)
{
  int N = vCams.size();
  VertexRelPoint* pPointVertex =  new VertexRelPoint;
  pPointVertex->setId(mnCurrId++);
  pPointVertex->setEstimate(v3PointInCam);
  pPointVertex->setFixed(bFixed);
  pPointVertex->setMarginalized(false);  // see comment in ChainBundle constructor about marginalization
  
  if(!mmHelpers.count(vCams)) // helper not found, so make a new one
    mmHelpers.insert(std::make_pair(vCams, new PoseChainHelper));
   
  PoseChainHelper* pHelper = mmHelpers[vCams];
  
  if((int)pHelper->_vpVertices.size() != N)  // helper hasn't been set up yet
  {
    pHelper->_vpVertices.resize(N);  // only needs to keep track of the N poses
    for(int i=0; i < N; ++i)
      pHelper->_vpVertices[i] = dynamic_cast<VertexPoseSE3*>(mpOptimizer->vertex(vCams[i]));
  }
  
  pPointVertex->_pPoseChainHelper = pHelper;  // link the measurement and the helper
  mpOptimizer->addVertex(pPointVertex);
  
  return pPointVertex->id();
}

// Add a measurement of one point from the end of one pose chain
void ChainBundle::AddMeas(std::vector<int> vCams, int nPointIdx, Vector<2> v2Pos, double dNoiseSigmaSquared, std::string cameraName)
{
  int N = vCams.size();
  EdgeChainMeas* pMeas = new EdgeChainMeas(mmCameraModels[cameraName], cameraName);
  pMeas->setMeasurement(v2Pos);
  pMeas->information().setIdentity();     
  pMeas->information() *= 1/sqrt(dNoiseSigmaSquared); // add noise here
  
  if(!mmHelpers.count(vCams)) // helper not found, so make a new one
    mmHelpers.insert(std::make_pair(vCams, new PoseChainHelper));
   
  PoseChainHelper* pHelper = mmHelpers[vCams];
  
  const VertexRelPoint* pPoint = dynamic_cast<const VertexRelPoint*>(mpOptimizer->vertex(nPointIdx));
  ROS_ASSERT(pPoint->_pPoseChainHelper);
  int pointN = pPoint->_pPoseChainHelper->_vpVertices.size();
  
  // Will connect totalN vertices: N poses from the measurement camera, the pointN poses that define the source of that point, and the 1 point
  int totalN = N + pointN + 1;
  pMeas->resize(totalN);
  
  for(int i=0; i < N; ++i)
    pMeas->vertices()[i] = mpOptimizer->vertex(vCams[i]);  // the measurement camera poses
    
  for(int i=N, j=0; i < N+pointN && j < pointN; ++i, ++j)
    pMeas->vertices()[i] = pPoint->_pPoseChainHelper->_vpVertices[j];  // the point source poses
    
  pMeas->vertices().back() = mpOptimizer->vertex(nPointIdx);  // finally the point
  
  if((int)pHelper->_vpVertices.size() != N)  // helper hasn't been set up yet
  {
    pHelper->_vpVertices.resize(N);  // only needs to keep track of the N measurement camera poses
    for(int i=0; i < N; ++i)
      pHelper->_vpVertices[i] = dynamic_cast<VertexPoseSE3*>(mpOptimizer->vertex(vCams[i]));
  }
    
  if(mbUseRobust)
    pMeas->setRobustKernel(new RobustKernelAdaptive(mpRobustKernelData));   // uses Huber robust error
    
  pMeas->_pPoseChainHelper = pHelper;  // link the measurement and the helper
  
  mpOptimizer->addEdge(pMeas);
}

// Initialize internal variables and the g2o optimizer
void ChainBundle::Initialize()
{ 
  mvOutlierMeasurementIdx.clear();
    
  mpOptimizer->setVerbose(mbVerbose);
  mpConvergedUpdateMagAction->SetNotConverged();
  mpConvergedResidualAction->SetNotConverged();
    
  ros::WallTime start = ros::WallTime::now();
  mpOptimizer->initializeOptimization();
  ROS_INFO_STREAM("ChainBundle: Initialized in "<<ros::WallTime::now() - start<<" seconds");
  
  mbHitMaxIterations = false;
  mbConverged = false;
}

// Perform bundle adjustment. pAbortSignal points to a signal bool 
// which mapmaker will set to high if another keyframe is incoming
// and bundle adjustment needs to be aborted.
// Returns number of accepted iterations if all good, negative 
// value for big error.
int ChainBundle::Compute(bool *pAbortSignal, int nNumIter, double dUserLambda)
{
  Initialize();
  
  mpOptimizer->setForceStopFlag(pAbortSignal);
  dynamic_cast<OptimizationAlgorithmLevenberg*>(mpOptimizer->solver())->setUserLambdaInit(dUserLambda);
  
  // The following two actions need access to the abort signal so they can trigger an abort if necessary
  mpConvergedUpdateMagAction->SetAbortFlag(pAbortSignal);
  mpConvergedResidualAction->SetAbortFlag(pAbortSignal);
  
  // Mostly for debugging, display the initial error
  {
    mpOptimizer->computeActiveErrors();
    mpRobustKernelData->RecomputeNextTime();
    double dChi2 = mpOptimizer->activeRobustChi2();
    double dSigmaSquaredLimited = mpRobustKernelData->GetSigmaSquaredLimited();
    ROS_INFO_STREAM("------- ComputeStep BEFORE optimization, chi2: "<<dChi2<<" sigma squared limited: "<<dSigmaSquaredLimited);
  }
  
  ros::WallTime start = ros::WallTime::now();
  mnTotalIterations = 0;
  int nCounter = mpOptimizer->optimize(nNumIter);
  ROS_INFO_STREAM("Optimization took "<<ros::WallTime::now() - start<<" seconds, nCounter: "<<nCounter);
  ROS_DEBUG_STREAM("Done ComputeStep, "<<nCounter<<" iterations accepted out of a max of "<<nNumIter);
  
  mbHitMaxIterations = (nCounter == nNumIter);
  
  if(nCounter > 0)
  {
    ROS_ASSERT(mnTotalIterations > 0);
  }
  
  // Mostly for debugging, display the final error
  {
    mpOptimizer->computeActiveErrors();
    mpRobustKernelData->RecomputeNextTime();
    double dChi2 = mpOptimizer->activeRobustChi2();
    double dSigmaSquaredLimited = mpRobustKernelData->GetSigmaSquaredLimited();
    ROS_INFO_STREAM("+++++++ ComputeStep AFTER optimization, chi2: "<<dChi2<<" sigma squared limited: "<<dSigmaSquaredLimited);
  }
  
  mbConverged = (mpConvergedUpdateMagAction->IsConverged() || mpConvergedResidualAction->IsConverged());  
  
  if(mbHitMaxIterations)
    ROS_DEBUG("Hit max iterations");
    
  if(mbConverged)
    ROS_DEBUG("Converged");
    
  bool bExternalAbort = false;
  if(*pAbortSignal && !mbConverged)
  {
    bExternalAbort = true;
    ROS_DEBUG("Externally Aborted");
  }
  
  if(nCounter == 0 && !bExternalAbort)  // we have a problem
    return -1;
  
  if(nCounter == 0 && *pAbortSignal)  // didn't actually take any steps, so we won't be able to do any of the tasks that follow, so just get out
    return 0; 
  
  if(mbUseTukey)  // Need to calculate Tukey sigma and weight computation to determine outliers
  {
    std::vector<double> vErrorSquared;
    for (g2o::OptimizableGraph::EdgeContainer::const_iterator edge_it = mpOptimizer->activeEdges().begin(); edge_it != mpOptimizer->activeEdges().end(); edge_it++) 
    {
      vErrorSquared.push_back(fabs((*edge_it)->chi2()));  // keeping in mind that negative chi2 means fixed point
    }
  
    double dSigmaSquared = Tukey::FindSigmaSquared(vErrorSquared);
    double dSigmaSquaredLimited = dSigmaSquared;
    double dMinSigmaSquared = ChainBundle::sdMinMEstimatorSigma * ChainBundle::sdMinMEstimatorSigma;

    // Initially the median error might be very small - set a minimum
    // value so that good measurements don't get erased!
    if(dSigmaSquaredLimited < dMinSigmaSquared)
      dSigmaSquaredLimited = dMinSigmaSquared;
    
    for (g2o::OptimizableGraph::EdgeContainer::const_iterator edge_it = mpOptimizer->activeEdges().begin(); edge_it != mpOptimizer->activeEdges().end(); edge_it++) 
    {
      EdgeChainMeas* pMeas = dynamic_cast<EdgeChainMeas*>(*edge_it);
      
      double dWeight = Tukey::Weight(fabs(pMeas->chi2()), dSigmaSquaredLimited);
      
      if(dWeight == 0)
      {
        int nPointId = pMeas->vertices().back()->id();
        int nBaseId = pMeas->vertices().front()->id();
        
        mvOutlierMeasurementIdx.push_back(std::make_tuple(nPointId, nBaseId, pMeas->_camName));
      }
    }
  }
  
  // Now we're goint to get the max covariance of the point depths
  // Gather the point vertices
  g2o::OptimizableGraph::VertexContainer vPointVertices;
  int nNumPoses = 0;
  for(g2o::OptimizableGraph::VertexContainer::const_iterator vertex_it = mpOptimizer->activeVertices().begin(); vertex_it != mpOptimizer->activeVertices().end(); vertex_it++)
  {
    if((*vertex_it)->fixed())
      continue;
    
    int dim = (*vertex_it)->dimension();
    if(dim == 3)
      vPointVertices.push_back(*vertex_it);
    else
      nNumPoses++;
  }
  
  SparseBlockMatrix<MatrixXd> spinv;  // This will hold the covariance matrices
  
  if(nNumPoses < 3 && mpOptimizer->computeMarginals(spinv, vPointVertices))
  {
    ROS_INFO("computeMarginals() success!");
    std::vector<double> vCov22;
    
    for(unsigned i = 0; i < vPointVertices.size(); ++i)
    {
      // Look at the (2,2) entry of each covariance matrix, which indicates
      // sensitivity in radial direction
      vCov22.push_back(spinv.block(i,i)->col(2)(2));
    }
    
    if(vCov22.size() > 0)
    {
      std::sort(vCov22.begin(), vCov22.end());
      double median = vCov22[vCov22.size()/2];
      ROS_INFO_STREAM("Point Cov 2,2 min: "<<*vCov22.begin()<<" max: "<<*vCov22.rbegin()<<" median: "<<median);
      mdLastMaxCov = median;
    }
    else
    {
      ROS_WARN("Point Cov: none found because all fixed");
      mdLastMaxCov = std::numeric_limits<double>::max();
    }
  }
  else
  {
    ROS_WARN("computeMarginals() failed!");
    mdLastMaxCov = 0; //std::numeric_limits<double>::max();
  }
    
  return nCounter;
}

Vector<3> ChainBundle::GetPoint(int n)
{
  const VertexRelPoint* pPointVertex = dynamic_cast<const VertexRelPoint*>(mpOptimizer->vertex(n));
  return pPointVertex->estimate();
}

SE3<> ChainBundle::GetPose(int n)
{
  const VertexPoseSE3* pPoseVertex = dynamic_cast<const VertexPoseSE3*>(mpOptimizer->vertex(n));
  return pPoseVertex->estimate();
}

std::vector<std::tuple<int, int, std::string> > ChainBundle::GetOutlierMeasurements()
{
  return mvOutlierMeasurementIdx;
}

double ChainBundle::GetSigmaSquared()
{ 
  ROS_ASSERT(mpRobustKernelData);
  return mpRobustKernelData->GetSigmaSquared(); 
}

double ChainBundle::GetMeanChiSquared()
{
  ROS_ASSERT(mpOptimizer);
  ROS_ASSERT(mpOptimizer->activeEdges().size() > 0);
  return mpOptimizer->activeRobustChi2() / mpOptimizer->activeEdges().size();
}

double ChainBundle::GetLambda()
{
  ROS_ASSERT(mpOptimizer && mpOptimizer->solver()); 
  return dynamic_cast<OptimizationAlgorithmLevenberg*>(mpOptimizer->solver())->currentLambda();
}





