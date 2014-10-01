// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_SPARSE_OPTIMIZER_INCREMENTAL_COVARIANCE_H
#define G2O_SPARSE_OPTIMIZER_INCREMENTAL_COVARIANCE_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

namespace g2o {

  // Forward declare, this is defined in linear_solver_cholmod.h
  struct CholmodExt; 

  class SparseOptimizerIncrementalCovariance : public SparseOptimizer
  {
    public:
      SparseOptimizerIncrementalCovariance();
      ~SparseOptimizerIncrementalCovariance();
      
      void setAlgorithm(OptimizationAlgorithm* algorithm) = delete;  // don't want user to be able to set algorithm since we manage that internally

      int optimize(int iterations, bool online = false);
      
      bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const std::vector<std::pair<int, int> >& blockIndices);
      bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const Vertex* vertex);
      bool computeMarginals(SparseBlockMatrix<MatrixXd>& spinv, const VertexContainer& vertices);

      bool computeUpdatedCovariance(HyperGraph::EdgeSet& eset, const VertexContainer& vertices, SparseBlockMatrix<MatrixXd>& spinv);

    protected:
      SparseBlockMatrix<MatrixXd> _updateMat;
      cholmod_common _cholmodCommon;
      CholmodExt* _cholmodSparse;
      cholmod_factor* _cholmodFactor;
      cholmod_triplet* _permutedUpdate;
      cholmod_triplet* _permutedDowndate;
      cholmod_factor* _L;
      
      LinearSolverCholmod<BlockSolverX::PoseMatrixType>* _linearSolver;
      BlockSolverX* _blockSolver;

      HyperGraph::VertexSet _touchedVertices;
      Eigen::VectorXi _perm;

      Eigen::VectorXi _tripletWorkspace;
      CholmodExt* _permutedUpdateAsSparse;
      CholmodExt* _permutedDowndateAsSparse;

      bool computeCholeskyUpdate();
      void convertTripletUpdateToSparse(cholmod_triplet* update, CholmodExt* updateAsSparse);
  };

}

#endif
