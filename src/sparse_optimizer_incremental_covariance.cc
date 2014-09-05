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

#include <mcptam/sparse_optimizer_incremental_covariance.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace std;

namespace g2o {

  namespace {
    /**
     * \brief backing up some information about the vertex
     */
    struct VertexBackup
    {
      int hessianIndex;
      OptimizableGraph::Vertex* vertex;
      double* hessianData;
      bool operator<(const VertexBackup& other) const
      {
        return hessianIndex < other.hessianIndex;
      }
    };
  }
  
  bool writeOctave(cholmod_sparse* sparseMatrix, std::string filename)
  {
    std::string name = filename;
    std::string::size_type lastDot = name.find_last_of('.');
    if (lastDot != std::string::npos) 
      name = name.substr(0, lastDot);
      
    std::ofstream fout(filename);
    fout << "# name: " << name << std::endl;
    fout << "# type: sparse matrix" << std::endl;
    fout << "# nnz: " << sparseMatrix->nzmax << std::endl;
    fout << "# rows: " << sparseMatrix->nrow << std::endl;
    fout << "# columns: " << sparseMatrix->ncol << std::endl;
    fout << std::setprecision(9) << std::fixed << std::endl;
    
    int* Ap = (int*)sparseMatrix->p;
    int* Ai = (int*)sparseMatrix->i;
    double* Ax = (double*)sparseMatrix->x;
    
    for (size_t c = 0; c < sparseMatrix->ncol; ++c) 
    {
      const int& rbeg = Ap[c];
      const int& rend = Ap[c+1];
      
      for (int j = rbeg; j < rend; j++) {
        const int& r = Ai[j];
        const double& val = Ax[j];

        fout << r+1 << " " << c+1 << " " << val << std::endl;
      }
    }
    
    return fout.good();
  }

  SparseOptimizerIncrementalCovariance::SparseOptimizerIncrementalCovariance()
  {
    // Set up algorithms internally
    _linearSolver = new LinearSolverCholmod<BlockSolverX::PoseMatrixType>(true, 6);  // additional space for a pose vertex (dim = 6)
    _blockSolver = new BlockSolverX(_linearSolver);
    OptimizationAlgorithmLevenberg* pAlgorithm = new OptimizationAlgorithmLevenberg(_blockSolver);
    
    // SparseOptimizer's destructor will free pAlgorithm, which will free _blockSolver, which will free _linearSolver
    SparseOptimizer::setAlgorithm(pAlgorithm);
    
    // Cholmod stuff for incremental updating
    _cholmodSparse = new CholmodExt();
    _cholmodFactor = 0;
    cholmod_start(&_cholmodCommon);

    // setup ordering strategy to not permute the matrix
    _cholmodCommon.nmethods = 1 ;
    _cholmodCommon.method[0].ordering = CHOLMOD_NATURAL;
    _cholmodCommon.postorder = 0;
    _cholmodCommon.supernodal = CHOLMOD_SIMPLICIAL;

    _permutedUpdate = cholmod_allocate_triplet(1000, 1000, 1024, 0, CHOLMOD_REAL, &_cholmodCommon);
    _L = 0;
    _cholmodFactor = 0;

    _permutedUpdateAsSparse = new CholmodExt;
  }

  SparseOptimizerIncrementalCovariance::~SparseOptimizerIncrementalCovariance()
  {
    delete _permutedUpdateAsSparse;
    _updateMat.clear(true);
    delete _cholmodSparse;
    if (_cholmodFactor) {
      cholmod_free_factor(&_cholmodFactor, &_cholmodCommon);
      _cholmodFactor = 0;
    }
    cholmod_free_triplet(&_permutedUpdate, &_cholmodCommon);
    cholmod_finish(&_cholmodCommon);
  }

  int SparseOptimizerIncrementalCovariance::optimize(int iterations, bool online)
  {
    int ret;
    
    if(iterations > 0)
    {
      ret = SparseOptimizer::optimize(iterations, online);
    }
    else
    {
      _algorithm->init(online);
      _blockSolver->buildStructure();
      computeActiveErrors();
      activeRobustChi2();
      _blockSolver->buildSystem();
      _blockSolver->solve();
      
      ret = 0;
    }
    
    return ret;
  }

  bool SparseOptimizerIncrementalCovariance::computeUpdatedCovariance(HyperGraph::EdgeSet& eset, const VertexContainer& vertices, SparseBlockMatrix<MatrixXd>& spinv)
  {
    // get the current cholesky factor along with the permutation
    _L = _linearSolver->L();
        
    cerr << "L size: " << _L->n << "," << _L->n << endl;
    
    if (_perm.size() < (int)_L->n)
      _perm.resize(2*_L->n);
    int* p = (int*)_L->Perm;
    for (size_t i = 0; i < _L->n; ++i)
      _perm[p[i]] = i;
      
      
    // get the touched vertices
    _touchedVertices.clear();
 
    for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      
      for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
        OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) e->vertex(viIdx);
        if (! v->fixed())
          _touchedVertices.insert(v);
      }
    }
    cerr << PVAR(_touchedVertices.size()) << endl;
    
    // Temporarily add the new edges to the optimizer's _activeEdges vector. This is done so that the robustification function
    // can recompute a proper sigma value, since it accesses all relevant edges through _activeEdges. Afterwards, these extra 
    // edges are discarded.
    size_t nOldEdgesSize = _activeEdges.size();
    _activeEdges.reserve(_activeEdges.size() + eset.size());
    for (HyperGraph::EdgeSet::iterator it = eset.begin(); it != eset.end(); ++it)
      _activeEdges.push_back(static_cast<OptimizableGraph::Edge*>(*it));
    
    // Make sure that there is only ONE vertex in touchedVertices that has dimension 6, and
    // that its hessianIndex is larger than all the other (point) vertices
    
    int largestPointHessianIndex = -1;
    int poseHessianIndex = -1;
    int numPosesFound = 0;
    for (HyperGraph::VertexSet::iterator it = _touchedVertices.begin(); it != _touchedVertices.end(); ++it) 
    {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      if(v->dimension() == 6)
      {
        numPosesFound++;
        poseHessianIndex = v->hessianIndex();
      }
      else
      {
        assert(v->dimension() == 3);
        largestPointHessianIndex = max(largestPointHessianIndex, v->hessianIndex());
      }
    }
    
    assert(numPosesFound == 1);
    assert(poseHessianIndex != -1 && largestPointHessianIndex != -1);
    assert(poseHessianIndex > largestPointHessianIndex);
    
    
    // backup the tempindex and prepare sorting structure
    /*
#ifdef _MSC_VER
    VertexBackup* backupIdx = new VertexBackup[_touchedVertices.size()];
#else
    VertexBackup backupIdx[_touchedVertices.size()];
#endif
    */
    std::vector<VertexBackup> backupIdx(_touchedVertices.size());

    memset(&backupIdx[0], 0, sizeof(VertexBackup) * backupIdx.size());
    int idx = 0;
    for (HyperGraph::VertexSet::iterator it = _touchedVertices.begin(); it != _touchedVertices.end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(*it);
      backupIdx[idx].hessianIndex = v->hessianIndex();
      backupIdx[idx].vertex = v;
      backupIdx[idx].hessianData = v->hessianData();
      ++idx;
    }
    //sort(backupIdx, backupIdx + _touchedVertices.size()); // sort according to the hessianIndex which is the same order as used later by the optimizer
    std::sort(backupIdx.begin(), backupIdx.end());
    for (int i = 0; i < idx; ++i) {
      backupIdx[i].vertex->setHessianIndex(i);
    }
    //cerr << "backup tempindex done." << endl;
    
    // building the structure of the update
    _updateMat.clear(true); // get rid of the old matrix structure
    _updateMat.rowBlockIndices().clear();
    _updateMat.colBlockIndices().clear();
    _updateMat.blockCols().clear();

    // placing the current stuff in _updateMat
    MatrixXd* lastBlock = 0;
    int sizePoses = 0;
    for (int i = 0; i < idx; ++i) {
      OptimizableGraph::Vertex* v = backupIdx[i].vertex;
      int dim = v->dimension();
      sizePoses+=dim;
      _updateMat.rowBlockIndices().push_back(sizePoses);
      _updateMat.colBlockIndices().push_back(sizePoses);
      _updateMat.blockCols().push_back(SparseBlockMatrix<MatrixXd>::IntBlockMap());
      int ind = v->hessianIndex();
      //cerr << PVAR(ind) << endl;
      if (ind >= 0) {
        MatrixXd* m = _updateMat.block(ind, ind, true);
        v->mapHessianMemory(m->data());
        lastBlock = m;
      }
    }
    lastBlock->diagonal().array() += 1e-6; // HACK to get Eigen value > 0
    
    for (HyperGraph::EdgeSet::const_iterator it = eset.begin(); it != eset.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);

      for (size_t viIdx = 0; viIdx < e->vertices().size(); ++viIdx) {
        OptimizableGraph::Vertex* v1 = (OptimizableGraph::Vertex*) e->vertex(viIdx);
        int ind1 = v1->hessianIndex();
        int indexV1Bak = ind1;
        if (ind1 == -1)
          continue;
        for (size_t vjIdx = viIdx + 1; vjIdx < e->vertices().size(); ++vjIdx) {
          OptimizableGraph::Vertex* v2 = (OptimizableGraph::Vertex*) e->vertex(vjIdx);
          int ind2 = v2->hessianIndex();
          if (ind2 == -1)
            continue;
          ind1 = indexV1Bak;
          bool transposedBlock = ind1 > ind2;
          if (transposedBlock) // make sure, we allocate the upper triangular block
            swap(ind1, ind2);

          if (! v1->marginalized() && !v2->marginalized()) {
            MatrixXd* m = _updateMat.block(ind1, ind2, true);
            e->mapHessianMemory(m->data(), viIdx, vjIdx, transposedBlock);
            //std::cerr << "Mapping hessian memory (into _updateMat) for edge connecting " << viIdx << " and " <<vjIdx<<std::endl;
          } else { 
            std::cerr << __PRETTY_FUNCTION__ << ": not supported" << std::endl;
          }
        }
      }
    }
    
    // build the system into _updateMat
    for (HyperGraph::EdgeSet::iterator it = eset.begin(); it != eset.end(); ++it) {
      OptimizableGraph::Edge * e = static_cast<OptimizableGraph::Edge*>(*it);
      e->computeError();
    }
    for (HyperGraph::EdgeSet::iterator it = eset.begin(); it != eset.end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->linearizeOplus(jacobianWorkspace());
      e->constructQuadraticForm();
    }
    
    // restore the original data for the vertex
    for (int i = 0; i < idx; ++i) {
      backupIdx[i].vertex->setHessianIndex(backupIdx[i].hessianIndex);
      if (backupIdx[i].hessianData)
        backupIdx[i].vertex->mapHessianMemory(backupIdx[i].hessianData);
    }
    
    // restore the original _activeEdges, getting rid of all the newly added (temporary) edges
    _activeEdges.resize(nOldEdgesSize);
    
    bool updateStatus = computeCholeskyUpdate();
    if (! updateStatus) {
      cerr << "Error while computing update" << endl;
      return false;
    }
    
    cerr << "Update factor size: " << _cholmodFactor->n << ", " << _cholmodFactor->n << endl;

    cholmod_sparse* updateAsSparseFactor = cholmod_factor_to_sparse(_cholmodFactor, &_cholmodCommon);
    
    cerr << "Update sparse size: " << updateAsSparseFactor->nrow << ", " << updateAsSparseFactor->ncol << endl;
    cerr << "L size: " << _L->n << ", " << _L->n << endl;
    
    //writeOctave(updateAsSparseFactor, "updateAsSparseFactor.dat");
    
    // convert CCS update by permuting back to the permutation of L
    if (updateAsSparseFactor->nzmax > _permutedUpdate->nzmax) {
      //cerr << "realloc _permutedUpdate" << endl;
      cholmod_reallocate_triplet(updateAsSparseFactor->nzmax, _permutedUpdate, &_cholmodCommon);
    }
    _permutedUpdate->nnz = 0;
    _permutedUpdate->nrow = _permutedUpdate->ncol = _L->n;
    
    int* Ap = (int*)updateAsSparseFactor->p;
    int* Ai = (int*)updateAsSparseFactor->i;
    double* Ax = (double*)updateAsSparseFactor->x;
    int* Bj = (int*)_permutedUpdate->j;
    int* Bi = (int*)_permutedUpdate->i;
    double* Bx = (double*)_permutedUpdate->x;
    
    for (size_t c = 0; c < updateAsSparseFactor->ncol; ++c) 
    {
      int coldim = c >= updateAsSparseFactor->ncol - 6 ? 6 : 3;
    
      const int& rbeg = Ap[c];
      const int& rend = Ap[c+1];
      int cc = coldim == 6 ?  (updateAsSparseFactor->ncol - 6)/3 : c / 3;  
      int coff = c - cc*3; // c % 3
      assert(backupIdx[cc].vertex->dimension() == coldim);
      const int& cbase = backupIdx[cc].vertex->colInHessian();
      assert(cbase != -1);
      const int& ccol = _perm(cbase + coff);
      for (int j = rbeg; j < rend; j++) {
        const int& r = Ai[j];
        const double& val = Ax[j];
        
        int rowdim = (size_t)r >= updateAsSparseFactor->nrow - 6 ? 6 : 3;

        int rr = rowdim == 6 ? (updateAsSparseFactor->nrow - 6)/3 : r / 3;
        int roff = r - rr*3; //r % 3;
        assert(backupIdx[rr].vertex->dimension() == rowdim);
        const int& rbase = backupIdx[rr].vertex->colInHessian();
        assert(rbase != -1);
        
        int row = _perm(rbase + roff);
        int col = ccol;
        if (col > row) // lower triangular entry
          swap(col, row);
        Bi[_permutedUpdate->nnz] = row;
        Bj[_permutedUpdate->nnz] = col;
        Bx[_permutedUpdate->nnz] = val;
        ++_permutedUpdate->nnz;
      }
    }
    
    cholmod_free_sparse(&updateAsSparseFactor, &_cholmodCommon);
    /*
#ifdef _MSC_VER
    delete[] backupIdx;
#endif
*/

    convertTripletUpdateToSparse();
    //writeOctave(_permutedUpdateAsSparse, "permutedUpdateAsSparse.dat");
    
    _linearSolver->saveFactor();
    _linearSolver->choleskyUpdate(_permutedUpdateAsSparse);
    
    bool marginalsStatus = computeMarginals(spinv, vertices); 
    
    _linearSolver->restoreFactor();

    return marginalsStatus;
  }
  

  bool SparseOptimizerIncrementalCovariance::computeCholeskyUpdate()
  {
    if (_cholmodFactor) {
      cholmod_free_factor(&_cholmodFactor, &_cholmodCommon);
      _cholmodFactor = 0;
    }

    const SparseBlockMatrix<MatrixXd>& A = _updateMat;
    size_t m = A.rows();
    size_t n = A.cols();

    if (_cholmodSparse->columnsAllocated < n) {
      //std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" << std::endl;
      _cholmodSparse->columnsAllocated = _cholmodSparse->columnsAllocated == 0 ? n : 2 * n; // pre-allocate more space if re-allocating
      delete[] (int*)_cholmodSparse->p;
      _cholmodSparse->p = new int[_cholmodSparse->columnsAllocated+1];
    }
    size_t nzmax = A.nonZeros();
    if (_cholmodSparse->nzmax < nzmax) {
      //std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" << std::endl;
      _cholmodSparse->nzmax = _cholmodSparse->nzmax == 0 ? nzmax : 2 * nzmax; // pre-allocate more space if re-allocating
      delete[] (double*)_cholmodSparse->x;
      delete[] (int*)_cholmodSparse->i;
      _cholmodSparse->i = new int[_cholmodSparse->nzmax];
      _cholmodSparse->x = new double[_cholmodSparse->nzmax];
    }
    _cholmodSparse->ncol = n;
    _cholmodSparse->nrow = m;

    A.fillCCS((int*)_cholmodSparse->p, (int*)_cholmodSparse->i, (double*)_cholmodSparse->x, true);
    //writeCCSMatrix("updatesparse.txt", _cholmodSparse->nrow, _cholmodSparse->ncol, (int*)_cholmodSparse->p, (int*)_cholmodSparse->i, (double*)_cholmodSparse->x, true);

    _cholmodFactor = cholmod_analyze(_cholmodSparse, &_cholmodCommon);
    cholmod_factorize(_cholmodSparse, _cholmodFactor, &_cholmodCommon);

#if 0
    int* p = (int*)_cholmodFactor->Perm;
    for (int i = 0; i < (int)n; ++i)
      if (i != p[i])
        cerr << "wrong permutation" << i << " -> " << p[i] << endl;
#endif

    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
      //std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)" << std::endl;
      //writeCCSMatrix("debug.txt", _cholmodSparse->nrow, _cholmodSparse->ncol, (int*)_cholmodSparse->p, (int*)_cholmodSparse->i, (double*)_cholmodSparse->x, true);
      return false;
    }

    // change to the specific format we need to have a pretty normal L
    int change_status = cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor, &_cholmodCommon);
    if (! change_status) {
      return false;
    }

    return true;
  }

  void SparseOptimizerIncrementalCovariance::convertTripletUpdateToSparse()
  {
    // re-allocate the memory
    if (_tripletWorkspace.size() < (int)_permutedUpdate->ncol) {
      _tripletWorkspace.resize(_permutedUpdate->ncol * 2);
    }

    // reallocate num-zeros
    if (_permutedUpdateAsSparse->nzmax < _permutedUpdate->nzmax) {
      _permutedUpdateAsSparse->nzmax = _permutedUpdate->nzmax;
      delete[] (int*)_permutedUpdateAsSparse->i;
      delete[] (double*)_permutedUpdateAsSparse->x;
      _permutedUpdateAsSparse->x = new double[_permutedUpdateAsSparse->nzmax];
      _permutedUpdateAsSparse->i = new int[_permutedUpdateAsSparse->nzmax];
    }

    if (_permutedUpdateAsSparse->columnsAllocated < _permutedUpdate->ncol) {
      _permutedUpdateAsSparse->columnsAllocated = 2*_permutedUpdate->ncol;
      delete[] (int*) _permutedUpdateAsSparse->p;
      _permutedUpdateAsSparse->p = new int[_permutedUpdateAsSparse->columnsAllocated + 1];
    }

    _permutedUpdateAsSparse->ncol = _permutedUpdate->ncol;
    _permutedUpdateAsSparse->nrow = _permutedUpdate->nrow;

    int* w = _tripletWorkspace.data();
    memset(w, 0, sizeof(int) * _permutedUpdate->ncol);

    int* Ti = (int*) _permutedUpdate->i;
    int* Tj = (int*) _permutedUpdate->j;
    double* Tx = (double*) _permutedUpdate->x;

    int* Cp = (int*) _permutedUpdateAsSparse->p;
    int* Ci = (int*) _permutedUpdateAsSparse->i;
    double* Cx = (double*) _permutedUpdateAsSparse->x;

    for (size_t k = 0; k < _permutedUpdate->nnz; ++k) /* column counts */
      w[Tj [k]]++;

    /* column pointers */
    int n = _permutedUpdate->ncol;
    int nz = 0;
    for (int i = 0 ; i < n ; i++) {
      Cp[i] = nz;
      nz += w[i];
      w[i] = Cp[i];
    }
    Cp[n] = nz;
    assert((size_t)nz == _permutedUpdate->nnz);

    int p;
    for (size_t k = 0 ; k < _permutedUpdate->nnz ; ++k) {
      p = w[Tj[k]]++;
      Ci[p] = Ti[k] ;    /* A(i,j) is the pth entry in C */
      Cx[p] = Tx[k] ;
    }

  }

} // end namespace
