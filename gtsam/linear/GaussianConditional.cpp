/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianConditional.cpp
 * @brief  Conditional Gaussian Base class
 * @author Christian Potthast
 */

#include <string.h>
#include <functional>
#include <boost/format.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include <gtsam/linear/linearExceptions.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  GaussianConditional::GaussianConditional(
    Key key, const Vector& d, const Matrix& R,
    Key name1, const Matrix& S, Key name2, const Matrix& T, const SharedDiagonal& sigmas) :
  BaseFactor(key, R, name1, S, name2, T, d, sigmas), BaseConditional(1) {}

  /* ************************************************************************* */
  void GaussianConditional::print(const string &s, const KeyFormatter& formatter) const
  {
    cout << s << "  Conditional density ";
    for(const_iterator it = beginFrontals(); it != endFrontals(); ++it) {
      cout << (boost::format("[%1%]")%(formatter(*it))).str() << " ";
    }
    cout << endl;
    cout << formatMatrixIndented("  R = ", get_R()) << endl;
    for(const_iterator it = beginParents() ; it != endParents() ; ++it ) {
      cout << formatMatrixIndented((boost::format("  S[%1%] = ")%(formatter(*it))).str(), getA(it))
        << endl;
    }
    cout << formatMatrixIndented("  d = ", getb(), true) << "\n";
    if(model_)
      model_->print("  Noise model: ");
    else
      cout << "  No noise model" << endl;
  }

  /* ************************************************************************* */
  bool GaussianConditional::equals(const GaussianFactor& f, double tol) const
  {
    if (const GaussianConditional* c = dynamic_cast<const GaussianConditional*>(&f))
    {
      // check if the size of the parents_ map is the same
      if (parents().size() != c->parents().size())
        return false;

      // check if R_ and d_ are linear independent
      for (DenseIndex i = 0; i < Ab_.rows(); i++) {
        list<Vector> rows1; rows1.push_back(Vector(get_R().row(i)));
        list<Vector> rows2; rows2.push_back(Vector(c->get_R().row(i)));

        // check if the matrices are the same
        // iterate over the parents_ map
        for (const_iterator it = beginParents(); it != endParents(); ++it) {
          const_iterator it2 = c->beginParents() + (it - beginParents());
          if (*it != *(it2))
            return false;
          rows1.push_back(row(getA(it), i));
          rows2.push_back(row(c->getA(it2), i));
        }

        Vector row1 = concatVectors(rows1);
        Vector row2 = concatVectors(rows2);
        if (!linear_dependent(row1, row2, tol))
          return false;
      }

      // check if sigmas are equal
      if ((model_ && !c->model_) || (!model_ && c->model_)
        || (model_ && c->model_ && !model_->equals(*c->model_, tol)))
        return false;

      return true;
    }
    else
    {
      return false;
    }
  }

  /* ************************************************************************* */
  VectorValues GaussianConditional::solve(const VectorValues& x) const
  {
    // Concatenate all vector values that correspond to parent variables
    const Vector xS = x.vector(FastVector<Key>(beginParents(), endParents()));

    // Update right-hand-side
    const Vector rhs = get_d() - get_S() * xS;

    // Solve matrix
    const Vector solution = get_R().triangularView<Eigen::Upper>().solve(rhs);

    // Check for indeterminant solution
    if (solution.hasNaN()) {
      throw IndeterminantLinearSystemException(keys().front());
    }

    // Insert solution into a VectorValues
    VectorValues result;
    DenseIndex vectorPosition = 0;
    for (const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      result.insert(*frontal, solution.segment(vectorPosition, getDim(frontal)));
      vectorPosition += getDim(frontal);
    }

    return result;
  }
//================ copyVecAt(), toLongVec() ==================
//[MH-A]:
void copyVecAt(std::vector<Vector>& in_arr, std::vector<Vector>& out_arr, const size_t& dim, const size_t& pos) {
  out_arr.resize(in_arr.size());
  for (size_t i = 0; i < in_arr.size(); ++i) {
    out_arr[i] = in_arr[i].segment(pos, dim);
  }
}
//[MH-A]:
Vector toLongVec(std::vector<Vector>& in_arr, const size_t& dim) {
  Vector out_v(dim*in_arr.size());
  size_t pos = 0;
  for (size_t i = 0; i < in_arr.size(); ++i) {
    out_v.segment(pos, dim) = in_arr[i];
    pos += dim;
  }
  return out_v; 
}
//================ END copyVecAt(), toLongVec() ==================

//============================================== mhSolve() ==========================================
  //[MH-A]:
  VectorValues GaussianConditional::mhSolve(Values& theta, const VectorValues& x, const double& splitThreshold) const
  {
    VectorValues result;

    std::vector<Vector> solution_arr;
    int max_layer_idx = -1;

    const int this_layer_idx = resulting_layer_->getLayerIdx();

    if (nrParents() != 0) { //NOT the root clique
      
      //[MH-A]: Find max-hypo among all xS (separators)
      std::vector<HypoLayer*> hypo_layer_arr(nrParents());

      int max_idx = -1;
      //size_t sum_dim = 0;
      std::vector<size_t> dim_arr(nrParents()); 

      //[MH-A]: find the max-hypo among all separators (parents) and use it to set the hypo of the output solution
      size_t hla_idx = 0;
      for(auto kit = beginParents(); kit != endParents(); ++kit) {

        Value* val_ptr = &(theta.at(*kit));
        HypoLayer* tmp_layer_ptr = val_ptr->getHypoLayer();
        
        dim_arr[hla_idx] = val_ptr->dim();

        const int tmp_layer_idx = tmp_layer_ptr->getLayerIdx();

        hypo_layer_arr[hla_idx] = tmp_layer_ptr; //later used in mhUpdateVectorArr()

        if (tmp_layer_idx >= max_layer_idx) {
          max_layer_idx = tmp_layer_idx;
          max_idx = hla_idx;
        }
        
        ++hla_idx;
      }

      HypoLayer* max_layer_ptr = hypo_layer_arr[max_idx];

      // WARNING: Consider HypoLayers of the frontals as well
      for(auto kit = beginFrontals(); kit != endFrontals(); ++kit) {
        Value* val_ptr = &(theta.at(*kit));
        HypoLayer* tmp_layer_ptr = val_ptr->getHypoLayer();

        const int tmp_layer_idx = tmp_layer_ptr->getLayerIdx();
        if (tmp_layer_idx >= max_layer_idx) {
          max_layer_idx = tmp_layer_idx;
          max_layer_ptr = tmp_layer_ptr;
        }
      }

      if (max_layer_idx < this_layer_idx) {
std::cout << "ERROR: max_layer_idx from both parents and frontals (" << max_layer_idx << ") < this_layer_idx (" << this_layer_idx << ") should NEVER happen !!" << std::endl;      
        // Force to expand
        max_layer_idx = this_layer_idx;
        max_layer_ptr = resulting_layer_;
std::cout << "FINAL_MAX: " << max_layer_idx << std::endl;      
      }
      
      size_t sum_dim = std::accumulate(dim_arr.begin(), dim_arr.end(), 0);
       
      //[MH-A]: Create Vector
      //const HypoList& max_hypo_list = *(hypo_list_arr[max_idx]);
      const HypoList& max_hypo_list = max_layer_ptr->getNodeList();
      std::vector<Vector> xS_arr(max_hypo_list.size(), Vector(sum_dim)); //(# hypo) * (dim)

      //max_layer_idx = hypo_layer_arr[max_idx]->getLayerIdx();
      size_t pos = 0;
      int hla_count = 0;
      for(auto kit = beginParents(); kit != endParents(); ++kit) {

        //[MH-A]: Used in all cliques except the root 
        
        const size_t& dim = dim_arr[hla_count];

        x.mhUpdateVectorArr((*kit), dim, pos, hypo_layer_arr[hla_count], xS_arr, max_layer_idx); //later half of the Keys
        hla_count++;
        pos += dim;

      }   
    
      //[MH-A]: Get solution based on the max-hypo (assocate across the HypoTree)
      const int layer_diff = max_layer_idx - this_layer_idx;
      
      std::vector<int> descendant_num_arr(hypoSize());
      
      size_t dna_idx = 0;

      const HypoList& hypo_list = getHypoList();
      for (HypoListCstIter it = hypo_list.begin(); it != hypo_list.end(); ++it) {
        //[MH-A]:
        descendant_num_arr[dna_idx] = (*it)->findDescendantNum(layer_diff);
        ++dna_idx;
      }

      solution_arr.resize(xS_arr.size());
      
      int dna_count = 0; //slower
      int xs_count = 0; //faster
      for (JacobListCstIter jit = jacobian_list_.begin(); jit != jacobian_list_.end(); ++jit) {

        auto& num = descendant_num_arr[dna_count];
        for (int k = 0; k < num; ++k) {

          solution_arr[xs_count] = mhGet_single_R(jit).triangularView<Eigen::Upper>().solve(mhGet_single_d(jit) - mhGet_single_S(jit)*xS_arr[xs_count]);
          xs_count++;
          
          if (solution_arr.back().hasNaN()) {
            throw IndeterminantLinearSystemException(keys().front());
          }
        }
        dna_count++;
      }
    
    } else { //root Clique
      //[MH-A]: Get solution based on itself
      max_layer_idx = this_layer_idx;
    
      solution_arr.resize(jacobian_list_.size());
      size_t sa_idx = 0;
      for (JacobListCstIter jit = jacobian_list_.begin(); jit != jacobian_list_.end(); ++jit) { 
        solution_arr[sa_idx] = mhGet_single_R(jit).triangularView<Eigen::Upper>().solve(mhGet_single_d(jit));
        ++sa_idx;

        if (solution_arr.back().hasNaN()) {
          throw IndeterminantLinearSystemException(keys().front());
        }
      }
    } // END if-else root

    //[MH-A]: Re-group the output VectorArr into VectorValues while eliminating deplicated items (might have to check if the baseline # hypo of each Vector is greater than the # hypo of the corresponding MHGV)
    //[MH-A]: Add corresponding hypo of Values based on the final # hypo of solutions
    int pos = 0;
    for(auto kit = beginFrontals(); kit != endFrontals(); ++kit) {
      size_t dim = getDim(kit);
      std::vector<Vector> vec_arr;
      copyVecAt(solution_arr, vec_arr, dim, pos); //mhsiao: each single Vector, could be merged based on hypo
      
      theta.at(*kit).mergeHypoAndSetAgree(vec_arr, max_layer_idx, (*kit), splitThreshold); //mhsiao: vec_arr and front_layer both got modified in the function...
      
      //[MH-opt]:
      result.insert((*kit), toLongVec(vec_arr, dim)); //the only time waste here: about 1% ~ 2% in total... (NOT worth optimizing this?)
      pos += dim;
    }

    return result;
  }

//============================================== END mhSolve() ==========================================


  /* ************************************************************************* */
  VectorValues GaussianConditional::solveOtherRHS(
    const VectorValues& parents, const VectorValues& rhs) const
  {
    // Concatenate all vector values that correspond to parent variables
    Vector xS = parents.vector(FastVector<Key>(beginParents(), endParents()));

    // Instead of updating getb(), update the right-hand-side from the given rhs
    const Vector rhsR = rhs.vector(FastVector<Key>(beginFrontals(), endFrontals()));
    xS = rhsR - get_S() * xS;

    // Solve Matrix
    Vector soln = get_R().triangularView<Eigen::Upper>().solve(xS);

    // Scale by sigmas
    if(model_)
      soln.array() *= model_->sigmas().array();

    // Insert solution into a VectorValues
    VectorValues result;
    DenseIndex vectorPosition = 0;
    for(const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      result.insert(*frontal, soln.segment(vectorPosition, getDim(frontal)));
      vectorPosition += getDim(frontal);
    }

    return result;
  }

  /* ************************************************************************* */
  void GaussianConditional::solveTransposeInPlace(VectorValues& gy) const
  {
    Vector frontalVec = gy.vector(FastVector<Key>(beginFrontals(), endFrontals()));
    frontalVec = gtsam::backSubstituteUpper(frontalVec, Matrix(get_R()));

    // Check for indeterminant solution
    if (frontalVec.hasNaN()) throw IndeterminantLinearSystemException(this->keys().front());

    for (const_iterator it = beginParents(); it!= endParents(); it++)
      gy[*it] += -1.0 * Matrix(getA(it)).transpose() * frontalVec;

    // Scale by sigmas
    if(model_)
      frontalVec.array() *= model_->sigmas().array();

    // Write frontal solution into a VectorValues
    DenseIndex vectorPosition = 0;
    for(const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      gy[*frontal] = frontalVec.segment(vectorPosition, getDim(frontal));
      vectorPosition += getDim(frontal);
    }
  }

  /* ************************************************************************* */
  void GaussianConditional::scaleFrontalsBySigma(VectorValues& gy) const
  {
    DenseIndex vectorPosition = 0;
    for(const_iterator frontal = beginFrontals(); frontal != endFrontals(); ++frontal) {
      gy[*frontal].array() *= model_->sigmas().segment(vectorPosition, getDim(frontal)).array();
      vectorPosition += getDim(frontal);
    }
  }

//============================== MHGaussianConditional::functions() =======================
/*
  bool MHGaussianConditional::equals(const GaussianFactor& f, double tol) const {
    //
    if (const GaussianConditional* c = dynamic_cast<const GaussianConditional*>(&f))
    {
      // check if the size of the parents_ map is the same
      if (parents().size() != c->parents().size())
        return false;

      // check if R_ and d_ are linear independent
      for (DenseIndex i = 0; i < Ab_.rows(); i++) {
        list<Vector> rows1; rows1.push_back(Vector(get_R().row(i)));
        list<Vector> rows2; rows2.push_back(Vector(c->get_R().row(i)));

        // check if the matrices are the same
        // iterate over the parents_ map
        for (const_iterator it = beginParents(); it != endParents(); ++it) {
          const_iterator it2 = c->beginParents() + (it - beginParents());
          if (*it != *(it2))
            return false;
          rows1.push_back(row(getA(it), i));
          rows2.push_back(row(c->getA(it2), i));
        }

        Vector row1 = concatVectors(rows1);
        Vector row2 = concatVectors(rows2);
        if (!linear_dependent(row1, row2, tol))
          return false;
      }

      // check if sigmas are equal
      if ((model_ && !c->model_) || (!model_ && c->model_)
        || (model_ && c->model_ && !model_->equals(*c->model_, tol)))
        return false;

      return true;
    }
    else
    {
      return false;
    }
    //
  } // END MHGaussianConditional::equals()
// */

//============================== END MHGaussianConditional::functions() =======================
} // END gtsam namespace
