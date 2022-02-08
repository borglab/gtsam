/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NonlinearFactor.cpp
 * @brief   Nonlinear Factor base classes
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>

namespace gtsam {

/* ************************************************************************* */
void NonlinearFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  std::cout << s << "  keys = { ";
  for(Key key: keys()) {
    std::cout << keyFormatter(key) << " ";
  }
  std::cout << "}" << std::endl;
}

/* ************************************************************************* */
bool NonlinearFactor::equals(const NonlinearFactor& f, double tol) const {
  return Base::equals(f);
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const std::map<Key, Key>& rekey_mapping) const {
  shared_ptr new_factor = clone();
  for (size_t i = 0; i < new_factor->size(); ++i) {
    Key& cur_key = new_factor->keys()[i];
    std::map<Key, Key>::const_iterator mapping = rekey_mapping.find(cur_key);
    if (mapping != rekey_mapping.end())
      cur_key = mapping->second;
  }
  return new_factor;
}

/* ************************************************************************* */
NonlinearFactor::shared_ptr NonlinearFactor::rekey(
    const std::vector<Key>& new_keys) const {
  assert(new_keys.size() == keys().size());
  shared_ptr new_factor = clone();
  new_factor->keys() = new_keys;
  return new_factor;
}

/* ************************************************************************* */
void NoiseModelFactor::print(const std::string& s,
    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  if (noiseModel_)
    noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool NoiseModelFactor::equals(const NonlinearFactor& f, double tol) const {
  const NoiseModelFactor* e = dynamic_cast<const NoiseModelFactor*>(&f);
  return e && Base::equals(f, tol)
      && ((!noiseModel_ && !e->noiseModel_)
          || (noiseModel_ && e->noiseModel_
              && noiseModel_->equals(*e->noiseModel_, tol)));
}

/* ************************************************************************* */
static void check(const SharedNoiseModel& noiseModel, size_t m) {
  if (noiseModel && m != noiseModel->dim())
    throw std::invalid_argument(
        boost::str(
            boost::format(
                "NoiseModelFactor: NoiseModel has dimension %1% instead of %2%.")
                % noiseModel->dim() % m));
}

/* ************************************************************************* */
Vector NoiseModelFactor::whitenedError(const Values& c) const {
  const Vector b = unwhitenedError(c);
  check(noiseModel_, b.size());
  return noiseModel_ ? noiseModel_->whiten(b) : b;
}

/* ************************************************************************* */
double NoiseModelFactor::error(const Values& c) const {
  if (active(c)) {
    const Vector b = unwhitenedError(c);
    check(noiseModel_, b.size());
    if (noiseModel_)
      return 0.5 * noiseModel_->distance(b);
    else
      return 0.5 * b.squaredNorm();
  } else {
    return 0.0;
  }
}

/* ************************************************************************* */
boost::shared_ptr<GaussianFactor> NoiseModelFactor::linearize(
    const Values& x) const {

  // Only linearize if the factor is active
  if (!active(x))
    return boost::shared_ptr<JacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  std::vector<Matrix> A(size());
  Vector b = -unwhitenedError(x, A);
  check(noiseModel_, b.size());

  // Whiten the corresponding system now
  if (noiseModel_)
    noiseModel_->WhitenSystem(A, b);

  // Fill in terms, needed to create JacobianFactor below
  std::vector<std::pair<Key, Matrix> > terms(size());
  for (size_t j = 0; j < size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second.swap(A[j]);
  }

  // TODO pass unwhitened + noise model to Gaussian factor
  using noiseModel::Constrained;
  if (noiseModel_ && noiseModel_->isConstrained()) {
    return GaussianFactor::shared_ptr(
        new JacobianFactor(terms, b,
            boost::static_pointer_cast<Constrained>(noiseModel_)->unit()));
  } else {
    return GaussianFactor::shared_ptr(new JacobianFactor(terms, b));
  }
}

/* ************************************************************************* */

//==================================== MHNoiseModelFactor::mhLinearize() ==========================================
//[MH-A]: 
boost::shared_ptr<GaussianFactor> MHNoiseModelFactor::mhLinearize(const Values& x) const {
  
  // Only linearize if the factor is active
  if (!active(x)) //default skip
    return boost::shared_ptr<MHJacobianFactor>();

  // Call evaluate error to get Jacobians and RHS vector b
  //[MH-A]: get max_hypo_layer from all connected input Values x;
  int max_layer_idx = 0; //mhsiao: the max_hypo_num defines the output number of hypos in MHJacobianfactor
  HypoLayer* resulting_layer; //WARNING: initialization needed????
    
  int max_key_idx = -1; 
  for (size_t i = 0; i < size(); ++i) {

    HypoLayer* tmp_layer_ptr = x.at(keys()[i]).getHypoLayer();
    int tmp_layer_idx = tmp_layer_ptr->getLayerIdx(); //mhsiao: define a virtual hypoNum() inside Value and define the implementation in both GenericValue and MHGenericValue so that we do not need MHValue here...
    
    if (tmp_layer_idx >= max_layer_idx) {
       
        max_layer_idx = tmp_layer_idx;
        resulting_layer = tmp_layer_ptr;
        
        max_key_idx = i;
    }
  }

  //[MHA]: check
  if (max_key_idx == -1) { //impossible... sth must go wrong...
    throw std::invalid_argument("ERROR: MHNoiseModelFactor::mhLinearize() cannot find max_hypo_num properly...");
  }
  
  setMaxKeyIdx(max_key_idx);
  
  std::vector<std::vector<Matrix> > A_arr(resulting_layer->getNodeSize(), std::vector<Matrix>(size())); //2D size-initialization of std::vector
  
  //[MH-G]: corresponding NoiseModels
  std::vector<SharedNoiseModel> corresp_noiseModel_arr(resulting_layer->getNodeSize());

  //[MH-A]: NOTICE: have to setMaxKeyIdx() before calling mhUnwhitenedError()
  //[MH-G]: output corresponding NoiseModels if different in each hypo
  std::vector<Vector> b_arr = mhUnwhitenedError(x, A_arr, corresp_noiseModel_arr); //NOTICE: the original negative (-) is now implemented in the loop below
  
  for (size_t i = 0; i < b_arr.size(); ++i) {
    b_arr[i] = -b_arr[i]; //negative (-) is here!!
  }
    
  //check(noiseModel_, b_arr.front().size());
  check(noiseModel_arr_.front(), b_arr.front().size());
  
  // Whiten the corresponding system now
  //noiseModel_->WhitenSystem(A_arr[i], b_arr[i]);
  if (noiseModel_arr_.size() == 1) {
    for (size_t i = 0; i < b_arr.size(); ++i) {
      //[MH-A]: same for MH
      noiseModel_arr_.front()->WhitenSystem(A_arr[i], b_arr[i]);
    }
  } else if (noiseModel_arr_.size() > 1) {
    for (size_t i = 0; i < b_arr.size(); ++i) {
      //[MH-G]: find each corresponding NoiseModel 
      corresp_noiseModel_arr[i]->WhitenSystem(A_arr[i], b_arr[i]);
    }
  }
    //[MH-G]:
  
  // Fill in terms, needed to create JacobianFactor below
  //[MH-A]: create mh_terms that will be used to generate MHJacobianFactor 
  std::vector<std::pair<Key, std::vector<Matrix> > > mh_terms(size()); //mh_terms[key_size][hypo_size]
  for (size_t j = 0; j < size(); ++j) { //j: key
    mh_terms[j].first = keys()[j];
    mh_terms[j].second.resize(A_arr.size());
    for (size_t i = 0; i < A_arr.size(); ++i) { //i: hypo

      (mh_terms[j].second)[i].swap(A_arr[i][j]); //A_arr[hypo_size][key_size]

    }
  }
    
    //[MH-A]: Create MHJacobianFactor (and assign corresponding hypos from the above hypo_list_ ... not implemented yet) 
    return GaussianFactor::shared_ptr(new MHJacobianFactor(mh_terms, b_arr, resulting_layer)); //we don't need hypo_list_ in MHJacobianFactor since we can recover the hypo_list_ from x when constructing MHHessianFactor...
}

//==================================== END MHNoiseModelFactor::mhLinearize() ==========================================

} // \namespace gtsam
