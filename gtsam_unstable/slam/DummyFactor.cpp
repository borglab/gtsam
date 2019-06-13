/**
 * @file DummyFactor.cpp
 *
 * @date Sep 10, 2012
 * @author Alex Cunningham
 */

#include <gtsam_unstable/slam/DummyFactor.h>

#include <boost/assign/list_of.hpp>

using namespace boost::assign;

namespace gtsam {

/* ************************************************************************* */
DummyFactor::DummyFactor(const Key& key1, size_t dim1, const Key& key2, size_t dim2)
: NonlinearFactor(cref_list_of<2>(key1)(key2))
{
  dims_.push_back(dim1);
  dims_.push_back(dim2);
  if (dim1 > dim2)
    rowDim_ = dim1;
  else
    rowDim_ = dim2;
}

/* ************************************************************************* */
void DummyFactor::print(const std::string& s, const KeyFormatter& keyFormatter) const {
  std::cout << s << "  DummyFactor dim = " << rowDim_ << ", keys = { ";
  for(Key key: this->keys()) { std::cout << keyFormatter(key) << " "; }
  std::cout << "}" << std::endl;
}

/* ************************************************************************* */
bool DummyFactor::equals(const NonlinearFactor& f, double tol) const {
  const DummyFactor* e = dynamic_cast<const DummyFactor*>(&f);
  return e && Base::equals(f, tol) && dims_ == e->dims_ && rowDim_ == e->rowDim_;
}

/* ************************************************************************* */
boost::shared_ptr<GaussianFactor>
DummyFactor::linearize(const Values& c) const {
  // Only linearize if the factor is active
  if (!this->active(c))
    return boost::shared_ptr<JacobianFactor>();

   // Fill in terms with zero matrices
  std::vector<std::pair<Key, Matrix> > terms(this->size());
  for(size_t j=0; j<this->size(); ++j) {
    terms[j].first = keys()[j];
    terms[j].second = Matrix::Zero(rowDim_, dims_[j]);
  }

  noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(rowDim_);
  return GaussianFactor::shared_ptr(
      new JacobianFactor(terms, Vector::Zero(rowDim_), model));
}

/* ************************************************************************* */

} // \namespace gtsam




