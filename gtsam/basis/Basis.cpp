/**
 * @file Basis.cpp
 * @brief Compute an interpolating basis
 * @author Varun Agrawal
 * @date July 4, 2020
 */

#include <gtsam/basis/Basis.h>

#include <iostream>

namespace gtsam {

template <typename DERIVED>
Matrix Basis<DERIVED>::WeightMatrix(size_t N, const Vector& X) {
  Matrix W(X.size(), N);
  for (int i = 0; i < X.size(); i++)
    W.row(i) = DERIVED::CalculateWeights(N, X(i));
  return W;
}

template <typename DERIVED>
Matrix Basis<DERIVED>::WeightMatrix(size_t N, const Vector& X, double a,
                                    double b) {
  Matrix W(X.size(), N);
  for (int i = 0; i < X.size(); i++)
    W.row(i) = DERIVED::CalculateWeights(N, X(i), a, b);
  return W;
}

// Define all the functions with `cout` outside the header

template <typename DERIVED>
void Basis<DERIVED>::EvaluationFunctor::print(const std::string& s) const {
  std::cout << s << (s != "" ? " " : "") << weights_ << std::endl;
}

template <typename DERIVED>
void Basis<DERIVED>::DerivativeFunctorBase::print(const std::string& s) const {
  std::cout << s << (s != "" ? " " : "") << weights_ << std::endl;
}

}  // namespace gtsam
