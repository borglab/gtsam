/*
 *  @file   betweenFactorMaxMix.h
 *  @author Ryan
 *  @brief  Header file for Pseudorange Switchable factor
 */

#include <Eigen/Eigen>
#include <gtsam/linear/NoiseModel.h>

namespace gtsam {

  template<class VALUE>
  class BetweenFactorMaxMix : public NoiseModelFactor2<VALUE, VALUE>
  {
    public:
      BetweenFactorMaxMix() {};
      BetweenFactorMaxMix(Key key1, Key key2, const VALUE& measured, const SharedNoiseModel& model, const SharedNoiseModel& model2, const Vector& hypVec, double w)
      : NoiseModelFactor2<VALUE, VALUE>(model, key1, key2), weight(w), nullHypothesisModel(model2), hyp(hypVec),
        betweenFactor(key1, key2, measured, model)  {   };

      Vector evaluateError(const VALUE& p1, const VALUE& p2,
          boost::optional<Matrix&> H1 = boost::none,
          boost::optional<Matrix&> H2 =  boost::none) const
        {

          // calculate error
          Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);

          // which hypothesis is more likely
          auto g1 = noiseModel::Gaussian::Covariance(hyp.asDiagonal());
          auto g2 = noiseModel::Gaussian::Covariance((hyp/weight).asDiagonal());

          double m1 = this->noiseModel_->distance(error);
          Matrix info1(g1->information());
          double nu1 = 1.0/sqrt(inverse(info1).determinant());
          double l1 = nu1 * exp(-0.5*m1);

          double m2 = nullHypothesisModel->distance(error);
          Matrix info2(g2->information());
          double nu2 = 1.0/sqrt(inverse(info2).determinant());
          double l2 = nu2 * exp(-0.5*m2);

          if (l2>l1) {
            if (H1) *H1 = *H1 * weight;
            if (H2) *H2 = *H2 * weight;
            error *= sqrt(weight);
          }

          return error;
        };

    private:
      BetweenFactor<VALUE> betweenFactor;
      SharedNoiseModel nullHypothesisModel;
      double weight;
      Vector hyp;

  };
}
