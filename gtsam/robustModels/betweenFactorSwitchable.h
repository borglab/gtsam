/*
 * betweenFactorSwitchable.h
 *
 *  Created on: 02.08.2012
 *      Author: niko
 */

//#include <gtsam/slam/planarSLAM.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/robustModels/switchVariableLinear.h>

#include <iostream>

using std::cout;
using std::endl;

namespace gtsam {

  template<class VALUE>
  class BetweenFactorSwitchableLinear : public NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear>
  {
    public:
      BetweenFactorSwitchableLinear() {};
      BetweenFactorSwitchableLinear(Key key1, Key key2, Key key3, const VALUE& measured, const SharedNoiseModel& model)
      : NoiseModelFactor3<VALUE, VALUE, SwitchVariableLinear>(model, key1, key2, key3),
        betweenFactor(key1, key2, measured, model) {};

      Vector evaluateError(const VALUE& p1, const VALUE& p2, const SwitchVariableLinear& s,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 =  boost::none,
          boost::optional<gtsam::Matrix&> H3 =  boost::none) const
        {

          // calculate error
          Vector error = betweenFactor.evaluateError(p1, p2, H1, H2);
          error *= s.value();

          // handle derivatives
          if (H1) *H1 = *H1 * s.value();
          if (H2) *H2 = *H2 * s.value();
          if (H3) *H3 = error;

          return error;
        };

    private:
      BetweenFactor<VALUE> betweenFactor;

  };

}
