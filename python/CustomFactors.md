# GTSAM Python-based factors

One now can build factors purely in Python using the `CustomFactor` factor.

## Usage

In order to use a Python-based factor, one needs to have a Python function with the following signature:

```python
import gtsam
import numpy as np
from typing import List

def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]) -> np.ndarray:
    ...
```

`this` is a reference to the `CustomFactor` object. This is required because one can reuse the same
`error_func` for multiple factors. `v` is a reference to the current set of values, and `H` is a list of
**references** to the list of required Jacobians (see the corresponding C++ documentation).  Note that 
the error returned must be a 1D `numpy` array.

If `H` is `None`, it means the current factor evaluation does not need Jacobians. For example, the `error`
method on a factor does not need Jacobians, so we don't evaluate them to save CPU. If `H` is not `None`,
each entry of `H` can be assigned a (2D) `numpy` array, as the Jacobian for the corresponding variable.

All `numpy` matrices inside should be using `order="F"` to maintain interoperability with C++.

After defining `error_func`, one can create a `CustomFactor` just like any other factor in GTSAM:

```python
noise_model = gtsam.noiseModel.Unit.Create(3)
# constructor(<noise model>, <list of keys>, <error callback>)
cf = gtsam.CustomFactor(noise_model, [X(0), X(1)], error_func)
```

## Example

The following is a simple `BetweenFactor` implemented in Python.

```python
import gtsam
import numpy as np
from typing import List

expected = Pose2(2, 2, np.pi / 2)

def error_func(this: CustomFactor, v: gtsam.Values, H: List[np.ndarray]) -> np.ndarray:
    """
    Error function that mimics a BetweenFactor
    :param this: reference to the current CustomFactor being evaluated
    :param v: Values object
    :param H: list of references to the Jacobian arrays
    :return: the non-linear error
    """
    key0 = this.keys()[0]
    key1 = this.keys()[1]
    gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
    error = expected.localCoordinates(gT1.between(gT2))

    if H is not None:
        result = gT1.between(gT2)
        H[0] = -result.inverse().AdjointMap()
        H[1] = np.eye(3)
    return error

noise_model = gtsam.noiseModel.Unit.Create(3)
cf = gtsam.CustomFactor(noise_model, gtsam.KeyVector([0, 1]), error_func)
```

In general, the Python-based factor works just like their C++ counterparts.

## Known Issues

Because of the `pybind11`-based translation, the performance of `CustomFactor` is not guaranteed.
Also, because `pybind11` needs to lock the Python GIL lock for evaluation of each factor, parallel
evaluation of `CustomFactor` is not possible.

## Implementation

`CustomFactor` is a `NonlinearFactor` that has a `std::function` as its callback.
This callback can be translated to a Python function call, thanks to `pybind11`'s functional support.

The constructor of `CustomFactor` is
```c++
/**
* Constructor
* @param noiseModel shared pointer to noise model
* @param keys keys of the variables
* @param errorFunction the error functional
*/
CustomFactor(const SharedNoiseModel& noiseModel, const KeyVector& keys, const CustomErrorFunction& errorFunction) :
  Base(noiseModel, keys) {
  this->error_function_ = errorFunction;
}
```

At construction time, `pybind11` will pass the handle to the Python callback function as a `std::function` object.

Something worth special mention is this:
```c++
/*
 * NOTE
 * ==========
 * pybind11 will invoke a copy if this is `JacobianVector &`, and modifications in Python will not be reflected.
 *
 * This is safe because this is passing a const pointer, and pybind11 will maintain the `std::vector` memory layout.
 * Thus the pointer will never be invalidated.
 */
using CustomErrorFunction = std::function<Vector(const CustomFactor&, const Values&, const JacobianVector*)>;
```

which is not documented in `pybind11` docs. One needs to be aware of this if they wanted to implement similar
"mutable" arguments going across the Python-C++ boundary.
