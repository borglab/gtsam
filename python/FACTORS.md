# GTSAM Python-based factors

One now can build factors purely in Python using the `CustomFactor` factor.

## Theory

`CustomFactor` is a `NonlinearFactor` that has a `std::function` as its callback.
This callback can be translated to a Python function call, thanks to `pybind11`'s functional support.

## Usage

In order to use a Python-based factor, one needs to have a Python function with the following signature:

```python
import gtsam
import numpy as np
from typing import List

def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    ...
```

`this` is a reference to the `CustomFactor` object. This is required because one can reuse the same
`error_func` for multiple factors. `v` is a reference to the current set of values, and `H` is a list of
**references** to the list of required Jacobians (see the corresponding C++ documentation).

If `H` is `None`, it means the current factor evaluation does not need Jacobians. For example, the `error`
method on a factor does not need Jacobians, so we don't evaluate them to save CPU. If `H` is not `None`,
each entry of `H` can be assigned a `numpy` array, as the Jacobian for the corresponding variable.

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

def error_func(this: gtsam.CustomFactor, v: gtsam.Values, H: List[np.ndarray]):
    # Get the variable values from `v`
    key0 = this.keys()[0]
    key1 = this.keys()[1]
    
    # Calculate non-linear error
    gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
    error = gtsam.Pose2(0, 0, 0).localCoordinates(gT1.between(gT2))

    # If we need Jacobian
    if H is not None:
        # Fill the Jacobian arrays
        # Note we have two vars, so two entries
        result = gT1.between(gT2)
        H[0] = -result.inverse().AdjointMap()
        H[1] = np.eye(3)
    
    # Return the error
    return error

noise_model = gtsam.noiseModel.Unit.Create(3)
cf = gtsam.CustomFactor(noise_model, gtsam.KeyVector([0, 1]), error_func)
```

In general, the Python-based factor works just like their C++ counterparts.

## Known Issues

Because of the `pybind11`-based translation, the performance of `CustomFactor` is not guaranteed.
Also, because `pybind11` needs to lock the Python GIL lock for evaluation of each factor, parallel
evaluation of `CustomFactor` is not possible.
