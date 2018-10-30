These examples are almost identical to the old handwritten python wrapper
examples. However, there are just some slight name changes, for example
noiseModel.Diagonal becomes noiseModel_Diagonal etc...
Also, annoyingly, instead of gtsam.Symbol('b',0) we now need to say gtsam.symbol(ord('b'), 0))
