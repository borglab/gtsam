from gtsam import *
import numpy as np

r = Rot3()
print(r)
print(r.pitch())
r2 = Rot3()
r3 = r.compose(r2)
print("r3 pitch:", r3.pitch())

v = np.array([.1, .1, .1])
print("v = ", v)
r4 = r3.retract(v)
print("r4 pitch:", r4.pitch())
r4._print(b'r4: ')
r3._print(b"r3: ")

v = r3.localCoordinates(r4)
print("localCoordinates:", v)

Rmat = np.array([
    [0.990074, -0.0942928, 0.104218], 
    [0.104218,  0.990074, -0.0942928], 
    [-0.0942928, 0.104218, 0.990074]
    ])
r5 = Rot3.ctor(Rmat)
r5._print(b"r5: ")

l = Rot3.Logmap(r5)
print("l = ", l)


noise = Gaussian.Covariance(Rmat)
noise._print(b"noise:")

D = np.array([1.,2.,3.])
diag = Diagonal.Variances(D)
print("diag:", diag)
diag._print(b"diag:")
print("diag R:", diag.R())

p = Point3()
factor = BetweenFactorPoint3(1,2,p, noise)
factor._print(b"factor:")

vv = VectorValues()
vv._print(b"vv:")
vv.insert(1, np.array([1.,2.,3.]))
vv.insert(2, np.array([3.,4.]))
vv.insert(3, np.array([5.,6.,7.,8.]))
vv._print(b"vv:")

vv2 = VectorValues.ctor2(vv)
vv2.insert(4, np.array([4.,2.,1]))
vv2._print(b"vv2:")
vv._print(b"vv:")

vv.insert(4, np.array([1.,2.,4.]))
vv._print(b"vv:")
vv3 = vv.add(vv2)

vv3._print(b"vv3:")

values = Values()
values.insertPoint3(1, Point3())
values.insertRot3(2, Rot3())
values._print(b"values:")