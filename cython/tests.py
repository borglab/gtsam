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
r4.print_(b'r4: ')
r3.print_(b"r3: ")

v = r3.localCoordinates(r4)
print("localCoordinates:", v)

Rmat = np.array([
    [0.990074, -0.0942928, 0.104218], 
    [0.104218,  0.990074, -0.0942928], 
    [-0.0942928, 0.104218, 0.990074]
    ])
r5 = Rot3.Rot3_1(Rmat)
r5.print_(b"r5: ")

l = Rot3.Logmap(r5)
print("l = ", l)


noise = noiseModel_Gaussian.Covariance(Rmat)
noise.print_(b"noise:")

D = np.array([1.,2.,3.])
diag = noiseModel_Diagonal.Variances(D)
print("diag:", diag)
diag.print_(b"diag:")
print("diag R:", diag.R())

p = Point3()
p.print_("p:")
factor = BetweenFactorPoint3.BetweenFactorPoint3(1,2,p, noise)
factor.print_(b"factor:")

vv = VectorValues()
vv.print_(b"vv:")
vv.insert(1, np.array([1.,2.,3.]))
vv.insert(2, np.array([3.,4.]))
vv.insert(3, np.array([5.,6.,7.,8.]))
vv.print_(b"vv:")

vv2 = VectorValues.VectorValues_1(vv)
vv2.insert(4, np.array([4.,2.,1]))
vv2.print_(b"vv2:")
vv.print_(b"vv:")

vv.insert(4, np.array([1.,2.,4.]))
vv.print_(b"vv:")
vv3 = vv.add(vv2)

vv3.print_(b"vv3:")

values = Values()
values.insertPoint3(1, Point3())
values.insertRot3(2, Rot3())
values.print_(b"values:")

factor = PriorFactorVector.PriorFactorVector(1, np.array([1.,2.,3.]), diag)
factor.print_("Prior factor vector: ")