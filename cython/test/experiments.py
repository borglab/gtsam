"""
This file contains small experiments to test the wrapper with gtsam_short,
not real unittests. Its name convention is different from other tests so it
won't be discovered.
"""
from gtsam.gtsam import *
import numpy as np
from gtsam_utils import Vector, Matrix

r = Rot3()
print(r)
print(r.pitch())
r2 = Rot3()
r3 = r.compose(r2)
print("r3 pitch:", r3.pitch())

v = Vector(1, 1, 1) 
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
r5 = Rot3(Rmat)
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
factor = BetweenFactorPoint3(1,2,p, noise)
factor.print_(b"factor:")

vv = VectorValues()
vv.print_(b"vv:")
vv.insert(1, np.array([1.,2.,3.]))
vv.insert(2, np.array([3.,4.]))
vv.insert(3, np.array([5.,6.,7.,8.]))
vv.print_(b"vv:")

vv2 = VectorValues(vv)
vv2.insert(4, np.array([4.,2.,1]))
vv2.print_(b"vv2:")
vv.print_(b"vv:")

vv.insert(4, np.array([1.,2.,4.]))
vv.print_(b"vv:")
vv3 = vv.add(vv2)

vv3.print_(b"vv3:")

values = Values()
values.insert(1, Point3())
values.insert(2, Rot3())
values.print_(b"values:")

factor = PriorFactorVector(1, np.array([1.,2.,3.]), diag)
print "Prior factor vector: ", factor



keys = KeyVector()

keys.push_back(1)
keys.push_back(2)
print 'size: ', keys.size()
print keys.at(0)
print keys.at(1)

noise = noiseModel_Isotropic.Precision(2, 3.0)
noise.print_('noise:')
print 'noise print:', noise
f = JacobianFactor(7, np.ones([2,2]), model=noise,  b=np.ones(2))
print 'JacobianFactor(7):\n', f
print "A = ", f.getA()
print "b = ", f.getb()

f = JacobianFactor(np.ones(2))
f.print_('jacoboian b_in:')


print "JacobianFactor initalized with b_in:", f

diag = noiseModel_Diagonal.Sigmas(np.array([1.,2.,3.]))
fv = PriorFactorVector(1, np.array([4.,5.,6.]), diag)
print "priorfactorvector: ", fv

print "base noise: ", fv.get_noiseModel()
print "casted to gaussian2: ", dynamic_cast_noiseModel_Diagonal_noiseModel_Base(fv.get_noiseModel())

X = symbol(65, 19)
print X 
print symbolChr(X)
print symbolIndex(X)
