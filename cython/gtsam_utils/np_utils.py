import numpy as np

def Vector(list1d): return np.array(list1d, dtype = 'float')
def Matrix(list2d): return np.array(list2d, dtype = 'float', order = 'F')