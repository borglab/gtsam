import ctypes

intersect_dso = ctypes.CDLL('./intersect_dso.so')
intersect_c = intersect_dso.intersect
intersect_c.argtypes = ([ctypes.c_double] * 6) + ([ctypes.POINTER(ctypes.c_double)] * 2)

def intersect(lat1, lon1, az1, lat2, lon2, az2):
    dX = ctypes.c_double()
    dY = ctypes.c_double()

    intersect_c(
        lat1, lon1, az1, lat2, lon2, az2,
        ctypes.byref(dX), ctypes.byref(dY),
    )

    return dX.value, dY.value
