# Example of wrapping cxx using ctypes

More info on ctypes in python here: <https://docs.python.org/3/extending/extending.html#writing-extensions-in-c>

```bash
# prerequisites
sudo apt install libgeographiclib-dev python3-dev

# bulding lib under Ubuntu
g++ -fPIC -O2 -pipe -shared -o intersect_dso.so PyGetIntersect.cpp -lGeographicLib -Wl,-rpath=/usr/lib/x86_64-linux-gnu
```

## Notes

From <https://geographiclib.sourceforge.io/C++/doc/IntersectTool.1.html>:

> The intersection is then specified as the displacements, x and y, along
> the geodesics X and Y from the starting points to the intersection.

So, you will need to solve direct geodesic problem to get intersection coordinates given `PyGetIntersect.intersect()` output, e.g.:

```python
# translated from <https://geographiclib.sourceforge.io/C++/doc/IntersectTool_8cpp_source.html>
import geographiclib.geodesic
from PyGetIntersect import intersect

# from Berlin, straight to the East, and from Krakow straight to the North
lat1, lon1, az1 = 52.51052876689272, 13.427111890069114, 90
lat2, lon2, az2 = 50.05006131270985, 19.939164396314474, 0

dx, dy = intersect(lat1, lon1, az1, lat2, lon2, az2)

geod = geographiclib.geodesic.Geodesic.WGS84
line = geod.Line(lat1, lon1, az1)
geo_dict = line.Position(dx)

intersection_coords = [geo_dict['lat2'], geo_dict['lon2']]

print(dx, dy)
print(intersection_coords)

# output
# 443330.70575374854 253739.9751311709
# [52.3308313176463, 19.939164396314474]
```
