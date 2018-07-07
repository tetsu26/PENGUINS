# -*- coding: utf-8 -*-
import pyproj

lat = 35.679933
lon = 139.714465

EPSG4612 = pyproj.Proj("+init=EPSG:4612")
EPSG2451 = pyproj.Proj("+init=EPSG:2451")
y,x = pyproj.transform(EPSG4612, EPSG2451, lon,lat)

print "x = %s" % x
print "y = %s" % y