from osgeo import gdal, ogr, osr

ds = gdal.Open('./doc/sample.tif')
xoff, a, b, yoff, d, e = ds.GetGeoTransform()

def pixel2coord(x, y):
    xp = a * x + b * y + xoff
    yp = d * x + e * y + yoff
    return(xp, yp)

rows = 334
colms = 380
#for row in range(0,rows):
#    for col in range(0,colms):
mn = pixel2coord(colms,rows)
print(mn[0], mn[1])

source = osr.SpatialReference()
source.ImportFromWkt(ds.GetProjection())
target = osr.SpatialReference()
target.ImportFromEPSG(4326)
transform = osr.CoordinateTransformation(source, target)
print(transform.TransformPoint(mn[0], mn[1]))
