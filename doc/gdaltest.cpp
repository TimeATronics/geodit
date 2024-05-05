#include <cstdlib>
#include <gdal/gdal_priv.h>
#include <gdal/ogr_core.h>
#include <gdal/ogr_spatialref.h>
#include <errno.h>
#include <stdlib.h>

const char *filename = "./doc/band1.tif";

double* pixel2coord(int x, int y, const char* filename) {
    const char* pszFilename = filename;
    GDALDatasetUniquePtr poDataset;
    GDALAllRegister();
    const GDALAccess eAccess = GA_ReadOnly;
    
    poDataset = GDALDatasetUniquePtr(GDALDataset::FromHandle(GDALOpen( pszFilename, eAccess )));
    if (!poDataset) {
        printf("Open failed.\n");
        exit(EXIT_FAILURE);
    }

    static double out[2];
    double  GT[6];
    if (poDataset->GetGeoTransform(GT) == CE_None) {
        //printf("Origin = (%.6f,%.6f)\n", GT[0], GT[3]);
        //printf("Pixel Size = (%.6f,%.6f)\n", GT[1], GT[5]);
    }
    double xoff = GT[0];
    double a = GT[1];
    double b = GT[2];
    double yoff = GT[3];
    double d = GT[4];
    double e = GT[5];
    
    double xp = a * x + b * y + xoff;
    double yp = d * x + e * y + yoff;
    printf("%f, %f\n", xp, yp);
    out[0] = xp; out[1] = yp;

    OGRSpatialReference source = OGRSpatialReference();
    source.importFromWkt(poDataset->GetProjectionRef());
    OGRSpatialReference target = OGRSpatialReference();
    target.importFromEPSG(4326);
    OGRCoordinateTransformation* coord = OGRCreateCoordinateTransformation(&source, &target);
    coord->SetEmitErrors(true);
    if (!coord) {
        return out;
    } else {
        double m = xp; double n = yp;
        coord->Transform(1, &m, &n, nullptr, nullptr);
        static double out1[2];
        out1[0] = m; out1[1] = n;
        return out1;
    }
}

int main(int argc, const char* argv[]) {
    int row = 334;
    int col = 380;
    double* out = pixel2coord(col, row, filename);
    printf("%f, %f\n", out[0], out[1]);
    return 0;
}