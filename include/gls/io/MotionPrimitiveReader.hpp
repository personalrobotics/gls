/* Authors: Matt Schmittle */

#ifndef GLS_IO_MOTIONPRIMITIVEREADER_HPP_
#define GLS_IO_MOTIONPRIMITIVEREADER_HPP_

#include <fstream>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>

namespace gls {
namespace io {

#define PI_CONST 3.141592653589793238462643383279502884
#define CONTXY2DISC(X, CELLSIZE) (((X)>=0)?((int)((X)/(CELLSIZE))):((int)((X)/(CELLSIZE))-1))    
#define DISCXY2CONT(X, CELLSIZE) ((X)*(CELLSIZE) + (CELLSIZE)/2.0)  

struct pt_xyt{
    float x;
    float y;
    float theta;
};

struct cell_xyt{
    int x;
    int y;
    int theta;
};

struct MotionPrimitive{
    int motprimID;
    int starttheta_c;
    int additionalactioncostmult;
    float turning_radius;
    std::vector<pt_xyt> intermptV;
    cell_xyt endcell;
    double length;
};

class MotionPrimitiveReader{
    public:
        MotionPrimitiveReader(){};
        bool ReadMotionPrimitives(const char* filename); // main api function

        float resolution;
        int NumThetaDirs;
        bool bUseNonUniformAngles;
        std::vector<float> ThetaDirs;
        std::vector<MotionPrimitive> mprimV; // primitives

        int ContTheta2DiscNew(double theta);
        double DiscTheta2ContNew(int theta);
    private:
        bool ReadinPose(pt_xyt* pose, FILE* fIn);
        double DiscTheta2ContFromSet(int theta);
        int ContTheta2DiscFromSet(double theta);

        bool ReadinCell(cell_xyt* cell,FILE* fIn);
        bool ReadinMotionPrimitive(MotionPrimitive* pMotPrim,FILE* fIn);
};

double normalizeAngle(double angle);
int normalizeDiscAngle(int theta, int numThetaDirs, bool bUseNonUniformAngles);
double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS);
int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS);

} // namespace io
} // namespace gls

#endif // GLS_IO_MOTIONPRIMITIVEREADER_HPP_
