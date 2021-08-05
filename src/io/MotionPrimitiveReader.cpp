/* Authors: Matt Schmittle */

#include "gls/io/MotionPrimitiveReader.hpp"

namespace gls {
namespace io {

bool MotionPrimitiveReader::ReadinPose(
    pt_xyt* pose,
    FILE* fIn)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->x = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->y = atof(sTemp);
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    pose->theta = atof(sTemp);

    pose->theta = normalizeAngle(pose->theta);

    return true;
}

double MotionPrimitiveReader::DiscTheta2ContNew(int theta)
{
    if (bUseNonUniformAngles) {
        return DiscTheta2ContFromSet(theta);
    }
    else {
        return DiscTheta2Cont(theta, NumThetaDirs);
    }
}

int MotionPrimitiveReader::ContTheta2DiscNew(double theta)
{
    if (bUseNonUniformAngles) {
        return ContTheta2DiscFromSet(theta);
    }
    else {
        return ContTheta2Disc(theta, NumThetaDirs);
    }
}

double MotionPrimitiveReader::DiscTheta2ContFromSet(int theta)
{
    theta = normalizeDiscAngle(theta, NumThetaDirs, bUseNonUniformAngles);

    // ThetaDirs should contain extra angle (2PI) for overlap
    if (NumThetaDirs >= (int)ThetaDirs.size()) {
        std::cout<<"ERROR: list of bin angles are not properly set to use function DiscTheta2ConfFromSet" << std::endl;
    }

    if (theta > NumThetaDirs || theta < 0) {
        std::cout << "ERROR: discrete value theta " << theta << " out of range"<<std::endl;
    }
    return ThetaDirs[theta];
}

int MotionPrimitiveReader::ContTheta2DiscFromSet(double theta)
{
    theta = normalizeAngle(theta);
    // ThetaDirs should contain extra angle (2PI) for overlap
    if (NumThetaDirs >= (int) ThetaDirs.size()) {
        std::cout<<"ERROR: list of bin angles are not properly set to use function ContTheta2DiscFromSet" << std::endl;
    }

    int lower_bound_ind = -1;
    int upper_bound_ind = -1;
    for (int i = 1; i < (int) ThetaDirs.size(); i++) {
        if ((ThetaDirs[i]) >= theta) {
            lower_bound_ind = i - 1;
            upper_bound_ind = i;
            break;
        }
    }

    // Critical error if could not find bin location from given angle
    if (lower_bound_ind == -1) {
        std::cout << "ERROR: unable to find bin index for angle " << theta<<std::endl;
    }

    // Get closest angle of two
    double angle_low = ThetaDirs[lower_bound_ind];
    double angle_up = ThetaDirs[upper_bound_ind];
    double diff_low = fabs(theta - angle_low);
    double diff_up = fabs(theta - angle_up);

    if (diff_low < diff_up) {
        return lower_bound_ind;
    }
    else {
        // Wrap upper bound index around when it reaches last index (assumed to be 2PI)
        if (upper_bound_ind == NumThetaDirs) {
            upper_bound_ind = 0;
        }
        return upper_bound_ind;
    }
}

bool MotionPrimitiveReader::ReadinCell(
    cell_xyt* cell,
    FILE* fIn, 
    std::string &metaID)
{
    char sTemp[60];

    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->x = atoi(sTemp);
    metaID += sTemp;
    metaID += ", ";
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->y = atoi(sTemp);
    metaID += sTemp;
    metaID += ", ";
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    cell->theta = atoi(sTemp);
    metaID += sTemp;
    metaID += ")";

    // normalize the angle
    cell->theta = normalizeDiscAngle(cell->theta, NumThetaDirs, bUseNonUniformAngles);

    return true;
}

bool MotionPrimitiveReader::ReadinMotionPrimitive(
    MotionPrimitive* pMotPrim,
    FILE* fIn,
    json metaPrims)
{
    char sTemp[1024];
    int dTemp;
    char sExpected[1024];
    int numofIntermPoses;
    float fTemp;
    std::string metaID = "(";

    // read in actionID
    strcpy(sExpected, "primID:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<<sTemp<<std::endl;
        fflush(stdout);
        return false;
    }
    if (fscanf(fIn, "%d", &pMotPrim->motprimID) != 1) {
        return false;
    }

    // read in start angle
    strcpy(sExpected, "startangle_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<<sTemp<<std::endl;
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) == 0) {
        std::cout<<"ERROR reading startangle\n"<<std::endl;
        return false;
    }
    pMotPrim->starttheta_c = dTemp;
    metaID += std::to_string(dTemp);
    metaID +=", ";

    // read in end pose
    strcpy(sExpected, "endpose_c:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<< sTemp<<std::endl;
        return false;
    }

    if (ReadinCell(&pMotPrim->endcell, fIn, metaID) == false) {
        std::cout<<"ERROR: failed to read in endsearchpose\n"<<std::endl;
        return false;
    }

    // read in action cost
    strcpy(sExpected, "additionalactioncostmult:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<< sTemp<<std::endl;
        return false;
    }
    if (fscanf(fIn, "%d", &dTemp) != 1) {
        return false;
    }
    pMotPrim->additionalactioncostmult = dTemp;

    if (bUseNonUniformAngles) {
        // read in action turning radius
        strcpy(sExpected, "turning_radius:");
        if (fscanf(fIn, "%s", sTemp) == 0) {
            return false;
        }
        if (strcmp(sTemp, sExpected) != 0) {
            std::cout<<"ERROR: expected "<< sExpected<<" but got "<<sTemp<<std::endl;
            return false;
        }
        if (fscanf(fIn, "%f", &fTemp) != 1) {
            return false;
        }
        pMotPrim->turning_radius = fTemp;
    }

    // read in intermediate poses
    strcpy(sExpected, "intermediateposes:");
    if (fscanf(fIn, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<< sTemp<<std::endl;
        return false;
    }
    if (fscanf(fIn, "%d", &numofIntermPoses) != 1) {
        return false;
    }
    // all intermposes should be with respect to 0,0 as starting pose since it
    // will be added later and should be done after the action is rotated by
    // initial orientation
    for (int i = 0; i < numofIntermPoses; i++) {
        pt_xyt intermpose;
        if (ReadinPose(&intermpose, fIn) == false) {
            std::cout<<"ERROR: failed to read in intermediate poses\n"<<std::endl;
            return false;
        }
        pMotPrim->intermptV.push_back(intermpose);
    }

    // Check that the last pose of the motion matches (within lattice
    // resolution) the designated end pose of the primitive
    pt_xyt sourcepose;
    sourcepose.x = DISCXY2CONT(0, resolution);
    sourcepose.y = DISCXY2CONT(0, resolution);
    sourcepose.theta = DiscTheta2ContNew(pMotPrim->starttheta_c);
    double mp_endx_m = sourcepose.x + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].x;
    double mp_endy_m = sourcepose.y + pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].y;
    double mp_endtheta_rad = pMotPrim->intermptV[pMotPrim->intermptV.size() - 1].theta;

    int endtheta_c;
    int endx_c = CONTXY2DISC(mp_endx_m, resolution);
    int endy_c = CONTXY2DISC(mp_endy_m, resolution);
    endtheta_c = ContTheta2DiscNew(mp_endtheta_rad);
    if (endx_c != pMotPrim->endcell.x ||
        endy_c != pMotPrim->endcell.y ||
        endtheta_c != pMotPrim->endcell.theta)
    {
        std::cout<<"ERROR: incorrect primitive last interm point does not match end pose\n"<<std::endl;
        return false;
    }

    pMotPrim->length = std::fabs(metaPrims[metaID]["path_len"].get<float>());

    return true;
}

bool MotionPrimitiveReader::ReadMotionPrimitives(const char* mprim_filename, const char* json_filename)
{
    FILE* fMotPrims = fopen(mprim_filename, "r"); 
    std::ifstream fMetadataPrims(json_filename);
    json metaPrims;
    fMetadataPrims >> metaPrims;

    char sTemp[1024], sExpected[1024];
    float fTemp;
    int dTemp;
    int totalNumofActions = 0;
    bUseNonUniformAngles = false; // default

    std::cout<<"Reading in motion primitives..." << std::endl;
    fflush(stdout);

    //read in the resolution
    strcpy(sExpected, "resolution_m:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<< "ERROR: expected "<< sExpected<< " but got "<< sTemp << std::endl;
        fflush(stdout);
        return false;
    }
    if (fscanf(fMotPrims, "%f", &fTemp) == 0) {
        return false;
    }
    std::cout<< "resolution_m: "<< fTemp << std::endl;
    resolution = fTemp;

    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strncmp(sTemp, "min_turning_radius_m:", 21) == 0) {
        bUseNonUniformAngles = true;
    }
    std::cout<< "bUseNonUniformAngles = " << bUseNonUniformAngles << std::endl;

    if (bUseNonUniformAngles) {
        float min_turn_rad;
        strcpy(sExpected, "min_turning_radius_m:");
        if (strcmp(sTemp, sExpected) != 0) {
            std::cout << "ERROR: expected "<< sExpected <<" but got " << sTemp <<std::endl;
            fflush(stdout);
            return false;
        }
        if (fscanf(fMotPrims, "%f", &min_turn_rad) == 0) {
            return false;
        }
        std::cout<< "min_turn_rad: " << min_turn_rad <<std::endl;
        fflush(stdout);
        if (fscanf(fMotPrims, "%s", sTemp) == 0) {
            return false;
        }
    }

    // read in the angular resolution
    strcpy(sExpected, "numberofangles:");
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<< "ERROR: expected " << sExpected<< " but got " << sTemp <<std::endl;
       return false;
    }
    if (fscanf(fMotPrims, "%d", &dTemp) == 0) {
        return false;
    }
    NumThetaDirs = dTemp;

    if (bUseNonUniformAngles) {
        // read in angles
        ThetaDirs.clear();
        for (int i = 0; i < NumThetaDirs; i++)
        {
            std::ostringstream string_angle_index;
            string_angle_index << i;
            std::string angle_string = "angle:" + string_angle_index.str();

            float angle;
            strcpy(sExpected, angle_string.c_str());
            if (fscanf(fMotPrims, "%s", sTemp) == 0) {
                return false;
            }
            if (strcmp(sTemp, sExpected) != 0) {
                std::cout<<"ERROR: expected "<< sExpected <<" but got "<< sTemp<<std::endl;
                return false;
            }
            if (fscanf(fMotPrims, "%f", &angle) == 0) {
                return false;
            }
            std::cout<<angle_string.c_str()<<" "<< angle<<std::endl;
            ThetaDirs.push_back(angle);
        }
        ThetaDirs.push_back(2.0 * PI_CONST); // Add 2 PI at end for overlap
    }

    // read in the total number of actions
    strcpy(sExpected, "totalnumberofprimitives:");
    if (fscanf(fMotPrims, "%s", sTemp) == 0) {
        return false;
    }
    if (strcmp(sTemp, sExpected) != 0) {
        std::cout<<"ERROR: expected "<<sExpected<<" but got "<< sTemp<<std::endl;
        return false;
    }
    if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
        return false;
    }
    std::cout<<"totalnumberofprimitives: "<< totalNumofActions<<std::endl;

    // Read in motion primitive for each action
    for (int i = 0; i < totalNumofActions; i++) {
        MotionPrimitive motprim;

        if (!ReadinMotionPrimitive(&motprim, fMotPrims, metaPrims) ){
            return false;
        }

        mprimV.push_back(motprim);
    }
    return true;
}

double normalizeAngle(double angle)
{
    double retangle = angle;

    //get to the range from -2PI, 2PI
    if (fabs(retangle) > 2 * PI_CONST) retangle = retangle - ((int)(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

    //get to the range 0, 2PI
    if (retangle < 0) retangle += 2 * PI_CONST;

    if (retangle < 0 || retangle > 2 * PI_CONST) {
        std::cout<<"ERROR: after normalization of angle="<<angle<<" we get angle="<<retangle <<std::endl;
    }

    return retangle;
}

int normalizeDiscAngle(int theta, int numThetaDirs, bool bUseNonUniformAngles)
{
    if (bUseNonUniformAngles) {
        if (theta < 0) {
            theta += numThetaDirs;
        }
        if (theta >= numThetaDirs) {
            theta -= numThetaDirs;
        }
    }
    else {
        theta = theta >= 0 ? theta%numThetaDirs : (theta%numThetaDirs + numThetaDirs)%numThetaDirs;
    }
    return theta;
}

//converts discretized version of angle into continuous (radians)
//maps 0->0, 1->delta, 2->2*delta, ...
double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS)
{
    double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
    return nTheta * thetaBinSize;
}

//converts continuous (radians) version of angle into discrete
//maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS)
{
    double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
    return (int)(normalizeAngle(fTheta + thetaBinSize / 2.0) / (2.0 * PI_CONST) * (NUMOFANGLEVALS));
}



} // namespace io
} // namespace gls
