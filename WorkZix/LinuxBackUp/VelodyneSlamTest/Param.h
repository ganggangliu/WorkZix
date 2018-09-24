#include "OpenCVInc.h"

class VelodyneParam
{
public:
    VelodyneParam();

    double dTreadSlope;     //road edge slope threashold  (degree)
    cv::Point3d CalibT;     //calibrate translation
    cv::Point3d CalibR;     //calibrate rotation
    double dVehicleWidthHalf;   //mm
    double dVehicleLen;     //mm
    double dPathChangeToleratePer; //percent[0-1]
    double dPathChangeDist; //m
    int nStableCont;
    int nStableValid;
};
